#include "unitree_teleop/unitree_teleop.hpp"
#include <unitree/robot/channel/channel_factory.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

UnitreeTeleop::UnitreeTeleop()
    : Node("unitree_teleop"),
      twist_buf_(),
      is_auto_(false),
      sit_transition_(false),
      is_armed_(false),
      gait_type_(0)
{
    // Declare and get parameters
    this->declare_parameter<std::string>("network_interface", "eth0");
    std::string network_interface = this->get_parameter("network_interface").as_string();

    // Initialize unitree_sdk2 DDS channel
    RCLCPP_INFO(this->get_logger(), "Initializing unitree_sdk2 ChannelFactory on interface: %s", network_interface.c_str());
    unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);

    // Initialize sport client (ChannelFactory must be initialized first)
    sport_client_ = std::make_unique<SportClientExt>();
    sport_client_->SetTimeout(10.0f);
    sport_client_->Init();

    // Input source: "joy" (FrSky USB receiver via joy_node) or "mavros" (RCIn from the flight controller)
    this->declare_parameter<std::string>("input_source", "joy");
    std::string input_source = this->get_parameter("input_source").as_string();

    // RCIn -> Joy mapping. Entry i gives the 1-indexed RC channel for joy axis i.
    // Defaults assume the FC receiver is bound to the same transmitter model as the USB
    // receiver, which exposes channels 1..N as axes 0..N-1. joy_node inverts SDL axes,
    // so all channels are reversed by default to reproduce its sign convention.
    this->declare_parameter<std::vector<int64_t>>("rc_axis_channels", {1, 2, 3, 4, 5, 6, 7});
    this->declare_parameter<std::vector<bool>>("rc_axis_reversed", {true, true, true, true, true, true, true});
    // Joy axes driven by switches; these are quantised to exactly -1/0/1
    this->declare_parameter<std::vector<int64_t>>("rc_switch_axes", {4, 5, 6});
    this->declare_parameter<double>("rc_pwm_center", 1500.0);
    this->declare_parameter<double>("rc_pwm_half_range", 500.0);
    this->declare_parameter<double>("rc_deadzone", 0.1);
    this->declare_parameter<double>("rc_timeout", 0.5);
    rc_axis_channels_ = this->get_parameter("rc_axis_channels").as_integer_array();
    rc_axis_reversed_ = this->get_parameter("rc_axis_reversed").as_bool_array();
    rc_switch_axes_ = this->get_parameter("rc_switch_axes").as_integer_array();
    rc_pwm_center_ = this->get_parameter("rc_pwm_center").as_double();
    rc_pwm_half_range_ = this->get_parameter("rc_pwm_half_range").as_double();
    rc_deadzone_ = this->get_parameter("rc_deadzone").as_double();
    rc_timeout_ = this->get_parameter("rc_timeout").as_double();
    last_rc_time_ = this->now();
    rc_timed_out_ = true;

    if (input_source == "mavros")
    {
        if (rc_axis_channels_.size() < 7 || rc_axis_reversed_.size() != rc_axis_channels_.size())
        {
            throw std::invalid_argument(
                "rc_axis_channels needs >= 7 entries and rc_axis_reversed must match its length");
        }

        // Subscribe to RCIn (mavros publishes this best-effort)
        rc_in_sub_ = this->create_subscription<mavros_msgs::msg::RCIn>(
            "rc_in",
            rclcpp::SensorDataQoS(),
            std::bind(&UnitreeTeleop::rc_in_callback, this, std::placeholders::_1));

        rc_watchdog_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&UnitreeTeleop::rc_watchdog_callback, this));

        RCLCPP_INFO(this->get_logger(), "Input source: mavros RCIn on %s", rc_in_sub_->get_topic_name());
    }
    else if (input_source == "joy")
    {
        // Subscribe to joy
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy",
            10,
            std::bind(&UnitreeTeleop::joy_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Input source: joy on %s", joy_sub_->get_topic_name());
    }
    else
    {
        throw std::invalid_argument("input_source must be 'joy' or 'mavros', got '" + input_source + "'");
    }

    // Subscribe to twist commands
    twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "twist_auto",
        10,
        std::bind(&UnitreeTeleop::twist_callback, this, std::placeholders::_1));

    // Publisher for auto flag
    is_auto_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        "~/is_auto",
        10);

    twist_buf_ = geometry_msgs::msg::Twist();

    RCLCPP_INFO(this->get_logger(), "UnitreeTeleop (sdk2) initialized.");
}

void UnitreeTeleop::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    // Publish auto flag
    auto is_auto_msg = std_msgs::msg::Bool();
    is_auto_msg.data = is_auto_;
    is_auto_pub_->publish(is_auto_msg);

    // Arm axis 6
    int arming_axis = msg->axes[6];
    if (arming_axis < 0)
    {
        if (!is_armed_)
        {
            is_armed_ = true;
            int32_t ret = sport_client_->BalanceStand();
            RCLCPP_INFO(this->get_logger(), "BalanceStand: %d", ret);
        }
    }
    else if (arming_axis == 0)
    {
        is_armed_ = false;
        int32_t ret = sport_client_->StandUp();
        RCLCPP_DEBUG(this->get_logger(), "StandUp: %d", ret);
    }
    else if (arming_axis > 0)
    {
        is_armed_ = false;
        int32_t ret = sport_client_->StandDown();
        RCLCPP_DEBUG(this->get_logger(), "StandDown: %d", ret);
    }

    if (!is_armed_)
    {
        // Don't send any vel cmds when not armed
        return;
    }

    // Set gait via axis 5
    int gait_axis = msg->axes[5];
    if (gait_axis == 1)
    {
        // Regular gait
        if (gait_type_ != 0)
        {
            int32_t ret = sport_client_->SetGait(0);
            if (ret == 0)
            {
                gait_type_ = 0;
                RCLCPP_INFO(this->get_logger(), "SetGait(0) regular: %d", ret);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "SetGait(0) regular failed: %d", ret);
            }
        }
    }
    else if (gait_axis == 0)
    {
        // Climb gait
        if (gait_type_ != 2)
        {
            int32_t ret = sport_client_->SetGait(2);
            if (ret == 0)
            {
                gait_type_ = 2;
                RCLCPP_INFO(this->get_logger(), "SetGait(2) climb: %d", ret);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "SetGait(2) climb failed: %d", ret);
            }
        }
    }
    else if (gait_axis == -1)
    {
        // Terrain gait
        if (gait_type_ != 1)
        {
            int32_t ret = sport_client_->SetGait(1);
            if (ret == 0)
            {
                gait_type_ = 1;
                RCLCPP_INFO(this->get_logger(), "SetGait(1) terrain: %d", ret);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "SetGait(1) terrain failed: %d", ret);
            }
        }
    }

    // Map joystick axes to twist
    // left stick vertical    (axis 0) -> max velocity scaler
    // left stick horizontal  (axis 1) -> linear.y  (strafe left/right)
    // right stick horizontal (axis 2) -> angular.z (turn left/right)
    // right stick vertical   (axis 3) -> linear.x  (forward/backward)

    // Get max velocity from axis 0 -- map [1.0,-1.0] to [1.0, 2.5]
    float max_linear_vel_x = (-msg->axes[0] + 1.0) + 0.5;

    // Linear velocity
    float right_vert_axis = msg->axes[3];
    float linear_vel_x = right_vert_axis * max_linear_vel_x;

    float left_horiz_axis = msg->axes[1];
    float max_linear_vel_y = 0.2;
    float linear_vel_y = left_horiz_axis * max_linear_vel_y;
    twist_buf_.linear.x = linear_vel_x;
    twist_buf_.linear.y = linear_vel_y;
    twist_buf_.linear.z = 0.0;

    // Angular velocity
    float right_horiz_axis = msg->axes[2];
    float max_angular_vel_z = 1.0;
    float angular_vel_z = right_horiz_axis * max_angular_vel_z;

    twist_buf_.angular.x = 0.0;
    twist_buf_.angular.y = 0.0;
    twist_buf_.angular.z = angular_vel_z;

    is_auto_ = msg->axes[4] > 0.0;

    if (!is_auto_)
    {
        // Only send teleop cmd when we receive joystick input
        sport_client_->Move(twist_buf_.linear.x, twist_buf_.linear.y, twist_buf_.angular.z);
    }
}

void UnitreeTeleop::rc_in_callback(const mavros_msgs::msg::RCIn::SharedPtr msg)
{
    // Convert PWM channels into a Joy message laid out like the FrSky USB receiver
    auto joy = std::make_shared<sensor_msgs::msg::Joy>();
    joy->header = msg->header;
    joy->axes.resize(rc_axis_channels_.size(), 0.0f);

    for (size_t i = 0; i < rc_axis_channels_.size(); ++i)
    {
        int64_t ch = rc_axis_channels_[i];
        if (ch < 1 || static_cast<size_t>(ch) > msg->channels.size())
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "RCIn has %zu channels, axis %zu wants channel %ld; ignoring message",
                msg->channels.size(), i, ch);
            return;
        }

        double value = (static_cast<double>(msg->channels[ch - 1]) - rc_pwm_center_) / rc_pwm_half_range_;
        value = std::clamp(value, -1.0, 1.0);
        if (rc_axis_reversed_[i])
        {
            value = -value;
        }

        bool is_switch = std::find(rc_switch_axes_.begin(), rc_switch_axes_.end(),
                                   static_cast<int64_t>(i)) != rc_switch_axes_.end();
        if (is_switch)
        {
            value = value > 0.5 ? 1.0 : (value < -0.5 ? -1.0 : 0.0);
        }
        else if (std::abs(value) < rc_deadzone_)
        {
            value = 0.0;
        }
        else
        {
            // Rescale outside the deadzone like joy_node does
            value = std::copysign((std::abs(value) - rc_deadzone_) / (1.0 - rc_deadzone_), value);
        }
        joy->axes[i] = static_cast<float>(value);
    }

    if (rc_timed_out_)
    {
        RCLCPP_INFO(this->get_logger(), "RCIn received");
        rc_timed_out_ = false;
    }
    last_rc_time_ = this->now();

    joy_callback(joy);
}

void UnitreeTeleop::rc_watchdog_callback()
{
    // joy_node keeps publishing on its own, but RCIn simply stops if mavros or the FC link drops
    if (rc_timed_out_ || (this->now() - last_rc_time_).seconds() < rc_timeout_)
    {
        return;
    }

    rc_timed_out_ = true;
    RCLCPP_WARN(this->get_logger(), "No RCIn for %.2f s; stopping and leaving auto", rc_timeout_);
    is_auto_ = false;
    if (is_armed_)
    {
        is_armed_ = false;
        int32_t ret = sport_client_->StopMove();
        RCLCPP_WARN(this->get_logger(), "StopMove: %d", ret);
    }

    auto is_auto_msg = std_msgs::msg::Bool();
    is_auto_msg.data = is_auto_;
    is_auto_pub_->publish(is_auto_msg);
}

void UnitreeTeleop::twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    // Forward twist cmd when in auto mode
    if (is_auto_ && is_armed_)
    {
        sport_client_->Move(msg->linear.x, msg->linear.y, msg->angular.z);
    }
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto unitree_teleop = std::make_shared<UnitreeTeleop>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(unitree_teleop);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
