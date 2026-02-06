#include "unitree_teleop/unitree_teleop.hpp"

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
    network_interface_ = this->get_parameter("network_interface").as_string();

    // Initialize unitree_sdk2 DDS channel
    RCLCPP_INFO(this->get_logger(), "Initializing unitree_sdk2 ChannelFactory on interface: %s", network_interface_.c_str());
    unitree::robot::ChannelFactory::Instance()->Init(0, network_interface_);

    // Initialize sport client
    sport_client_.SetTimeout(10.0f);
    sport_client_.Init();

    // Subscribe to joy
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy",
        10,
        std::bind(&UnitreeTeleop::joy_callback, this, std::placeholders::_1));

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
            int32_t ret = sport_client_.BalanceStand();
            RCLCPP_INFO(this->get_logger(), "BalanceStand: %d", ret);
        }
    }
    else if (arming_axis == 0)
    {
        is_armed_ = false;
        int32_t ret = sport_client_.StandUp();
        RCLCPP_DEBUG(this->get_logger(), "StandUp: %d", ret);
    }
    else if (arming_axis > 0)
    {
        is_armed_ = false;
        int32_t ret = sport_client_.StandDown();
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
            int32_t ret = sport_client_.SetGait(0);
            gait_type_ = 0;
            RCLCPP_INFO(this->get_logger(), "SetGait(0) regular: %d", ret);
        }
    }
    else if (gait_axis == 0)
    {
        // Climb gait
        if (gait_type_ != 2)
        {
            int32_t ret = sport_client_.SetGait(2);
            gait_type_ = 2;
            RCLCPP_INFO(this->get_logger(), "SetGait(2) climb: %d", ret);
        }
    }
    else if (gait_axis == -1)
    {
        // Terrain gait
        if (gait_type_ != 1)
        {
            int32_t ret = sport_client_.SetGait(1);
            gait_type_ = 1;
            RCLCPP_INFO(this->get_logger(), "SetGait(1) terrain: %d", ret);
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
        sport_client_.Move(twist_buf_.linear.x, twist_buf_.linear.y, twist_buf_.angular.z);
    }
}

void UnitreeTeleop::twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    // Forward twist cmd when in auto mode
    if (is_auto_ && is_armed_)
    {
        sport_client_.Move(msg->linear.x, msg->linear.y, msg->angular.z);
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
