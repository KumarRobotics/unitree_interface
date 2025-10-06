#include "unitree_teleop/unitree_teleop.hpp"

UnitreeTeleop::UnitreeTeleop() 
    : Node("unitree_teleop"),
      sport_client_(this),
      twist_buf_(),
      is_auto_(false),
      sit_transition_(false),
      is_armed_(false)
{
    // subscribe to joy
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 
        10,
        std::bind(&UnitreeTeleop::joy_callback, this, std::placeholders::_1));
    // subscribe to twist commands
    twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "twist_auto", 
        10,
        std::bind(&UnitreeTeleop::twist_callback, this, std::placeholders::_1));
    // publisher for auto flag
    is_auto_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        "~/is_auto", 
        10);
    
    twist_buf_ = geometry_msgs::msg::Twist();
}

void UnitreeTeleop::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    // auto axis 4
    // publish auto flag
    auto is_auto_msg = std_msgs::msg::Bool();
    is_auto_msg.data = is_auto_;
    is_auto_pub_->publish(is_auto_msg);
    
    // arm axis 6
    int arming_axis = msg->axes[6];
    if (arming_axis < 0)
    {
        if (!is_armed_)
	    {
            is_armed_ = true;
            sport_client_.BalanceStand(req_);
        }
    }
    else if (arming_axis == 0)
    {
        is_armed_ = false;
        sport_client_.StandUp(req_);
    }
    else if (arming_axis > 0)
    {
        is_armed_ = false;
        sport_client_.StandDown(req_);
    }

    if (!is_armed_)
    {
        // we don't send any vel cmds when not armed
        return;
    }

    // map joystick axes to twist
    // left stick vertical    (axis 0) -> ignored
    // left stick horizontal  (axis 1) -> linear.y  (strafe left/right) [-1, 1]
    // right stick horizontal (axis 2) -> angular.z (turn left/right)   [-1, 1]
    // right stick vertical   (axis 3) -> linear.x  (forward/backward)  [-1, 1]
    
    // linear velocity
    float right_vert_axis = msg->axes[3];
    float max_linear_vel_x = 1.0;
    float linear_vel_x = right_vert_axis * max_linear_vel_x;

    float left_horiz_axis = msg->axes[1];
    float max_linear_vel_y = 0.2;
    float linear_vel_y = left_horiz_axis * max_linear_vel_y;
    twist_buf_.linear.x = linear_vel_x;
    twist_buf_.linear.y = linear_vel_y;
    twist_buf_.linear.z = 0.0;

    // angular velocity
    float right_horiz_axis = msg->axes[2];
    float max_angular_vel_z = 1.0;
    float angular_vel_z = right_horiz_axis * max_angular_vel_z;
    
    twist_buf_.angular.x = 0.0;
    twist_buf_.angular.y = 0.0;
    twist_buf_.angular.z = angular_vel_z;

    is_auto_ = msg->axes[4] > 0.0;

    if (!is_auto_)
    {
        // only send teleop cmd when we receive joystick input
        sport_client_.Move(req_, twist_buf_.linear.x, twist_buf_.linear.y, twist_buf_.angular.z);
    }

    // stand up/down axis 5
    int sit_axis = msg->axes[5];
    if (sit_axis == 0)
    {
        sit_transition_ = true;
    }
    if (sit_axis > 0 && sit_transition_)
    {
        sport_client_.StandDown(req_);
        sit_transition_ = false;
    }
    else if (sit_axis < 0 && sit_transition_)
    {
        sport_client_.StandUp(req_);
        sit_transition_ = false;
    }
}

void UnitreeTeleop::twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    // forward twist cmd when in auto mode
    if (is_auto_ && is_armed_)
    {
        sport_client_.Move(req_, msg->linear.x, msg->linear.y, msg->angular.z);
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
