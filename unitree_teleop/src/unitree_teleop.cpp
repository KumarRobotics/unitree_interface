#include "unitree_teleop/unitree_teleop.hpp"

UnitreeTeleop::UnitreeTeleop() 
    : Node("unitree_teleop"),
      twist_buf_(),
      is_auto_(false)
{
    // Initialize twist buffer
    twist_buf_ = geometry_msgs::msg::Twist();

    // Create subscribers
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 
        10,
        std::bind(&UnitreeTeleop::joy_callback, this, std::placeholders::_1));

    twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "twist_auto", 
        10,
        std::bind(&UnitreeTeleop::twist_callback, this, std::placeholders::_1));

    // Create publishers
    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "twist_out", 
        10);

    is_auto_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        "~/is_auto", 
        10);

    // Create timer for publishing twist commands at 100Hz
    pub_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&UnitreeTeleop::pub_callback, this));

    RCLCPP_INFO(this->get_logger(), "Unitree teleop node started");
}

void UnitreeTeleop::pub_callback()
{
    // Timer callback to publish twist commands when not in auto mode
    if (!is_auto_) {
        twist_pub_->publish(twist_buf_);
    }
}

void UnitreeTeleop::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    // map joystick axes to twist commands
    // eft stick vertical (axis 0)     -> ignored
    // left stick horizontal  (axis 1) -> linear.y  (strafe left/right) [-1, 1]
    // right stick horizontal (axis 2) -> angular.z (turn left/right)   [-1, 1]
    // right stick vertical   (axis 3) -> linear.x  (forward/backward)  [-1, 1]
    
    
    if (msg->axes.size() > 3)
    {
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
    }
    
    // Check auto mode button (e.g., left trigger)
    if (msg->axes.size() > 4) {
        is_auto_ = msg->axes[4] > 0.0;
    }

    // Publish auto flag
    auto is_auto_msg = std_msgs::msg::Bool();
    is_auto_msg.data = is_auto_;
    is_auto_pub_->publish(is_auto_msg);
    
    if (is_auto_) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Auto mode enabled");
    } else {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, un
            "Manual mode - Linear: %.2f, Angular: %.2f", 
            twist_buf_.linear.x, twist_buf_.angular.z);
    }
}

void UnitreeTeleop::twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    // Twist callback to forward autonomous commands when in auto mode
    if (is_auto_) {
        twist_pub_->publish(*msg);
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Auto twist - Linear: %.2f, Angular: %.2f", 
            msg->linear.x, msg->angular.z);
    }
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    
    try {
        auto unitree_teleop = std::make_shared<UnitreeTeleop>();
        rclcpp::spin(unitree_teleop);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("unitree_teleop"), "Exception caught: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}