#ifndef UNITREE_TELEOP_HPP_
#define UNITREE_TELEOP_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <mavros_msgs/msg/rc_in.hpp>
#include <memory>
#include <string>
#include <vector>

#include "unitree_teleop/sport_client_ext.hpp"

class UnitreeTeleop : public rclcpp::Node
{
public:
    UnitreeTeleop();

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void rc_in_callback(const mavros_msgs::msg::RCIn::SharedPtr msg);
    void rc_watchdog_callback();

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr rc_in_sub_;
    rclcpp::TimerBase::SharedPtr rc_watchdog_timer_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr is_auto_pub_;

    std::unique_ptr<SportClientExt> sport_client_;

    geometry_msgs::msg::Twist twist_buf_;
    bool is_auto_;
    bool sit_transition_;
    bool is_armed_;
    int gait_type_;

    // RCIn -> Joy conversion (input_source == "mavros")
    std::vector<int64_t> rc_axis_channels_;
    std::vector<bool> rc_axis_reversed_;
    std::vector<int64_t> rc_switch_axes_;
    double rc_pwm_center_;
    double rc_pwm_half_range_;
    double rc_deadzone_;
    double rc_timeout_;
    rclcpp::Time last_rc_time_;
    bool rc_timed_out_;
};

#endif  // UNITREE_TELEOP_HPP_
