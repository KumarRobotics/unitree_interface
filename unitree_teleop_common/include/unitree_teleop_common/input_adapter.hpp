#ifndef UNITREE_TELEOP_COMMON__INPUT_ADAPTER_HPP_
#define UNITREE_TELEOP_COMMON__INPUT_ADAPTER_HPP_

#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

#include <mavros_msgs/msg/rc_in.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

namespace unitree_teleop_common
{

class InputAdapter
{
public:
  using JoyCallback = std::function<void(sensor_msgs::msg::Joy::SharedPtr)>;
  using TimeoutCallback = std::function<void()>;

  InputAdapter(
    rclcpp::Node & node, JoyCallback joy_callback,
    TimeoutCallback timeout_callback);

private:
  void rc_in_callback(const mavros_msgs::msg::RCIn::SharedPtr msg);
  void rc_watchdog_callback();

  rclcpp::Node & node_;
  JoyCallback joy_callback_;
  TimeoutCallback timeout_callback_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr rc_in_sub_;
  rclcpp::TimerBase::SharedPtr rc_watchdog_timer_;

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

}  // namespace unitree_teleop_common

#endif  // UNITREE_TELEOP_COMMON__INPUT_ADAPTER_HPP_
