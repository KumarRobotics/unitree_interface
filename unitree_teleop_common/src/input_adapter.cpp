#include "unitree_teleop_common/input_adapter.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace unitree_teleop_common
{

InputAdapter::InputAdapter(
  rclcpp::Node & node, JoyCallback joy_callback,
  TimeoutCallback timeout_callback)
: node_(node),
  joy_callback_(std::move(joy_callback)),
  timeout_callback_(std::move(timeout_callback)),
  rc_pwm_center_(1500.0),
  rc_pwm_half_range_(500.0),
  rc_deadzone_(0.1),
  rc_timeout_(0.5),
  last_rc_time_(node.now()),
  rc_timed_out_(true)
{
  node_.declare_parameter<std::string>("input_source", "mavros");
  const auto input_source = node_.get_parameter("input_source").as_string();

  // Entry i gives the 1-indexed RC channel used for Joy axis i:
  // speed, strafe, rotation, forward/back, auto, gait, arm/posture.
  // Forward/back is not reversed to match the MAVROS stick direction. The auto
  // switch is not reversed: low PWM selects manual, high PWM selects auto.
  node_.declare_parameter<std::vector<int64_t>>(
    "rc_axis_channels", {3, 4, 1, 2, 5, 6, 7});
  node_.declare_parameter<std::vector<bool>>(
    "rc_axis_reversed", {true, true, true, false, false, true, true});
  node_.declare_parameter<std::vector<int64_t>>("rc_switch_axes", {4, 5, 6});
  node_.declare_parameter<double>("rc_pwm_center", rc_pwm_center_);
  node_.declare_parameter<double>("rc_pwm_half_range", rc_pwm_half_range_);
  node_.declare_parameter<double>("rc_deadzone", rc_deadzone_);
  node_.declare_parameter<double>("rc_timeout", rc_timeout_);

  rc_axis_channels_ = node_.get_parameter("rc_axis_channels").as_integer_array();
  rc_axis_reversed_ = node_.get_parameter("rc_axis_reversed").as_bool_array();
  rc_switch_axes_ = node_.get_parameter("rc_switch_axes").as_integer_array();
  rc_pwm_center_ = node_.get_parameter("rc_pwm_center").as_double();
  rc_pwm_half_range_ = node_.get_parameter("rc_pwm_half_range").as_double();
  rc_deadzone_ = node_.get_parameter("rc_deadzone").as_double();
  rc_timeout_ = node_.get_parameter("rc_timeout").as_double();

  if (input_source == "mavros") {
    if (rc_axis_channels_.size() < 7 ||
      rc_axis_reversed_.size() != rc_axis_channels_.size())
    {
      throw std::invalid_argument(
              "rc_axis_channels needs >= 7 entries and rc_axis_reversed must match its length");
    }
    if (rc_pwm_half_range_ <= 0.0) {
      throw std::invalid_argument("rc_pwm_half_range must be greater than zero");
    }
    if (rc_deadzone_ < 0.0 || rc_deadzone_ >= 1.0) {
      throw std::invalid_argument("rc_deadzone must be in [0, 1)");
    }
    if (rc_timeout_ <= 0.0) {
      throw std::invalid_argument("rc_timeout must be greater than zero");
    }

    rc_in_sub_ = node_.create_subscription<mavros_msgs::msg::RCIn>(
      "rc_in", rclcpp::SensorDataQoS(),
      std::bind(&InputAdapter::rc_in_callback, this, std::placeholders::_1));
    rc_watchdog_timer_ = node_.create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&InputAdapter::rc_watchdog_callback, this));

    RCLCPP_INFO(
      node_.get_logger(), "Input source: mavros RCIn on %s",
      rc_in_sub_->get_topic_name());
  } else if (input_source == "joy") {
    joy_sub_ = node_.create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, joy_callback_);
    RCLCPP_INFO(
      node_.get_logger(), "Input source: joy on %s", joy_sub_->get_topic_name());
  } else {
    throw std::invalid_argument(
            "input_source must be 'joy' or 'mavros', got '" + input_source + "'");
  }
}

void InputAdapter::rc_in_callback(const mavros_msgs::msg::RCIn::SharedPtr msg)
{
  auto joy = std::make_shared<sensor_msgs::msg::Joy>();
  joy->header = msg->header;
  joy->axes.resize(rc_axis_channels_.size(), 0.0F);

  for (size_t i = 0; i < rc_axis_channels_.size(); ++i) {
    const int64_t channel = rc_axis_channels_[i];
    if (channel < 1 || static_cast<size_t>(channel) > msg->channels.size()) {
      RCLCPP_WARN_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 2000,
        "RCIn has %zu channels, axis %zu wants channel %ld; ignoring message",
        msg->channels.size(), i, channel);
      return;
    }

    double value =
      (static_cast<double>(msg->channels[channel - 1]) - rc_pwm_center_) /
      rc_pwm_half_range_;
    value = std::clamp(value, -1.0, 1.0);
    if (rc_axis_reversed_[i]) {
      value = -value;
    }

    const bool is_switch =
      std::find(rc_switch_axes_.begin(), rc_switch_axes_.end(),
      static_cast<int64_t>(i)) != rc_switch_axes_.end();
    if (is_switch) {
      value = value > 0.5 ? 1.0 : (value < -0.5 ? -1.0 : 0.0);
    } else if (std::abs(value) < rc_deadzone_) {
      value = 0.0;
    } else {
      value = std::copysign(
        (std::abs(value) - rc_deadzone_) / (1.0 - rc_deadzone_), value);
    }
    joy->axes[i] = static_cast<float>(value);
  }

  if (rc_timed_out_) {
    RCLCPP_INFO(node_.get_logger(), "RCIn received");
    rc_timed_out_ = false;
  }
  last_rc_time_ = node_.now();
  joy_callback_(joy);
}

void InputAdapter::rc_watchdog_callback()
{
  if (rc_timed_out_ || (node_.now() - last_rc_time_).seconds() < rc_timeout_) {
    return;
  }

  rc_timed_out_ = true;
  RCLCPP_WARN(
    node_.get_logger(), "No RCIn for %.2f s; stopping and leaving auto", rc_timeout_);
  timeout_callback_();
}

}  // namespace unitree_teleop_common
