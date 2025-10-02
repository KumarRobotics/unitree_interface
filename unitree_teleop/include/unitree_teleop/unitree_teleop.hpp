#ifndef UNITREE_TELEOP_HPP_
#define UNITREE_TELEOP_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <thread>
#include "unitree_api/msg/request.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"
#include "unitree_teleop/common/ros2_sport_client.h"

class UnitreeTeleop : public rclcpp::Node
{
public:
    UnitreeTeleop();

private:
    // callback functions
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

    // publishers and subscribers
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr highstate_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr is_auto_pub_;

    // sport client for robot commands
    SportClient sport_client_;
    unitree_api::msg::Request req_;

    // state variables
    geometry_msgs::msg::Twist twist_buf_;
    bool is_auto_;
    bool sit_transition_;
    bool is_armed_;
};

#endif  // UNITREE_TELEOP_HPP_