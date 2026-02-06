#ifndef UNITREE_TELEOP_HPP_
#define UNITREE_TELEOP_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>

#include <unitree/robot/channel/channel_factory.hpp>
#include "unitree_teleop/sport_client_ext.hpp"

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
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr is_auto_pub_;

    // unitree_sdk2 sport client with SetGait extension
    SportClientExt sport_client_;

    // state variables
    geometry_msgs::msg::Twist twist_buf_;
    bool is_auto_;
    bool sit_transition_;
    bool is_armed_;
    int gait_type_;

    // network interface for DDS communication
    std::string network_interface_;
};

#endif  // UNITREE_TELEOP_HPP_
