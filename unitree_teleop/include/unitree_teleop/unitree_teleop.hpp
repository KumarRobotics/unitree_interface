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
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void pub_callback();
    void high_state_handler(const unitree_go::msg::SportModeState::SharedPtr msg);
    void get_init_state();

    // Publishers and subscribers
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr highstate_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr is_auto_pub_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr pub_timer_;

    // Sport client for robot communication
    SportClient sport_client_;
    unitree_api::msg::Request req_;

    // State variables
    geometry_msgs::msg::Twist twist_buf_;
    bool is_auto_;
};

#endif  // UNITREE_TELEOP_HPP_