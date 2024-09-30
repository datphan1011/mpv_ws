#include <iostream>
#include <functional>
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/String.hpp>
#include <std_msgs/msg/int32.hpp>

class LinearActuator : public rclcpp::Node{
public:
    LinearActuator() : Node("linear_actuator_node"){
        actuator_left_sensor_sub_=this->create_subscription<std_msgs::msg::Int32
    }
}