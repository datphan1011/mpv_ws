#include <iostream>
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/Bool.hpp>
#include <std_msgs/msg/string>

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    //auto node = std::make_shared<LeadScrew>();
    //rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}