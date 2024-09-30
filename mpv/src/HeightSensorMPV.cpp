#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

class HeightSensorMPV : public rclcpp::Node{
public:
    HeightSensorMPV() : Node("height_sensor_mpv_node"){
    // Initialize the publisher for left and right sensor
    left_sensor_pub_ = this->create_publisher<std_msgs::msg::Int32>(
        "left_height_sensor", 10);
    right_sensor_pub_ = this->create_publisher<std_msgs::msg::Int32>(
        "right_height_sensor", 10);
    // Create a timer for a publisher to publishing data
    left_sensor_timer_ = this->create_wall_timer(std::chrono::milliseconds(750),
        std::bind(&HeightSensorMPV::left_sensor_data_callback, this));
    right_sensor_timer_ = this->create_wall_timer(std::chrono::milliseconds(750),
        std::bind(&HeightSensorMPV::right_sensor_data_callback, this));
    }
private:
    //Class member variables
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr left_sensor_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr right_sensor_pub_;
    rclcpp::TimerBase::SharedPtr left_sensor_timer_;
    rclcpp::TimerBase::SharedPtr right_sensor_timer_;

    void left_sensor_data_callback(){
        auto left_message = std_msgs::msg::Int32();
        left_message.data = 1200;
        RCLCPP_INFO(this->get_logger(), "Publishing left sensor data: %d", left_message.data);
        left_sensor_pub_->publish(left_message);
    }
    void right_sensor_data_callback(){
        auto right_message = std_msgs::msg::Int32();
        right_message.data = 1200;
        RCLCPP_INFO(this->get_logger(), "Publishing right sensor data: %d", right_message.data);
        right_sensor_pub_->publish(right_message);
    }
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HeightSensorMPV>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}