#include <iostream>
#include <functional>
#include <chrono>
#include <memory>
#include <pigpio.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
// Custom Header file
#include "station/CustomHeader/InitialisePIGPIOStation.hpp"
class LinearActuator : public rclcpp::Node{
public:
    LinearActuator(int left_up_act_pin, int left_down_act_pin, int right_up_act_pin, int right_down_act_pin) 
    : Node("linear_actuator_node"), left_differences(0), right_differences(0){
        // Initialize the node subscription
        // mpv left sensor
        mpv_left_sensor_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        "left_height_mpv", 10,std::bind(&LinearActuator::mpv_left_sensor_callback, this, std::placeholders::_1));
        // station left sensor
        station_left_sensor_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        "left_height_station", 10,std::bind(&LinearActuator::station_left_sensor_callback, this, std::placeholders::_1));
        // mpv right sensor
        mpv_right_sensor_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        "right_height_mpv", 10,std::bind(&LinearActuator::mpv_right_sensor_callback, this, std::placeholders::_1));
        // station right sensor
        station_right_sensor_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        "right_height_station", 10,std::bind(&LinearActuator::station_right_sensor_callback, this, std::placeholders::_1));
        // station control
        station_control_sub_ = this->create_subscription<std_msgs::msg::String>(
        "station_control", 10,std::bind(&LinearActuator::station_control_callback, this, std::placeholders::_1));
        // Initialize Pigpio library
        initialize_PIGPIO_station(this->get_logger());
        // Setting up mode
        gpioSetMode(left_up_act_pin, PI_OUTPUT);
        gpioSetMode(left_down_act_pin, PI_OUTPUT);
        gpioSetMode(right_up_act_pin, PI_OUTPUT);
        gpioSetMode(right_down_act_pin, PI_OUTPUT);
    }
    // Make the left actuator moving up if it is at peak moving right actuator.
    void moveUp(const std::string &actuator = "left") {
        if (actuator == "left") {
            gpioWrite(left_down_act_pin, 0);
            gpioWrite(left_up_act_pin, 1);
            RCLCPP_INFO(this->get_logger(), "Left actuator moving up");
        } else {
            gpioWrite(right_down_act_pin, 0);
            gpioWrite(right_up_act_pin, 1);
            RCLCPP_INFO(this->get_logger(), "Right actuator moving up");
        }
        /*
        gpioWrite(left_down_act_pin, 0);
        gpioWrite(left_up_act_pin, 1);
        gpioWrite(right_down_act_pin, 0);
        gpioWrite(right_up_act_pin, 1);
        RCLCPP_INFO(this->get_logger(), "Both actuators moving up");
        */
    }
    // Make the left actuator moving down if it is at peak moving right actuator.
    void moveDown(const std::string &actuator = "left") {
        if (actuator == "left") {
            gpioWrite(left_down_act_pin, 1);
            gpioWrite(left_up_act_pin, 0);
            RCLCPP_INFO(this->get_logger(), "Left actuator moving down");
        } else {
            gpioWrite(right_down_act_pin, 1);
            gpioWrite(right_up_act_pin, 0);
            RCLCPP_INFO(this->get_logger(), "Right actuator moving down");
        }
        /*
        gpioWrite(left_down_act_pin, 1);
        gpioWrite(left_up_act_pin, 0);
        gpioWrite(right_down_act_pin, 1);
        gpioWrite(right_up_act_pin, 0);
        RCLCPP_INFO(this->get_logger(), "Both actuators moving down");
        */
    }
    // Make the left actuator stop moving if it is at peak do right actuator.
    void stopMoving(const std::string &actuator = "left") {
        if (actuator == "left") {
            gpioWrite(left_down_act_pin, 0);
            gpioWrite(left_up_act_pin, 0);
            RCLCPP_INFO(this->get_logger(), "Left actuator stopped");
        } else {
            gpioWrite(right_down_act_pin, 0);
            gpioWrite(right_up_act_pin, 0);
            RCLCPP_INFO(this->get_logger(), "Right actuator stopped");
        }
        /*
        gpioWrite(left_down_act_pin, 0);
        gpioWrite(left_up_act_pin, 0);
        gpioWrite(right_down_act_pin, 0);
        gpioWrite(right_up_act_pin, 0);
        RCLCPP_INFO(this->get_logger(), "Both actuators stopped");
        */
    }
    ~LinearActuator() {
        gpioTerminate();  // Terminate pigpio on deconstructor
    }
private:
    // Class member variables and parameters
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mpv_left_sensor_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr station_left_sensor_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mpv_right_sensor_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr station_right_sensor_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr station_control_sub_;
    // Pin variable
    int left_up_act_pin, left_down_act_pin, right_up_act_pin, right_down_act_pin;
    // Differences variable
    int left_differences, right_differences;
    int mpv_left_sensor_data = -1, mpv_right_sensor_data = -1;
    int station_left_sensor_data =-1, station_right_sensor_data = -1;

    // Function to initialize the GPIO (Will change to another file so it more compact)
    void adjustMovement(const std::string& side, int station_value, int mpv_value, int difference) {
        if (station_value < mpv_value + difference) {
            moveUp(side);
        } else if (station_value > mpv_value + difference) {
            moveDown(side);
        } else {
            stopMoving(side);
        }
    }

    void station_control_callback(const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data == "HEIGHT") {
            if (station_left_sensor_data != -1 && mpv_left_sensor_data != -1) {
                adjustMovement("left", station_left_sensor_data, mpv_left_sensor_data, left_differences);
            }
            if (station_right_sensor_data != -1 && mpv_right_sensor_data != -1) {
                adjustMovement("right", station_right_sensor_data, mpv_right_sensor_data, right_differences);
            }
        }
    }
    // function to take the sensor data 
    void mpv_left_sensor_callback(const std_msgs::msg::Int32::SharedPtr msg){
        mpv_left_sensor_data = msg->data;
    }
    void mpv_right_sensor_callback(const std_msgs::msg::Int32::SharedPtr msg){
        mpv_right_sensor_data = msg->data;
    }
    void station_left_sensor_callback(const std_msgs::msg::Int32::SharedPtr msg){
        station_left_sensor_data = msg->data;
    }
    void station_right_sensor_callback(const std_msgs::msg::Int32::SharedPtr msg){
        station_right_sensor_data = msg->data;
    }
};
int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LinearActuator>(17, 18, 22, 23);  // GPIO pins for the actuators
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}