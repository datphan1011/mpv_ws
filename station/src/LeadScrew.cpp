#include <iostream>
#include <chrono>
#include <memory>
#include "CustomGPIO.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

class LeadScrew : public rclcpp :: Node{
public:
    LeadScrew(uint8_t i2c_address, uint8_t direction_pin, uint8_t pulse_pin, int total_steps = 2150, int max_speed = 500, int acceleration_rate = 100) 
    : Node("lead_screw_node"), i2c_address_(i2c_address), direction_pin_(direction_pin), pulse_pin_(pulse_pin),
        total_steps_(total_steps), max_speed_(max_speed), acceleration_rate_(acceleration_rate), current_speed_(0.0), is_moving_(false){
        // Create step_count publisher
        step_count_publisher_ = this->create_publisher<std_msgs::msg::Int32>("step_count", 10);
        // Initialize the Pigpio
        initialize_PIGPIO()
        // Setting up mode
        gpioSetMode(direction_pin_, PI_OUTPUT);
        gpioSetMode(pulse_pin_, PI_OUTPUT);
        // Set total steps
        void setTotalSteps(int steps) { 
            total_steps_ = steps; 
        }
        // Set max speed
        void setMaxSpeed(int speed) { 
            max_speed_ = speed; 
        }
        // Set Acceleration
        void setAccelerationRate(int rate) { 
            acceleration_rate_ = rate; 
        }
    }
    ~LeadScrew() {
    }
private:
    // Variable declaration
    uint8_t i2c_address, direction_pin, pulse_pin;
    int total_steps, max_speed, acceleration_rate;
    double current_speed_;
    bool is_moving_;
    // ROS2 node
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr step_count_publisher_;
    // Function to initalize the GPIO
    int initialize_PIGPIO() {
        if (gpioInitialise() < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize pigpio library");
            return -1;  // Return error code
        }
        RCLCPP_INFO(this->get_logger(), "pigpio library initialized successfully");
        return 0;  // Success
    }
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    //auto node = std::make_shared<LeadScrew>();
    //rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}