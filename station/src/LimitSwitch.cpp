#include "rclcpp/rclcpp.hpp"    // ROS2 C++ client library
#include "std_msgs/msg/bool.hpp"  // ROS2 message type for Boolean messages
// #include <wiringPi.h>           // WiringPi library for GPIO control
#include <chrono>               // For time handling
#include <memory>               // For smart pointers
#include <iostream>             // Standard cpp library
#include <pigpio.h>             // pigpio library for GPIO control, replace WiringPi

class LimitSwitch : public rclcpp::Node{
public:
    LimitSwitch() : Node("limit_switch_node"), limitswitch_pin_start_(17), limitswitch_pin_end_(27), default_state_(false){
        // Create limitswitch publisher
        limitswitch_start_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        "limitswitch_state_start", 10);
        limitswitch_end_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        "limitswitch_state_end", 10);
        // Create a timer for the publisher to publish the signal 
        limitswitch_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&LimitSwitch::timer_callback, this));
        // Initialize the pigpio library
        if (gpioInitialise() < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize pigpio library");
            return;
        }
        // Initialize GPIO pins
        gpioSetMode(limitswitch_pin_start_, PI_INPUT);
        gpioSetMode(limitswitch_pin_end_, PI_INPUT);
        // Enable pull-up resistors for both switches
        gpioSetPullUpDown(limitswitch_pin_start_, PI_PUD_UP);
        gpioSetPullUpDown(limitswitch_pin_end_, PI_PUD_UP);
    }
    ~LimitSwitch(){
        reset();
        gpioTerminate();
    }
private:
    // Class member variables
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr limitswitch_start_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr limitswitch_end_pub_;
    rclcpp::TimerBase::SharedPtr limitswitch_timer_;
    // Parametes
    int limitswitch_pin_start_;
    int limitswitch_pin_end_;
    bool default_state_;
    // Functions
    // This function publish the data to the limitswitch state
    void timer_callback(){
        bool state_start = get_current_state(limitswitch_pin_start_);    // current state at start
        RCLCPP_INFO(this->get_logger(), "Switch start state: %s", state_start ? "Pressed" : "Released"); // Printed out is 
        auto limitswitch_msg_start = std_msgs::msg::Bool();
        limitswitch_msg_start.data = state_start;
        limitswitch_start_pub_->publish(limitswitch_msg_start);

        bool state_end = get_current_state(limitswitch_pin_end_);
        RCLCPP_INFO(this->get_logger(), "Switch end state: %s", state_end ? "Pressed" : "Released");
        auto limitswitch_msg_end = std_msgs::msg::Bool();
        limitswitch_msg_end.data = state_end;
        limitswitch_end_pub_->publish(limitswitch_msg_end);
    }
    // Function to get the current state of a switch 
    bool get_current_state(int pin){
        int currentState = gpioRead(pin);
        return (currentState == PI_LOW) != default_state_;
    }
    // Function to reset the limit state after use
    void reset(){
        gpioSetMode(limitswitch_pin_start_, PI_INPUT);
        gpioSetMode(limitswitch_pin_end_, PI_INPUT);
    }
};
int main(int argc, char **argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<LimitSwitch>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
