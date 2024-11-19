#include <iostream>
#include <chrono>
#include <memory>
#include <pigpio.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
// Custom Header file
#include "station/CustomHeader/InitialisePIGPIOStation.hpp"

class LeadScrew : public rclcpp :: Node{
public:
    LeadScrew(uint8_t i2c_address, uint8_t direction_pin, uint8_t pulse_pin, int total_steps = 2150, int max_speed = 500, int acceleration_rate = 100) 
    : Node("lead_screw_node"), i2c_address_(i2c_address), direction_pin_(direction_pin), pulse_pin_(pulse_pin),
        total_steps_(total_steps), max_speed_(max_speed), acceleration_rate_(acceleration_rate), current_speed_(0.0), is_moving_(false){
        // Create step_count publisher
        step_count_publisher_ = this->create_publisher<std_msgs::msg::Int32>("step_count", 10);
        // Initialize the Pigpio
        initialize_PIGPIO_station(this->get_logger()); 
        // Setting up mode
        gpioSetMode(direction_pin_, PI_OUTPUT);
        gpioSetMode(pulse_pin_, PI_OUTPUT);
    }
    // Public methods for setting parameters
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
    
    ~LeadScrew() {
        stop();
        gpioTerminate();
    }
private:
    // Variable declaration
    uint8_t i2c_address_, direction_pin_, pulse_pin_;
    int total_steps_, max_speed_, acceleration_rate_;
    double current_speed_;
    bool is_moving_;
    // ROS2 node
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr step_count_publisher_;

    // Function for Lead Screw
    void stop(){
        is_moving_ = false;
        gpioWrite(pulse_pin_, 0);
        gpioWrite(direction_pin_, 0);
    }
    void move(const std::string& direction){
        // Checking direction
        if (direction != "FORWARD" && direction != "BACKWARD") {
            throw std::invalid_argument("Invalid direction. Use 'FORWARD' or 'BACKWARD'.");
        }
        // setting the moment statement
        is_moving_ = true;
        // setting direction pin
        gpioWrite(direction_pin_, direction == "FORWARD" ? 1 : 0);
        // loop for generation step
        for (int step = 0; step < total_steps_ && is_moving_; ++step) {
            current_speed_ = calculateSpeed(step);
            double half_step_delay = 0.5 / current_speed_;
            // Write the pulse pin.
            /*The function writes a 1 (HIGH) to the pulse_pin_, turning on the pulse.
            It pauses for half_step_delay seconds.
            Then, it writes a 0 (LOW) to the pulse_pin_, turning off the pulse.
            Another delay (half_step_delay) follows.*/
            gpioWrite(pulse_pin_, 1);
            std::this_thread::sleep_for(std::chrono::duration<double>(half_step_delay));
            gpioWrite(pulse_pin_, 0);
            std::this_thread::sleep_for(std::chrono::duration<double>(half_step_delay));
        }
        stop();  // Reset the direction and pulse pins after movement
    }
    double calculateSpeed(int current_step){
        int remaining_steps = total_steps_ - current_step;
        if (remaining_steps == 0) {
            return 0.0;
        }

        double target_speed = std::sqrt(2.0 * remaining_steps * acceleration_rate_);
        if (target_speed > max_speed_) {
            return max_speed_;
        }
        return target_speed;
    }
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    // This creates a shared pointer to new instance of LeadScrew node
    auto lead_screw_node1 = std::make_shared<LeadScrew>(0x41, 4, 5);
    auto lead_screw_node2 = std::make_shared<LeadScrew>(0x42, 6, 7);
    // This line creates a single-threaded executor.
    rclcpp::executors::SingleThreadedExecutor executor;
    // The executor will manage this node, running its callbacks and keeping it alive as long as the executor is active.
    executor.add_node(lead_screw_node1);
    executor.add_node(lead_screw_node2);
    // This starts the executor’s main loop. It keeps running, managing the nodes added to it, and processing their callbacks 
    executor.spin();
    gpioTerminate();
    rclcpp::shutdown();
    return 0;
}