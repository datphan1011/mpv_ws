#include <rclcpp/rclcpp.hpp>    // ROS2 C++ client library
#include <std_msgs/msg/int32.hpp>  // ROS2 message type
// #include "wiringPiI2C.h"        // WiringPi for I2C communication
// #include <wiringPi.h>           // WiringPi for GPIO control
#include <chrono>               // For time handling
#include <memory>               // For smart pointers
#include <iostream>             // Standard cpp library
#include <pigpio.h>             // Pigpio for GPIO control
// Custom header file
#include <station/CustomHeader/InitialisePIGPIOStation.hpp>
#include <station/CustomHeader/HeightSensorConfiguration.hpp>

class HeightSensorStation : public rclcpp::Node{
public:
    HeightSensorStation()
        : Node("height_sensor_node_station"),  // Initialize the ROS2 node with the name "height_sensor_node_mpv"
          logger_(this->get_logger()),     // Initialize logger from the ROS2 Node's logger
          height_sensor_config_(23, 24, logger_) { // Initialize HeightSensorConfiguration with GPIO pins 23 and 24 and logger, pin can be changed
        
        // Initialize the left and right sensor publisher
        left_sensor_publisher_station_=this->create_publisher<std_msgs::msg::Int32>("left_sensor_station", 10);
        right_sensor_publisher_station_=this->create_publisher<std_msgs::msg::Int32>("right_sensor_station", 10);
        // Create a timer to call the `timer_callback` function every 100 milliseconds
        height_sensor_timer_station_ = this->create_wall_timer(
            std::chrono::milliseconds(100), // 100ms timer interval
            std::bind(&HeightSensorStation::timer_callback, this)  // Bind the timer callback function
        );
        // Initialize pigpio
        initialize_PIGPIO_station(this->get_logger());
        // Initialize sensors for the left and right height sensors (I2C initialization)
        vl53l0x_1_fd_ = height_sensor_config_.initialize_sensor(height_sensor_config_.get_left_pin(), 0x30); // Left sensor, I2C address 0x30
        vl53l0x_2_fd_ = height_sensor_config_.initialize_sensor(height_sensor_config_.get_right_pin(), 0x31); // Right sensor, I2C address 0x31
        // Set timing budgets for both sensors (300ms in this case)
        height_sensor_config_.set_timing_budget(vl53l0x_1_fd_, 300000); // Set timing budget for left sensor
        height_sensor_config_.set_timing_budget(vl53l0x_2_fd_, 300000); // Set timing budget for right sensor
        }
    // Deconstructor 
    ~HeightSensorStation() {
        i2cClose(vl53l0x_1_fd_); // Close the I2C connection for left sensor
        i2cClose(vl53l0x_2_fd_); // Close the I2C connection for right sensor
    }

private:
    //Class member variables
    rclcpp::Logger logger_;  // Logger to log messages within the node
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr left_sensor_publisher_station_;  // Publisher for left sensor data
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr right_sensor_publisher_station_; // Publisher for right sensor data
    rclcpp::TimerBase::SharedPtr height_sensor_timer_station_; // Timer to call the callback function periodically
        
// Sensor configuration and file descriptors
    HeightSensorConfiguration height_sensor_config_; // The configuration for the height sensors (handles GPIO, I2C, etc.)
    int vl53l0x_1_fd_; // File descriptor for the left sensor (VL53L0X)
    int vl53l0x_2_fd_; // File descriptor for the right sensor (VL53L0X)

// Timer callback function to periodically fetch sensor data and publish it
    void timer_callback() {
        // Create messages to hold sensor data
        auto msg_1 = std::make_shared<std_msgs::msg::Int32>();
        auto msg_2 = std::make_shared<std_msgs::msg::Int32>();

        // Fetch sensor data for both sensors
        msg_1->data = height_sensor_config_.get_sensor_range(vl53l0x_1_fd_); // Get range from left sensor
        msg_2->data = height_sensor_config_.get_sensor_range(vl53l0x_2_fd_); // Get range from right sensor

        // Publish the sensor data
        left_sensor_publisher_station_->publish(*msg_1); // Publish left sensor data
        right_sensor_publisher_station_->publish(*msg_2); // Publish right sensor data

        // Log the sensor values
        RCLCPP_INFO(this->get_logger(), "Left sensor (MPV): %d, Right sensor (MPV): %d", msg_1->data, msg_2->data);
    }
};

int main(int argc, char **argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<HeightSensorStation>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
