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
#include <mpv/CustomHeader/HeightSensorConfiguration.hpp>
class HeightSensorStation : public rclcpp::Node{
public:
    HeightSensorStation() : Node("height_sensor_node_station"){
        // Initialize the left and right sensor publisher
        left_sensor_publisher_=this->create_publisher<std_msgs::msg::Int32>("left_sensor_station", 10);
        right_sensor_publisher_=this->create_publisher<std_msgs::msg::Int32>("right_sensor_station", 10);
        height_sensor_timer_=this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&HeightSensorStation::timer_callback, this));

        // Initialize pigpio
        initialize_PIGPIO_station(this->get_logger());
        // Left and right sensor pin
        left_sensor_shudown_pin = 23; // GPIO23
        right_sensor_shudown_pin = 24; // GPIO24
        // Configure GPIO as an output
        gpioSetMode(left_sensor_shudown_pin, PI_OUTPUT);  
        gpioSetMode(right_sensor_shudown_pin, PI_OUTPUT);

        // Turn off both sensors by setting the pin to LOW
        gpioWrite(left_sensor_shudown_pin, PI_LOW);   
        gpioWrite(right_sensor_shudown_pin, PI_LOW);
        usleep(100000); // 100 ms
            
        // Initialize first sensor
        gpioWrite(left_sensor_shudown_pin, PI_HIGH);
        usleep(100000); // 100 ms
        vl53l0x_1_fd_ = i2cOpen(1, 0x29, 0); // Default I2C address before changing

        if (vl53l0x_1_fd_ == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize sensor 1");
        }
        // Change the I2C address to 0x30 for the first sensor
        i2cWriteByteData(vl53l0x_1_fd_, 0x00, 0x30);

        // Initialize second sensor
        gpioWrite(right_sensor_shudown_pin, PI_HIGH);
        usleep(100000); // 100 ms
        vl53l0x_2_fd_ = i2cOpen(1, 0x30, 0); // Default I2C address
        if (vl53l0x_2_fd_ == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize sensor 2");
        }
        // Change the I2C address to 0x31 for the second sensor
        i2cWriteByteData(vl53l0x_2_fd_ , 0x00, 0x31);

        // Set measurement timing budget for both sensors
        set_timing_budget(vl53l0x_1_fd_, 300000);
        set_timing_budget(vl53l0x_2_fd_, 300000);
        }
    ~HeightSensorStation(){
        // Close I2C handles and terminate pigpio
        i2cClose(vl53l0x_1_fd_);
        i2cClose(vl53l0x_2_fd_);
        gpioTerminate();
    }

private:
    //Class member variables
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr left_sensor_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr right_sensor_publisher_;
    rclcpp::TimerBase::SharedPtr height_sensor_timer_;
        
    int left_sensor_shudown_pin;
    int right_sensor_shudown_pin;

    int vl53l0x_1_fd_; // File descriptor for the left sensor
    int vl53l0x_2_fd_; // File descriptor for the right sensor
    void timer_callback()
    {
        //create 2 message that hold the data
        auto msg_1 = std::make_shared<std_msgs::msg::Int32>(); 
        auto msg_2 = std::make_shared<std_msgs::msg::Int32>();
        //create a variable that call the function
        int left_sensor_range = get_sensor_range(vl53l0x_1_fd_);
        int right_sensor_range = get_sensor_range(vl53l0x_2_fd_);
        //adding the data of the function into the left sensor and publish it
        msg_1->data = left_sensor_range;
        left_sensor_publisher_->publish(*msg_1);
        //adding the data of the function into the right sensor and publish it
        msg_2->data = right_sensor_range;
        right_sensor_publisher_->publish(*msg_2);
        // Print the data into the console so I can check it
        RCLCPP_INFO(this->get_logger(), "The data of the left sensor:  %d", left_sensor_range);
        RCLCPP_INFO(this->get_logger(), "The data of the right sensor: %d", right_sensor_range);
    }

    int get_sensor_range(int sensor_fd)
    {
        // VL53L0X register to read the range data (usually in millimeters)
        const int RESULT_RANGE_STATUS = 0x14;  // Example register for range
        int range = i2cReadWordData(sensor_fd, RESULT_RANGE_STATUS);
            
        if (range == -1) {
            RCLCPP_ERROR(rclcpp::get_logger("VL53L0XNode"), "Failed to read range from sensor");
        }
        return range;
    }
    /*sets the "timing budget" for the VL53L0X time-of-flight (ToF) sensor. In the VL53L0X sensor,
    the timing budget represents how much time the sensor spends measuring distances for each ranging operation (in microseconds). 
    A longer timing budget generally results in more accurate measurements, but it also makes the sensor slower.*/
    void set_timing_budget(int sensor_fd_, uint32_t budget)
    {
        // Timing budget register addresses, This internal register can be access directly via I2C without VL53L0X library
        const int FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x71; //change it back base on the datasheet
        const int FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO = 0x72; //change it back base on the datasheet

        // Convert the budget (in microseconds) to register values
        // Formula based on VL53L0X datasheet
        uint16_t budget_reg_value = (budget / 2) - 15;
        // High byte: (budget_reg_value >> 8) & 0xFF shifts the 16-bit value 8 bits to the right and extracts the upper byte.
        i2cWriteByteData(sensor_fd_, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, (budget_reg_value >> 8) & 0xFF);
        // Low byte: budget_reg_value & 0xFF extracts the lower byte.
        i2cWriteByteData(sensor_fd_, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO, budget_reg_value & 0xFF);
    }
};

int main(int argc, char **argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<HeightSensorStation>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
