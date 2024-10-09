//--------------------------------------------
//Standard library
#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <pigpio.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
//--------------------------------------------
// This is just simulate the behaviour of the height sensor.
class HeightSensorMPV : public rclcpp::Node{
public:
    HeightSensorMPV() : Node("height_sensor_mpv_node"){
        // Initialize the publisher for left and right sensor
        left_sensor_pub_ = this->create_publisher<std_msgs::msg::Int32>(
            "left_height_mpv", 10);
        right_sensor_pub_ = this->create_publisher<std_msgs::msg::Int32>(
            "right_height_mpv", 10);
        // Create a timer for a publisher to publishing data
        height_sensor_timer_=this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&HeightSensorMPV::timer_callback, this));
        // initialize pigpio
        if(gpioInitialise() < 0){
            RCLCPP_ERROR(this->get_logger(), "Failed to initalize the Pigpio");
            rclcpp::shutdown();
    }
        // Setting left and right sensor pin
        left_sensor_shutdown_pin_mpv = 5;
        right_sensor_shutdown_pin_mpv = 6;

        // Configure GPIO as an output
        gpioSetMode(vl53l0x_1_fd_, PI_OUTPUT);
        gpioSetMode(vl53l0x_2_fd_, PI_OUTPUT);

        // Set both sensor to be off first Pi_low
        gpioWrite(vl53l0x_1_fd_, PI_LOW);
        gpioWrite(vl53l0x_2_fd_, PI_LOW);
        usleep(100000); // delay for 100ms

        // Initialize the first sensors 
        gpioWrite(left_sensor_shutdown_pin_mpv, PI_HIGH);
        usleep(100000); // delay for 100ms, increase latency will increase accuracy and vise versa.
        vl53l0x_1_fd_ = i2cOpen(1, 0x29, 0);

        if (vl53l0x_1_fd_ == -1){
            RCLCPP_ERROR(this->get_logger(), "Failed to intialize the first sensor.");
            rclcpp::shutdown();
        }
        // Change the first sensor address
        i2cWriteByteData(vl53l0x_1_fd_, 0x00, 0x30);    // Change the address to 0x30

        // Initialize the seconds sensor
        gpioWrite(right_sensor_shutdown_pin_mpv, PI_HIGH);
        usleep(100000);
        vl53l0x_2_fd_ = i2cOpen(1, 0x30, 0);

        if (vl53l0x_2_fd_ == -1){
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize the second sensor.");
            rclcpp::shutdown();
        }
        // Change the second sensor address
        i2cWriteByteData(vl53l0x_2_fd_, 0x00, 0x31);    // Change the address to 0x31

        // Set measurement timing budget for both sensors (300000 µs = 300 ms)
        set_timing_budget(vl53l0x_1_fd_, 300000);
        set_timing_budget(vl53l0x_1_fd_, 300000);
    }
    ~HeightSensorMPV(){
        i2cClose(vl53l0x_1_fd_);
        i2cClose(vl53l0x_1_fd_);
        gpioTerminate();
    }
private:
    //Class member variables
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr left_sensor_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr right_sensor_pub_;
    rclcpp::TimerBase::SharedPtr height_sensor_timer_;

    int left_sensor_shutdown_pin_mpv;
    int right_sensor_shutdown_pin_mpv;

    int vl53l0x_1_fd_; // File descriptor for the left sensor
    int vl53l0x_2_fd_; // File descriptor for the right sensor

    void timer_callback(){
        //create 2 message that hold the data
        auto msg1 = std_msgs::msg::Int32();
        auto msg2 = std_msgs::msg::Int32();
        //create a variable that call the function and get the sensor range
        int left_sensor_range = get_sensor_range(vl53l0x_1_fd_);
        int right_sensor_range = get_sensor_range(vl53l0x_2_fd_);
        //Adding data to the message and 
        msg1.data = left_sensor_range;
        msg2.data = right_sensor_range;
        // Publish the messages
        left_sensor_pub_->publish(msg1);
        right_sensor_pub_->publish(msg2);
    }   

    int get_sensor_range(int sensor_fd){
        // VL53L0X register to read the range data (register address can vary)
        const int RESULT_RANGE_STATUS = 0x14;  // Example range register address
        int range = i2cReadWordData(sensor_fd, RESULT_RANGE_STATUS);
        if (range == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to read range from sensor");
        }
        return range;
    }
    /*sets the "timing budget" for the VL53L0X time-of-flight (ToF) sensor. In the VL53L0X sensor,
    the timing budget represents how much time the sensor spends measuring distances for each ranging operation (in microseconds). 
    A longer timing budget generally results in more accurate measurements, but it also makes the sensor slower.*/
    void set_timing_budget(int sensor_fd_, uint32_t budget){
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
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HeightSensorMPV>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}