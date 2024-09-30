#include "rclcpp/rclcpp.hpp"  // ROS2 C++ client library
#include "std_msgs/msg/int32.hpp"  // ROS2 message type
#include "wiringPiI2C.h" // WiringPi for I2C communication
#include <wiringPi.h>  // WiringPi for GPIO control
// #include <VL53L0X.h>
#include <chrono>  // For time handling
#include <memory>  // For smart pointers
#include <iostream> 


class HeightSensorStation : public rclcpp::Node{
    public:
        HeightSensorStation() : Node("vl53l1x_node_publisher"){
            left_sensor_publisher_=this->create_publisher<std_msgs::msg::Int32>("left_sensor_data", 10);
            right_sensor_publisher_=this->create_publisher<std_msgs::msg::Int32>("right_sensor_data", 10);
            height_sensor_timer_=this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&HeightSensorStation::timer_callback, this));

            // Initialize GPIO pins for shutdown
            wiringPiSetupGpio();

            left_sensor_shudown_pin = 23; // GPIO23
            right_sensor_shudown_pin = 24; // GPIO24
            // Configure GPIO as an output
            pinMode(left_sensor_shudown_pin, OUTPUT);  
            pinMode(right_sensor_shudown_pin, OUTPUT);

            // Turn off both sensors by setting the pin to LOW
            digitalWrite(left_sensor_shudown_pin, LOW);   
            digitalWrite(right_sensor_shudown_pin, LOW);
            usleep(100000); // 100 ms
            // Initialize first sensor
            digitalWrite(left_sensor_shudown_pin, HIGH);
            usleep(100000); // 100 ms
            vl53_1_fd = wiringPiI2CSetup(0x29); // Default I2C address before changing
            if (vl53_1_fd == -1)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to initialize sensor 1");
            }
            // Change the I2C address to 0x30 for the first sensor
            wiringPiI2CWriteReg8(vl53_1_fd, 0x00, 0x30);

            // Initialize second sensor
            digitalWrite(right_sensor_shudown_pin, HIGH);
            usleep(100000); // 100 ms
            vl53_2_fd = wiringPiI2CSetup(0x29); // Default I2C address
            if (vl53_2_fd == -1)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to initialize sensor 2");
            }
            // Change the I2C address to 0x31 for the second sensor
            wiringPiI2CWriteReg8(vl53_2_fd, 0x00, 0x31);

            // Set measurement timing budget for both sensors
            set_timing_budget(vl53_1_fd, 300000);
            set_timing_budget(vl53_2_fd, 300000);
        }

    private:
        //Class member variables
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr left_sensor_publisher_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr right_sensor_publisher_;
        rclcpp::TimerBase::SharedPtr height_sensor_timer_;

        int left_sensor_shudown_pin;
        int right_sensor_shudown_pin;

        int vl53_1_fd; // File descriptor for the left sensor
        int vl53_2_fd; // File descriptor for the right sensor
        void timer_callback()
        {
            //create 2 message that hold the data
            auto msg_1 = std::make_shared<std_msgs::msg::Int32>(); 
            auto msg_2 = std::make_shared<std_msgs::msg::Int32>();
            //create a variable that call the function
            int left_sensor_range = get_sensor_range(vl53_1_fd);
            int right_sensor_range = get_sensor_range(vl53_2_fd);
            //adding the data of the function into the left sensor and publish it
            msg_1->data = left_sensor_range;
            left_sensor_publisher_->publish(*msg_1);
            //adding the data of the function into the right sensor and publish it
            msg_2->data = right_sensor_range;
            right_sensor_publisher_->publish(*msg_2);
        }

        int get_sensor_range(int sensor_fd)
        {
            // VL53L0X register to read the range data (usually in millimeters)
            const int RESULT_RANGE_STATUS = 0x14;  // Example register for range
            int range = wiringPiI2CReadReg16(sensor_fd, RESULT_RANGE_STATUS);
            
            if (range == -1) {
                RCLCPP_ERROR(rclcpp::get_logger("VL53L0XNode"), "Failed to read range from sensor");
            }
            return range;
        }

        void set_timing_budget(int sensor_fd, uint32_t budget)
        {
            // Timing budget register addresses (example)
            const int FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x71; //change it back base on the datasheet
            const int FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO = 0x72; //change it back base on the datasheet

            // Convert the budget (in microseconds) to register values
            // Formula based on VL53L0X datasheet
            uint16_t budget_reg_value = (budget / 2) - 15;

            wiringPiI2CWriteReg8(sensor_fd, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, (budget_reg_value >> 8) & 0xFF);
            wiringPiI2CWriteReg8(sensor_fd, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO, budget_reg_value & 0xFF);
        }
};

int main(int argc, char **argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<HeightSensorStation>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
