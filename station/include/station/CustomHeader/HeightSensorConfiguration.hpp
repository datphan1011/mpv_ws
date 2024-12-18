#ifndef HEIGHT_SENSOR_CONFIGURATION_HPP
#define HEIGHT_SENSOR_CONFIGURATION_HPP

#include <stdexcept>
#include <rclcpp/rclcpp.hpp>
#include <pigpio.h>
#include <unistd.h>
#include <station/CustomHeader/InitialisePIGPIOStation.hpp> // Include your new initializer header

class HeightSensorConfiguration {
public:
    // Constructor that takes left and right pin numbers and a ROS logger
    HeightSensorConfiguration(int left_pin, int right_pin, const rclcpp::Logger &logger)
        : left_sensor_shutdown_pin_(left_pin), right_sensor_shutdown_pin_(right_pin), logger_(logger) {
        // Initialize the Pigpio library
        initialize_PIGPIO_station(this->logger_);
    }

    // Destructor to clean up
    ~HeightSensorConfiguration() {
        gpioTerminate(); // Ensure proper cleanup
    }

    // Sensor initialization
    int initialize_sensor(int pin, int new_i2c_address) {
        gpioSetMode(pin, PI_OUTPUT);
        gpioWrite(pin, PI_LOW);
        usleep(100000); // 100 ms

        gpioWrite(pin, PI_HIGH);
        usleep(100000); // 100 ms

        int sensor_fd = i2cOpen(1, 0x29, 0);
        if (sensor_fd == -1) {
            RCLCPP_ERROR(logger_, "Failed to initialize sensor on pin %d", pin);
            return -1;
        }

        i2cWriteByteData(sensor_fd, 0x00, new_i2c_address);
        return sensor_fd;
    }

    // Get the sensor range
    int get_sensor_range(int sensor_fd) {
        const int RESULT_RANGE_STATUS = 0x14; // Replace with actual register address
        int range = i2cReadWordData(sensor_fd, RESULT_RANGE_STATUS);
        if (range == -1) {
            RCLCPP_ERROR(logger_, "Failed to read range from sensor");
        }
        return range;
    }

    /*sets the "timing budget" for the VL53L0X time-of-flight (ToF) sensor. In the VL53L0X sensor,
    the timing budget represents how much time the sensor spends measuring distances for each ranging operation (in microseconds). 
    A longer timing budget generally results in more accurate measurements, but it also makes the sensor slower.*/
    void set_timing_budget(int sensor_fd, uint32_t budget) {
        const int FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x71;
        const int FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO = 0x72;

        uint16_t budget_reg_value = (budget / 2) - 15;
        i2cWriteByteData(sensor_fd, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, (budget_reg_value >> 8) & 0xFF);
        i2cWriteByteData(sensor_fd, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO, budget_reg_value & 0xFF);
    }

    // Pin getter methods
    int get_left_pin() const { return left_sensor_shutdown_pin_; }
    int get_right_pin() const { return right_sensor_shutdown_pin_; }

private:
    int left_sensor_shutdown_pin_;
    int right_sensor_shutdown_pin_;
    rclcpp::Logger logger_;
};

#endif // HEIGHT_SENSOR_CONFIGURATION_HPP
