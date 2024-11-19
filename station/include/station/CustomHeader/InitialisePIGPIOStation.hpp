// Pigpio_initialise_station.hpp

#ifndef INITIALISE_PIGPIO_STATION_HPP
#define INITIALISE_PIGPIO_STATION_HPP

#include <pigpio.h>
#include <rclcpp/rclcpp.hpp>

int initialize_PIGPIO_station(const rclcpp::Logger &logger) {
    if (gpioInitialise() < 0) {
        RCLCPP_ERROR(logger, "Failed to initialize pigpio library");
        return -1;  // Return error code
    }
    RCLCPP_INFO(logger, "pigpio library initialized successfully");
    return 0;  // Success
}

#endif  // INITIALISE_PIGPIO_STATION_HPP