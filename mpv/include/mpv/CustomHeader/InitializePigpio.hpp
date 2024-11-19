// Pigpio_initialise_mpv.hpp

#ifndef PIGPIO_INITIALIZER_HPP
#define PIGPIO_INITIALIZER_HPP

#include <pigpio.h>
#include <rclcpp/rclcpp.hpp>

int initialize_PIGPIO(const rclcpp::Logger &logger) {
    if (gpioInitialise() < 0) {
        RCLCPP_ERROR(logger, "Failed to initialize pigpio library");
        return -1;  // Return error code
    }
    RCLCPP_INFO(logger, "pigpio library initialized successfully");
    return 0;  // Success
}

#endif  // PIGPIO_INITIALIZER_HPP