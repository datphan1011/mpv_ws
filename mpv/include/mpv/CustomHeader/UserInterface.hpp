#ifndef USER_INTERFACE_HPP
#define USER_INTERFACE_HPP

#include <rclcpp/rclcpp.hpp>

// Use inline here to avoid multiple definition error.
inline void user_menu_interface_options(const rclcpp::Logger &logger){
    RCLCPP_INFO_STREAM(logger, "--------------------------------");
    RCLCPP_INFO_STREAM(logger, "(u) for unload module (from MPV)");
    RCLCPP_INFO_STREAM(logger, "(l) for load module (on MPV)");
    RCLCPP_INFO_STREAM(logger, "(p) for position 0 (on station)");
    RCLCPP_INFO_STREAM(logger, "(o) to move lockpins out");
    RCLCPP_INFO_STREAM(logger, "(i) to move lockpins in");
    RCLCPP_INFO_STREAM(logger, "(h) adjust height of the station");
    RCLCPP_INFO_STREAM(logger, "(q) quit the program");
    RCLCPP_INFO_STREAM(logger, "--------------------------------");
}

#endif