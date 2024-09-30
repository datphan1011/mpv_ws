#include <iostream>
#include <memory>
#include <chrono>
#include <string>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class StationControl : public rclcpp::Node {
public:
    StationControl() : Node("station_control_node") {
        // Subscription to the /control topic
        station_control_sub_ = this->create_subscription<std_msgs::msg::String>(
            "control", 10, std::bind(&StationControl::control_callback, this, std::placeholders::_1));
        
        // Publisher to the /station_control topic
        station_control_pub_ = this->create_publisher<std_msgs::msg::String>("station_control", 10);
        
        // Timer to publish data periodically
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&StationControl::send_data, this));
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr station_control_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr station_control_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string last_command_; // Store the last received command

    // Callback function to handle incoming messages on /control topic
    void control_callback(const std_msgs::msg::String::SharedPtr msg) {
        last_command_ = msg->data; // Update last command with received message
        RCLCPP_INFO(this->get_logger(), "Command Received from MPV: %s", msg->data.c_str());
        // Check for "UNLOAD" command and publish "HEIGHT"
        if (last_command_ == "UNLOAD") {
            auto response_msg = std_msgs::msg::String();
            response_msg.data = "HEIGHT";
            station_control_pub_->publish(response_msg);
            RCLCPP_INFO(this->get_logger(), "Station responding with HEIGHT");
        }
    }

    // Function to publish data to the /station_control topic
    void send_data() {
        if (!last_command_.empty()) { // Ensure there's a command to process
            auto message = std_msgs::msg::String();
            message.data = "Station processing command: " + last_command_; // Add a space after the colon
            station_control_pub_->publish(message);
            RCLCPP_INFO(this->get_logger(), "Station sending data: %s", message.data.c_str());
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StationControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
