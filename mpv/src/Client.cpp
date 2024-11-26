//--------------------------------------------
//Standard library
#include <iostream>
#include <string> 
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/int32.hpp>
#include <my_custom_service/srv/mpv_and_station.hpp>  // Include the custom service header
//--------------------------------------------
class Client : public rclcpp::Node {
public:
    Client() : Node("client_node") {
        // Initialize the client to call MPVAndStation service
        client_ = this->create_client<my_custom_service::srv::MPVAndStation>("server_service");

        // Subscription to the MPV control topic
        mpv_control_sub_ = this->create_subscription<std_msgs::msg::String>(
            "mpv_control", 10, std::bind(&Client::command_callback, this, std::placeholders::_1));

        // Subscriptions to left and right height sensors
        left_height_sensor_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "left_height_sensor_mpv", 10, std::bind(&Client::left_height_sensor_callback, this, std::placeholders::_1));
        right_height_sensor_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "right_height_sensor_mpv", 10, std::bind(&Client::right_height_sensor_callback, this, std::placeholders::_1));

        // Timer for continuously sending sensor data
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&Client::send_sensor_data, this));

        // Initialize variables
        last_command_ = "";
        last_left_sensor_data_ = 0;
        last_right_sensor_data_ = 0;
        command_sent_ = false;
    }

private:
    // ROS2 Client and Subscriptions
    rclcpp::Client<my_custom_service::srv::MPVAndStation>::SharedPtr client_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mpv_control_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr left_height_sensor_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr right_height_sensor_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Variables for storing data
    std::string last_command_;
    int32_t last_left_sensor_data_;
    int32_t last_right_sensor_data_;
    bool command_sent_;

    // Callback for left height sensor data
    void left_height_sensor_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Client received left height data: %d", msg->data);
        last_left_sensor_data_ = msg->data;
    }

    // Callback for right height sensor data
    void right_height_sensor_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Client received right height data: %d", msg->data);
        last_right_sensor_data_ = msg->data;
    }

    // Callback for MPV control commands
    void command_callback(const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Client received command: %s", msg->data.c_str());
        last_command_ = msg->data;
        command_sent_ = false;  // Reset the flag to allow sending this new command
        send_command();  // Immediately process the new command
    }

    // Function to send the command to the server
    void send_command() {
        if (command_sent_) {
            RCLCPP_INFO(this->get_logger(), "Command already sent to the server. Waiting for a new command...");
            return;
        }

        if (!client_->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Server service not available");
            return;
        }

        auto request = std::make_shared<my_custom_service::srv::MPVAndStation::Request>();
        request->command = last_command_;
        request->left_sensor_data = last_left_sensor_data_;
        request->right_sensor_data = last_right_sensor_data_;

        // Send the request asynchronously
        client_->async_send_request(request, 
            [this](rclcpp::Client<my_custom_service::srv::MPVAndStation>::SharedFuture future_response) {
                try {
                    auto response = future_response.get();
                    if (response->success) {
                        RCLCPP_INFO(this->get_logger(), "Response Command: %s", response->response_command.c_str());
                        command_sent_ = true;  // Mark the command as processed
                    } else {
                        RCLCPP_WARN(this->get_logger(), "Service call failed: %s", response->message.c_str());
                    }
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
                }
            });

        RCLCPP_INFO(this->get_logger(), "Command sent to the server: %s", last_command_.c_str());
    }

    // Function to continuously send sensor data to the server
    void send_sensor_data() {
        // Check if the server service is available; wait for up to 5 seconds
        if (!client_->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Server service not available for sensor data");
            return;
        }
        // Create a new service request object
        auto request = std::make_shared<my_custom_service::srv::MPVAndStation::Request>();
        // Set the command field to an empty string since we're only sending sensor data
        request->command = "";  
        // Populate the request with the latest left and right sensor data
        request->left_sensor_data = last_left_sensor_data_;
        request->right_sensor_data = last_right_sensor_data_;

        // Send the service request asynchronously
        client_->async_send_request(request, 
            [](rclcpp::Client<my_custom_service::srv::MPVAndStation>::SharedFuture future_response) {
                try {
                    // Retrieve the server's response
                    auto response = future_response.get();
                    // Log a success message if the server processed the request successfully
                    if (response->success) {
                        RCLCPP_INFO(rclcpp::get_logger("client_node"), "Sensor data successfully sent.");
                    }
                } catch (const std::exception &e) {
                    // Log an error if the service call fails
                    RCLCPP_ERROR(rclcpp::get_logger("client_node"), "Sensor data call failed: %s", e.what());
                }
            });

        // Log the sensor data being sent to the server for debugging
        RCLCPP_INFO(this->get_logger(), "Sensor data sent to the server: Left: %d, Right: %d",
                    last_left_sensor_data_, last_right_sensor_data_);
    }
};

// Main code to run the program
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    // Create the node
    auto node = std::make_shared<Client>();
    // Use an executor to handle shutdown signals gracefully
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    RCLCPP_INFO(node->get_logger(), "Client node started. Press Ctrl+C to stop.");
    // Run the executor
    try {
        executor.spin();
    } catch (const std::exception &e) {
        RCLCPP_ERROR(node->get_logger(), "Exception in executor: %s", e.what());
    }
    // Shutdown cleanly
    RCLCPP_INFO(node->get_logger(), "Shutting down client node.");
    rclcpp::shutdown();
    return 0;
}

