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
class Client: public rclcpp::Node {
public:
    Client() : Node("client_node") {
        // Initialize the client to call MPVAndStation service
        client_ = this->create_client<my_custom_service::srv::MPVAndStation>("server_service");

        // Timer to periodically send requests (every 2 seconds)
        timer_ = this->create_wall_timer(
            std::chrono::seconds(2), std::bind(&Client::send_request, this));

        // Subscription to the MPV control topic
        mpv_control_sub_ = this->create_subscription<std_msgs::msg::String>(
            "mpv_control", 10, std::bind(&Client::command_callback, this, std::placeholders::_1));

        // Subscriptions to left and right height sensors
        left_height_sensor_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "left_height_sensor", 10, std::bind(&Client::left_height_sensor_callback, this, std::placeholders::_1));
        right_height_sensor_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "right_height_sensor", 10, std::bind(&Client::right_height_sensor_callback, this, std::placeholders::_1));

        // Initial values for command and sensor data
        last_command_ = "";
        last_left_sensor_data_ = 0;
        last_right_sensor_data_ = 0;
    }
private:
    //class member variables
    // ROS2 Client and Timer
    rclcpp::Client<my_custom_service::srv::MPVAndStation>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;

    // ROS2 Subscriptions
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mpv_control_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr left_height_sensor_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr right_height_sensor_sub_;
    
    // Variables for storing last received data
    std::string last_command_;
    int32_t last_left_sensor_data_;
    int32_t last_right_sensor_data_;
    bool data_sent_ = false; // Check if data has been sent?

    //Callback function for left height sensor data (corrected logging)
    void left_height_sensor_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Client received left height data: %d", msg->data);
        last_left_sensor_data_ = msg->data;
        send_request();
    }

    //Callback function for right height sensor data (corrected logging and parentheses)
    void right_height_sensor_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Client received right height data: %d", msg->data);
        last_right_sensor_data_ = msg->data;
        send_request();
    }

    void command_callback(const std_msgs::msg::String::SharedPtr msg){
        RCLCPP_INFO(this->get_logger(), "Client received command: %s", msg->data.c_str());
        last_command_ = msg->data;
        send_request();
    }

    void send_request() {
        if (last_command_.empty()){
            RCLCPP_INFO(this->get_logger(),"Waiting for command before sending to the server");
            return;
        }
        if (data_sent_){
            RCLCPP_INFO(this->get_logger(), "Data already sent to the server. Waiting for new data...");
            return;
        }
        if (!client_->wait_for_service(std::chrono::seconds(5))){
            RCLCPP_ERROR(this->get_logger(), "Server service not available");
            return;
        }
        // Prepare the service request
        auto request = std::make_shared<my_custom_service::srv::MPVAndStation::Request>();
        request->command = last_command_;
        request->left_sensor_data = last_left_sensor_data_;   
        request->right_sensor_data = last_right_sensor_data_;
        // Send the service request and handle the response
        auto future = client_->async_send_request(request);
        try {
            auto response = future.get();
            
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "Response Command: %s", response->response_command.c_str());
                RCLCPP_INFO(this->get_logger(), "Response Left Sensor: %d", response->response_left_sensor);
                RCLCPP_INFO(this->get_logger(), "Response Right Sensor: %d", response->response_right_sensor);
                RCLCPP_INFO(this->get_logger(), "Message: %s", response->message.c_str());
            } else {
                RCLCPP_WARN(this->get_logger(), "Service call failed: %s", response->message.c_str());
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Client>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
