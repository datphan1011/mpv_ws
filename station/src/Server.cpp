#include <rclcpp/rclcpp.hpp>
#include <my_custom_service/srv/mpv_and_station.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>  

class Server : public rclcpp::Node {
public:
    Server() : Node("server_node") {
        // Create a service that listens for incoming service requests
        service_ = this->create_service<my_custom_service::srv::MPVAndStation>(
            "server_service", std::bind(&Server::handle_service, this, std::placeholders::_1, std::placeholders::_2));
        // Publisher to send messages on a topic (command and sensor data)
        pub_control_ = this->create_publisher<std_msgs::msg::String>("control", 10);
        pub_left_sensor_ = this->create_publisher<std_msgs::msg::Int32>("left_sensor", 10);
        pub_right_sensor_ = this->create_publisher<std_msgs::msg::Int32>("right_sensor", 10);
    }

private:
    // Class member variables
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_control_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_left_sensor_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_right_sensor_;
    rclcpp::Service<my_custom_service::srv::MPVAndStation>::SharedPtr service_;

    // Handle service requests
    void handle_service(const std::shared_ptr<my_custom_service::srv::MPVAndStation::Request> request,
                        std::shared_ptr<my_custom_service::srv::MPVAndStation::Response> response) {
        // Log the received command and sensor data
        RCLCPP_INFO(this->get_logger(), "Received command: %s", request->command.c_str());
        RCLCPP_INFO(this->get_logger(), "Left Sensor Data: %d", request->left_sensor_data);
        RCLCPP_INFO(this->get_logger(), "Right Sensor Data: %d", request->right_sensor_data);

        // Respond with the data received
        response->response_command = "Command received and processed";
        response->response_left_sensor = request->left_sensor_data;
        response->response_right_sensor = request->right_sensor_data;
        response->success = true;
        response->message = "Data processed and response sent back";

        // Publish a message to the /control topic (Command message)
        auto msg_command = std_msgs::msg::String();
        msg_command.data = "Processed: " + request->command;
        pub_control_->publish(msg_command);

        // Publishing sensor data of the MPV height sensor
        auto msg_left_sensor_data = std_msgs::msg::Int32();
        auto msg_right_sensor_data = std_msgs::msg::Int32();
        msg_left_sensor_data.data = request->left_sensor_data;   
        msg_right_sensor_data.data = request->right_sensor_data; 
        pub_left_sensor_->publish(msg_left_sensor_data);
        pub_right_sensor_->publish(msg_right_sensor_data);

        RCLCPP_INFO(this->get_logger(), "Response and sensor data sent back to client.");
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Server>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
