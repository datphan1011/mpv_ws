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
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr station_control_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr station_control_pub_;
    std::string last_command_; // Store the last received command

    // Callback function to handle incoming messages on /control topic
    void control_callback(const std_msgs::msg::String::SharedPtr msg) {
        last_command_ = msg->data;  //update the lastest command with the receive messages
        RCLCPP_INFO(this->get_logger(), "Command received from MPV. %s", msg->data.c_str());
        // Check the message from the MPV 
        // Switch-case can't not be use for string so use if-else  
        if(last_command_ == "UNLOAD"){  
            perform_unload_task();  // call the matching function
        }
        else if(last_command_ == "LOAD"){
            perform_load_task();    // call the matching function
        }
        else if(last_command_ == "INITIAL_POSE"){
            perform_initial_pose_task();    // call the matching function
        }
        else if(last_command_ == "LOCKIN"){
            perform_lockin_task();     // call the matching function
        }
        else if(last_command_ == "LOCKOUT"){
            perform_lockout_task(); // call the matching function
        }
        else if(last_command_ == "HEIGHT"){
            perform_height_task();  // call the matching function
        }
        else if(last_command_ == "QUIT"){
            RCLCPP_INFO(this->get_logger(), "The station has been shutdown");
            rclcpp::shutdown();
        }
    }
    // Do unload task
    void perform_unload_task(){
        send_message("HEIGHT");     // Send HEIGHT to the /station_control topic
        std::this_thread::sleep_for(std::chrono::seconds(5));  // delay 5 seconds for each messages
        send_message("UNLOAD");  // Send UNLOAD to the /station_control topic
        RCLCPP_INFO(this->get_logger(), "UNLOAD task completed");   // Print out on the console
    }
    // Do load task
    void perform_load_task(){
        send_message("HEIGHT");     // Send HEIGHT to the /station_control topic
        std::this_thread::sleep_for(std::chrono::seconds(5));   // delay 5 seconds for each messages
        send_message("GRI_UN");     // Send GRI_UN to the /station_control topic
        std::this_thread::sleep_for(std::chrono::seconds(5));   // delay 5 seconds for each messages
        send_message("LOAD");       // Send LOAD to the /station_control topic
        RCLCPP_INFO(this->get_logger(), "LOAD task completed"); // Print out on the console
        std::this_thread::sleep_for(std::chrono::seconds(20));  //delay for 20 seconds wait for loading complete then send stop message
        send_message("GRI_ST");
    }
    // Do initial position task
    void perform_initial_pose_task(){
        send_message("HEIGHT");     // Send HEIGHT to the /station_control topic
        std::this_thread::sleep_for(std::chrono::seconds(5));   // delay for 5 seconds for each messages
        send_message("INITIAL_POSE");// Send INITIAL_POSE to the /station_control topic
        RCLCPP_INFO(this->get_logger(), "INITIAL_POSE task completed"); // Print out on the console
    }
    // Do lockin task
    void perform_lockin_task(){
        send_message("LOCKIN_START");  // Command to start lock-in process
        std::this_thread::sleep_for(std::chrono::seconds(5));
        send_message("LOCKIN_ENGAGE"); // Command to engage lock-in mechanism
        std::this_thread::sleep_for(std::chrono::seconds(5));
        send_message("LOCKIN_COMPLETE"); // Confirm completion of lock-in
        RCLCPP_INFO(this->get_logger(), "LOCKIN task completed");
    }

    // Do lock out task
    void perform_lockout_task(){
        send_message("LOCKOUT_START");  // Command to start lock-out process
        std::this_thread::sleep_for(std::chrono::seconds(5));
        send_message("LOCKOUT_DISENGAGE"); // Command to disengage lock-out mechanism
        std::this_thread::sleep_for(std::chrono::seconds(5));
        send_message("LOCKOUT_COMPLETE"); // Confirm completion of lock-out
        RCLCPP_INFO(this->get_logger(), "LOCKOUT task completed");
    }
   
    // Do HEIGHT task
    void perform_height_task(){
        send_message("HEIGHT");
        std::this_thread::sleep_for(std::chrono::seconds(5));   // delay for 5 seconds for each messages
        RCLCPP_INFO(this->get_logger(), "HEIGHT task completed");
    }   
    // Function to publish message to the /station_control topic
    void send_message(const std::string &data) {
        auto message = std_msgs::msg::String();     // Create a string message variable
        message.data = data;                        // add the data field of the data argument to the message data
        station_control_pub_->publish(message);     // Publish the message to the /station_control topic
        RCLCPP_INFO(this->get_logger(), "Published message: %s", message.data.c_str()); // print the message to the console
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StationControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
