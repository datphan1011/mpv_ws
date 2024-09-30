#include <iostream>
#include <functional>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rclcpp/rclcpp.hpp>

class Gripper : public rclcpp::Node{
public:
    Gripper() : Node("gripper_node"){
        // Subscribe to the "station_control" topic
        gripper_state_sub_ = this->create_subscription<std_msgs::msg::String>(
            "station_control", 10, std::bind(&Gripper::state_callback, this, std::placeholders::_1)
        );
        
        // Publisher to publish the gripper state (0 or 1) on the "gripper_state" topic
        gripper_state_pub_ = this->create_publisher<std_msgs::msg::Bool>("gripper_state", 10);

        // Timer to publish the current state periodically
        gripper_state_timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&Gripper::current_state, this)
        );
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gripper_state_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_state_pub_;
    rclcpp::TimerBase::SharedPtr gripper_state_timer_;
    bool gripper_open_ = false; // Track the state of the gripper

    // Callback function to process incoming messages on /station_control
    void state_callback(const std_msgs::msg::String::SharedPtr msg){
        RCLCPP_INFO(this->get_logger(), "Received data from /station_control: %s", msg->data.c_str());

        if (msg->data == "GRI-UNLOCK") {
            gripper_open_ = true;  // Unlock gripper (open)
        } else if (msg->data == "GRI-LOCK") {
            gripper_open_ = false; // Lock gripper (close)
        }
    }

    // Function to publish the current state of the gripper
    void current_state() {
        auto gripper_state_msg = std_msgs::msg::Bool();
        gripper_state_msg.data = gripper_open_;
        gripper_state_pub_->publish(gripper_state_msg);

        RCLCPP_INFO(this->get_logger(), "Publishing gripper state: %s", gripper_open_ ? "OPEN" : "CLOSED");
    }
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Gripper>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
