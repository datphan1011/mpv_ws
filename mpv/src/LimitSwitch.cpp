#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

class LimitSwitch : public rclcpp::Node {
public:
    LimitSwitch() : Node("limit_switch_nodes") {
        // Create publishers for left and right limit switches
        left_limit_pub_ = this->create_publisher<std_msgs::msg::Bool>("limit_switch1", 10);
        right_limit_pub_ = this->create_publisher<std_msgs::msg::Bool>("limit_switch2", 10);

        // Timer to simulate publishing the state every second
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&LimitSwitch::limit_switch_state_publish, this)
        );

        // Initialize the state of the limit switches
        left_limitswitch_state = true;  // 1 (true) is locked
        right_limitswitch_state = true; // 1 (true) is locked
    }

private:
    // Publishers for the left and right limit switches
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr left_limit_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr right_limit_pub_;

    // Timer for periodic publishing
    rclcpp::TimerBase::SharedPtr timer_;

    // States of the limit switches (true for locked, false for unlocked)
    bool left_limitswitch_state;
    bool right_limitswitch_state;

    // Function to send the message to the lock stepper (Change when implemented on the MPV)
    void limit_switch_state_publish() {
        // Simulate alternating between locked and unlocked states for testing
        left_limitswitch_state = !left_limitswitch_state;
        right_limitswitch_state = !right_limitswitch_state;

        // Create and publish left switch state message
        auto left_msg = std_msgs::msg::Bool();
        left_msg.data = left_limitswitch_state;
        left_limit_pub_->publish(left_msg);
        RCLCPP_INFO(this->get_logger(), "Left limit switch state: %s", left_limitswitch_state ? "Locked" : "Unlocked");

        // Create and publish right switch state message
        auto right_msg = std_msgs::msg::Bool();
        right_msg.data = right_limitswitch_state;
        right_limit_pub_->publish(right_msg);
        RCLCPP_INFO(this->get_logger(), "Right limit switch state: %s", right_limitswitch_state ? "Locked" : "Unlocked");
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LimitSwitch>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
