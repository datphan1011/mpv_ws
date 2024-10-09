//--------------------------------------------
//Standard library
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
//--------------------------------------------

class LockStepper : public rclcpp::Node{
public:
    LockStepper() : Node("lock_stepper_node"){
        // Subscriptions for limit switches
        left_lock_state_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "limit_switch1", 10,std::bind(&LockStepper::left_state_callback,this, std::placeholders::_1));
        right_lock_state_sub_= this->create_subscription<std_msgs::msg::Bool>(
        "limit_switch2", 10, std::bind(&LockStepper::right_state_callback, this, std::placeholders::_1));
        // Publishers to unlock the limit switches
        left_locker_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        "left_unlock", 10);
        right_locker_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        "right_unlock", 10);
    }
private:
    // Subscriptions for limit switches
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr left_lock_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr right_lock_state_sub_;
    //Publisher for limit switches
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr left_locker_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr right_locker_pub_;
    // Call back function
    void left_state_callback(const std_msgs::msg::Bool::SharedPtr msg){
        if (msg->data == false) {
            RCLCPP_INFO(this->get_logger(), "Left limit switch is unlocked");
            // You can add logic here to lock it if needed
            lock_limit_switches();  // Lock after unlock is detected
        } else {
            RCLCPP_INFO(this->get_logger(), "Left limit switch is locked");
            unlock_limit_switches(); // Unlock after lock is detected
        }
    }
    void right_state_callback(const std_msgs::msg::Bool::SharedPtr msg){
            if (msg->data == false) {
            RCLCPP_INFO(this->get_logger(), "Right limit switch is unlocked");
            // You can add logic here to lock it if needed
            lock_limit_switches();  // Lock after unlock is detected
        } else {
            RCLCPP_INFO(this->get_logger(), "Right limit switch is locked");
            unlock_limit_switches();  // Unlock after lock is detected
        }
    }
    // this function sent a message to both of the locker to unlock their lock 
    void unlock_limit_switches(){
        auto unlock_msg = std_msgs::msg::Bool();
        unlock_msg.data = false; // false mean unlock the Limit Switch

        // Publish to unlock both switch
        left_locker_pub_->publish(unlock_msg);
        right_locker_pub_->publish(unlock_msg);

        RCLCPP_INFO(this->get_logger(), "Unlock commands sent to both limit switches");
    }
    // this function sent a message to both of the locker to lock
    void lock_limit_switches(){
        auto lock_msg = std_msgs::msg::Bool();
        lock_msg.data = true;

        // Publish to lock both switch
        left_locker_pub_->publish(lock_msg);
        right_locker_pub_->publish(lock_msg);

        RCLCPP_INFO(this->get_logger(), "Lock commands sent to both limit switches");
    }
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LockStepper>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}