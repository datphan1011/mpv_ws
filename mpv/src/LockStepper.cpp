#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

class LockStepper : public rclcpp::Node{
public:
    LockStepper() : Node("lock_stepper_node"){
        // Subscriptions for limit switches
        left_lock_state_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "left_limit_switch", 10,std::bind(&LockStepper::left_state_callback,this, std::placeholders::_1));
        right_lock_state_sub_= this->create_subscription<std_msgs::msg::Bool>(
        "right_limit_switch", 10, std::bind(&LockStepper::right_state_callback, this, std::placeholders::_1));
        // Publishers to unlock the limit switches
        left_locker_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        "left_unlock", 10);
        right_locker_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        "right_unlock", 10);
    }
private:
    // Subscriptions for limit switches
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr left_lock_state_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr right_lock_state_pub_;
    //Publisher for limit switches
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr left_locker_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr right_locker_pub_;
    // Call back function
    void left_state_callback(const std_msgs::msgs::Bool::SharedPtr msg){
        // TODO: Add the logic of this function
        if (msg->data == false){
            RCLCPP_INFO(this->get_logger(), "left limit switch is open");
        }
    }
    void right_state_callback(const std_msgs::msgs::Bool::SharedPtr msg){
        // TODO: Add the logic of this function
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
}

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LockStepper>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}