//--------------------------------------------
//Standard library
#include <iostream>
#include <pigpio.h>
#include <chrono>
#include <cmath>
#include <thread>
#include <pigpio.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
//--------------------------------------------
//Include custom library
#include "mpv/CustomHeader/InitializePigpio.hpp"
//--------------------------------------------

class LockStepper : public rclcpp::Node{
public:
    static constexpr int IN = PI_LOW;
    static constexpr int OUT = PI_HIGH;
    LockStepper(int dirPin, int pulPin, int stepCount = 2150, int maxSpeed = 500, int acceleration = 100)
        : Node("lock_stepper_node"), dirPin_(dirPin), pulPin_(pulPin),
          stepCount_(stepCount), maxSpeed_(maxSpeed), acceleration_(acceleration),
          currentSpeed_(0.0){

        // Subscriptions for limit switches
        left_lock_state_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "limit_switch1", 10, std::bind(&LockStepper::left_state_callback, this, std::placeholders::_1));
        right_lock_state_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "limit_switch2", 10, std::bind(&LockStepper::right_state_callback, this, std::placeholders::_1));
        // Subscription to MPV control
        mpv_control_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/MPV_control", 10, std::bind(&LockStepper::mpv_control_callback, this, std::placeholders::_1));

        // State tracking for limit switches
        lm1_active_ = false;
        lm2_active_ = false;
        // Initialize pigpio
        initialize_PIGPIO(this->get_logger());
        // Set pin
        gpioSetMode(dirPin_, PI_OUTPUT);
        gpioSetMode(pulPin_, PI_OUTPUT);     
    }
    ~LockStepper() {
        gpioWrite(dirPin_, PI_LOW);
        gpioWrite(pulPin_, PI_LOW);
        gpioTerminate();
    }
private:
    int dirPin_, pulPin_;
    int stepCount_, maxSpeed_, acceleration_;
    double currentSpeed_;
    bool lm1_active_, lm2_active_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr left_lock_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr right_lock_state_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mpv_control_sub_;

    void left_state_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        lm1_active_ = msg->data;
        control_stepper();
    }

    void right_state_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        lm2_active_ = msg->data;
        control_stepper();
    }

    void mpv_control_callback(const std_msgs::msg::String::SharedPtr msg) {
        std::string info = msg->data;
        if (info == "LOCKIN") {
            RCLCPP_INFO(this->get_logger(), "Moving IN");
            move(IN);
        } else if (info == "LOCKOUT") {
            RCLCPP_INFO(this->get_logger(), "Moving OUT");
            move(OUT);
        }
    }
        void move(int direction) {
        gpioWrite(dirPin_, direction);
        for (int i = 0; i < stepCount_; i++) {
            currentSpeed_ = calculate_speed(i);
            double stepDelayHalf = 0.5 / currentSpeed_;
            gpioWrite(pulPin_, PI_HIGH);
            std::this_thread::sleep_for(std::chrono::duration<double>(stepDelayHalf));
            gpioWrite(pulPin_, PI_LOW);
            std::this_thread::sleep_for(std::chrono::duration<double>(stepDelayHalf));
        }
        gpioWrite(dirPin_, PI_LOW);
    }

    double calculate_speed(int currentStep) {
        int distanceToGo = stepCount_ - currentStep;
        double requiredSpeed = 0;

        if (distanceToGo == 0) {
            return 0.0;
        } else {
            requiredSpeed = std::sqrt(2.0 * distanceToGo * acceleration_);
        }

        if (requiredSpeed > currentSpeed_) {
            requiredSpeed = currentSpeed_ == 0 ? std::sqrt(2.0 * acceleration_)
                                               : currentSpeed_ + std::abs(acceleration_ / currentSpeed_);
            if (requiredSpeed > maxSpeed_) requiredSpeed = maxSpeed_;
        } else if (requiredSpeed < currentSpeed_) {
            requiredSpeed = currentSpeed_ == 0 ? -std::sqrt(2.0 * acceleration_)
                                               : currentSpeed_ - std::abs(acceleration_ / currentSpeed_);
            if (requiredSpeed < -maxSpeed_) requiredSpeed = -maxSpeed_;
        }

        return requiredSpeed;
    }

    void control_stepper() {
        if (lm1_active_ && lm2_active_) {
            RCLCPP_INFO(this->get_logger(), "Both limit switches active. Moving OUT.");
            move(OUT);
        } else if (lm1_active_) {
            RCLCPP_INFO(this->get_logger(), "Left limit switch active. Moving OUT.");
            move(OUT);
        } else if (lm2_active_) {
            RCLCPP_INFO(this->get_logger(), "Right limit switch active. Moving OUT.");
            move(OUT);
        } else {
            RCLCPP_INFO(this->get_logger(), "No limit switch active. Moving IN.");
            move(IN);
        }
    }
    // int initialize_PIGPIO() {
    //     if (gpioInitialise() < 0) {
    //         RCLCPP_ERROR(this->get_logger(), "Failed to initialize pigpio library");
    //         return -1;  // Return error code
    //     }
    //     RCLCPP_INFO(this->get_logger(), "pigpio library initialized successfully");
    //     return 0;  // Success
    // }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto stepper1 = std::make_shared<LockStepper>(17, 27);
    auto stepper2 = std::make_shared<LockStepper>(23, 24);

    try {
        rclcpp::spin(stepper1);
        rclcpp::spin(stepper2);
    } catch (const std::exception &e) {
        std::cerr << "Exception: " << e.what() << std::endl;
    }
    rclcpp::shutdown();
    return 0;
}