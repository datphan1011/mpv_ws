#include <iostream>
#include <functional>
// #include <wiringPi.h>
// #include <wiringPiI2C.h>
#include <pigpio.h>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rclcpp/rclcpp.hpp>

// I2C address and WiringPi pin definitions
#define PCA9685_ADDR 0x40
#define PWM_FREQUENCY 1600  // PCA9685 PWM a typical of 24 Hz to 1526 Hz with a duty cycle that is adjustable from 0 % to 100 %
#define PWM_PIN 0  // Modify this to match your setup
#define READ_PIN 1 // Modify this to match your setup

// Register addresses for the PCA9685
#define PCA9685_MODE1 0x00
#define PCA9685_MODE2 0x01
#define PCA9685_LED0_ON_L 0x06
#define PCA9685_PRESCALE 0xFE

class Gripper : public rclcpp::Node{
public:
    Gripper() : Node("gripper_node"){
        // Subscribe to the "station_control" topic
        gripper_state_sub_ = this->create_subscription<std_msgs::msg::String>(
            "station_control", 10, std::bind(&Gripper::station_callback, this, std::placeholders::_1)
        );
        // Publisher to publish the gripper state (0 or 1) on the "gripper_state" topic
        gripper_state_pub_ = this->create_publisher<std_msgs::msg::Bool>("gripper_state", 10);

        // Timer to publish the current state periodically
        gripper_state_timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&Gripper::current_state, this)
        );

        // Initialize pigpio
        if (gpioInitialise() < 0) {
            RCLCPP_INFO(this->get_logger(), "Failed to initialize pigpio");
            rclcpp::shutdown();
        }

        // Initialize I2C communication with the PCA9685
        pca9685_fd_ = i2cOpen(1, PCA9685_ADDR, 0);
        if(pca9685_fd_ < 0){
            RCLCPP_INFO(this->get_logger(), "Failed to initialize the I2C");
            rclcpp::shutdown();
        }

        // Set PWM Frequency 
        set_pwm_frequency(PWM_FREQUENCY);
    }
    ~Gripper(){
        // Reset the PWM signal on the actuator
        set_pwm(PWM_PIN, 0, 0);
        i2cClose(pca9685_fd_);  // Close the I2C connection
        gpioTerminate(); // Terminate pigpio
    }
private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gripper_state_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_state_pub_;
    rclcpp::TimerBase::SharedPtr gripper_state_timer_;
    int pca9685_fd_;                    // File descriptor for I2C communication
    bool gripper_open_ = false; // Track the state of the gripper
    
    // Set PWM Frequency function   
    void set_pwm_frequency(int freq){
        float prescale_value = 25000000;  // Clock speed of the PCA9685 is 25MHz
        prescale_value /= 4096.0f;        // 12-bit = 2^12 = 4096
        prescale_value /= float(freq);    // prescale divide to the frequency in float
        prescale_value -= 1.0f;
        /*The static_cast<int> converts prescale_value to an integer type, ensuring that the prescale register gets an integer value.
        Adding 0.5f ensures proper rounding of the floating-point value to the nearest whole number.*/
        int prescale = static_cast<int>(prescale_value + 0.5f);

        int oldmode = i2cReadByteData(pca9685_fd_, PCA9685_MODE1);  // Reads the current MODE1 register value from the PCA9685 using the WiringPi function wiringPiI2CReadReg8
        int newmode = (oldmode & 0x7F) | 0x10; // sleep
        i2cWriteByteData(pca9685_fd_, PCA9685_MODE1, newmode);  // Writes newmode back to the MODE1 register of the PCA9685, putting it in sleep mode so that we can safely set the prescale value.
        i2cWriteByteData(pca9685_fd_, PCA9685_PRESCALE, prescale);  // Sets the PRESCALE register of the PCA9685 to the calculated prescale value using WiringPi's wiringPiI2CWriteReg8 function. This sets the desired PWM frequency.
        i2cWriteByteData(pca9685_fd_, PCA9685_MODE1, oldmode);  // Restores the original MODE1 register value, waking the device from sleep.
        gpioDelay(10);   // Pauses execution for 10 milliseconds, giving the device time to process the register changes. (Can increase if it too fast)
        i2cWriteByteData(pca9685_fd_, PCA9685_MODE1, oldmode | 0x80);
    }

    // Set PWM duty cycle
    /*The function configures the PCA9685 to generate a PWM signal on a specific channel by defining the start (on) and end (off) times of the PWM pulse*/
    void set_pwm(int channel, int on, int off) {
        i2cWriteByteData(pca9685_fd_, PCA9685_LED0_ON_L + 4 * channel, on & 0xFF);
        i2cWriteByteData(pca9685_fd_, PCA9685_LED0_ON_L + 4 * channel + 1, on >> 8);
        i2cWriteByteData(pca9685_fd_, PCA9685_LED0_ON_L + 4 * channel + 2, off & 0xFF);
        i2cWriteByteData(pca9685_fd_, PCA9685_LED0_ON_L + 4 * channel + 3, off >> 8);
    }

    // Callback function to process incoming messages on /station_control
    void station_callback(const std_msgs::msg::String::SharedPtr msg){
        RCLCPP_INFO(this->get_logger(), "Receive data from the station. %s", msg->data.c_str());    // print the command onto the console
        if(msg->data == "GRI_UN"){      // Check if the message sent by station is satisfied with the condition
            unlock();                   // call the function to do task
            RCLCPP_INFO(this->get_logger(), "Gripper unlocked");    // Notify the user by printed out to the console 
        }
        else if(msg->data == "GRI_ST"){ // Check if the message sent by station is satisfied with the condition
            stop();                     // call the function to do task
            RCLCPP_INFO(this->get_logger(), "Gripper stop");
        }
        else{
            RCLCPP_INFO(this->get_logger(), "Unknown command");
        }
    }
    // Unlock the gripper
    void unlock(){
        set_pwm(PWM_PIN, 0, 4095); // Set the actuator to HIGH
        gripper_open_ = true;
    }
    // Lock the gripper
    void stop(){
        set_pwm(PWM_PIN, 0, 0); // Set the actuator to LOW
        gripper_open_ = false;
    }

    // Publish current state of the Gripper
    void current_state(){
        auto state_msg = std_msgs::msg::Bool();
        state_msg.data = gripper_open_;
        gripper_state_pub_->publish(state_msg);
    }
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Gripper>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
