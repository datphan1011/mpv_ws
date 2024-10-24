#include <pigpio.h>                      // Pigpio library for handling GPIO
#include "rclcpp/rclcpp.hpp"             // ROS2 core library for creating nodes
#include "std_msgs/msg/bool.hpp"         // Standard ROS2 message type (Boolean)

// Define a class for the LimitSwitch, inheriting from rclcpp::Node
class LimitSwitch : public rclcpp::Node {
public:
    // Constructor that initializes the limit switch, sets up the GPIO pin, and ROS2 publisher
    LimitSwitch(int pin, const std::string &name, int edge, bool inverse = false)
    : Node(name), pin_(pin), state_(false), inverse_(inverse) {
        pub_limitswitch_ = create_publisher<std_msgs::msg::Bool>("/" + name + "_state", 10);  // Create ROS2 publisher
        timer_ = create_wall_timer(std::chrono::milliseconds(50), std::bind(&LimitSwitch::publish_state, this));  // Set timer to publish state every 50ms

        gpioInitialise();  // Initialize the pigpio library
        gpioSetMode(pin_, PI_INPUT);     // Set the pin as input mode
        gpioSetPullUpDown(pin_, PI_PUD_UP);  // Enable pull-up resistor on the pin
        gpioSetISRFunc(pin_, edge, 0, &LimitSwitch::callback, this);  // Attach ISR for edge detection
    }

    // Destructor cleans up the GPIO pins
    ~LimitSwitch() {
        gpioTerminate();  // Close GPIO
    }

private:
    int pin_;                // GPIO pin number
    bool state_;             // Current state of the limit switch
    bool inverse_;           // Whether to invert the state
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_limitswitch_;  // ROS2 publisher for switch state
    rclcpp::TimerBase::SharedPtr timer_;  // Timer to periodically publish the state

    // Method to get the current state of the limit switch
    bool getState() {
        int pin_state = gpioRead(pin_);  // Read the pin state (high or low)
        return inverse_ ? !pin_state : pin_state;  // Return inverted state if specified, otherwise return actual state
    }

    // Callback method called when an edge event is detected on the GPIO pin
    static void callback(int gpio, int level, uint32_t tick, void *user_data) {
        auto *obj = static_cast<LimitSwitch*>(user_data);  // Access the LimitSwitch instance
        obj->state_ = obj->getState();  // Update the state
        obj->publish_state();  // Publish the state to ROS2 topic
    }

    // Method to publish the current state of the limit switch
    void publish_state() {
        auto msg = std::make_shared<std_msgs::msg::Bool>();  // Create a new Bool message
        msg->data = getState();  // Set the message data to the current state
        pub_limitswitch_->publish(*msg);  // Publish the state to the ROS2 topic
    }
};

// Main function to initialize ROS2 and run the node
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);  // Initialize ROS2

    // Create two limit switch nodes, each handling a different GPIO pin
    auto limit_switch_1 = std::make_shared<LimitSwitch>(22, "limit_switch_1", FALLING_EDGE, false);
    auto limit_switch_2 = std::make_shared<LimitSwitch>(25, "limit_switch_2", FALLING_EDGE, false);

    rclcpp::spin(limit_switch_1);  // Keep the first node alive and running
    rclcpp::shutdown();  // Shutdown ROS2 after the node stops
    return 0;
}
