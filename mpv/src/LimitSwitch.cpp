#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <pigpio.h>    // pigpio for GPIO control
#include <memory>      // For smart pointers

class LimitSwitch : public rclcpp::Node {
public:
    LimitSwitch(int pin, const std::string& name, int edge, bool inverse = false)
        : Node(name), pin_(pin), state_(false), inverse_(inverse) {
        
        // Initialize pigpio library
        if (gpioInitialise() < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize pigpio");
            return;
        }

        // Set the pin as input and enable pull-up resistor
        gpioSetMode(pin_, PI_INPUT);
        gpioSetPullUpDown(pin_, PI_PUD_UP);

        // Add event detection (edge detection)
        if (gpioSetISRFunc(pin_, edge, 0, &LimitSwitch::callback_wrapper) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set ISR function for pin %d", pin_);
        }

        // Create ROS2 publisher
        pub_limitswitch_ = this->create_publisher<std_msgs::msg::Bool>("/" + name + "_state", 10);

        // Set a timer to publish the state at a regular interval
        timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&LimitSwitch::publishState, this));

        RCLCPP_INFO(this->get_logger(), "LimitSwitch node initialized for pin %d", pin_);
    }

    ~LimitSwitch() {
        // Clean up GPIO resources
        gpioTerminate();
    }

private:
    int pin_;
    bool state_;
    bool inverse_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_limitswitch_;
    rclcpp::TimerBase::SharedPtr timer_;

    static void callback_wrapper(int gpio, int level, uint32_t tick, void* userData) {
        auto* self = reinterpret_cast<LimitSwitch*>(userData);
        self->callback(gpio, level, tick);
    }

    void callback(int gpio, int level, uint32_t tick) {
        if (gpio == pin_) {
            state_ = getState();
            publishState();
        }
    }

    bool getState() const {
        return inverse_ ? !gpioRead(pin_) : gpioRead(pin_);
    }

    void publishState() {
        auto msg = std_msgs::msg::Bool();
        msg.data = getState();
        pub_limitswitch_->publish(msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // Initialize two limit switches
    auto limit_switch_1 = std::make_shared<LimitSwitch>(22, "limit_switch_1", RISING_EDGE);
    auto limit_switch_2 = std::make_shared<LimitSwitch>(25, "limit_switch_2", RISING_EDGE);

    // Create a multi-threaded executor to handle multiple nodes
    rclcpp::executors::MultiThreadedExecutor executor;
    
    // Add both nodes to the executor
    executor.add_node(limit_switch_1);
    executor.add_node(limit_switch_2);
    
    // Spin the executor to handle both nodes
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}

