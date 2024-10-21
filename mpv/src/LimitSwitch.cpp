#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include <pigpio.h>

class LimitSwitch : public rclcpp::Node {
public:
    LimitSwitch(int pin, std::string name, unsigned int edge, bool inverse = false)
        : Node(name), pin_(pin), inverse_(inverse) {
        gpioSetMode(pin_, PI_INPUT);
        gpioSetPullUpDown(pin_, PI_PUD_UP);

        // Register the static wrapper callback
        if (gpioSetISRFunc(pin_, edge, 0, LimitSwitch::callback_wrapper) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set ISR function for pin %d", pin_);
        }

        pub_limitswitch_ = this->create_publisher<std_msgs::msg::Bool>(name + "_state", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&LimitSwitch::publish_state, this));
    }

    ~LimitSwitch() {
        gpioSetISRFunc(pin_, RISING_EDGE, 0, NULL);  // Remove the ISR
    }

private:
    int pin_;
    bool inverse_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_limitswitch_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool state_ = false;

    static void callback_wrapper(int gpio, int level, uint32_t tick) {
        // Access the correct instance using the pin (assuming you only have a limited number of instances)
        LimitSwitch *instance = getInstanceByPin(gpio);  // You need to implement this function
        if (instance) {
            instance->callback(gpio, level, tick);
        }
    }

    void callback(int gpio, int level, uint32_t tick) {
        // Update the state
        (void)gpio;  // Mark gpio as intentionally unused
        (void)level;  // Mark level as intentionally unused
        (void)tick;   // Mark tick as intentionally unused
        state_ = getState();
        publish_state();
    }

    bool getState() {
        int gpio_value = gpioRead(pin_);
        return inverse_ ? !gpio_value : gpio_value;
    }

    void publish_state() {
        std_msgs::msg::Bool msg;
        msg.data = getState();
        pub_limitswitch_->publish(msg);
    }

    static LimitSwitch* getInstanceByPin(int pin) {
        // Implement a mechanism to return the correct instance for the pin.
        // For example, you can keep track of instances in a static map or array.
        // This part depends on how many instances you have and your design.
        (void)pin;
        return nullptr;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto limit_switch_1 = std::make_shared<LimitSwitch>(22, "limit_switch_1", RISING_EDGE);
    auto limit_switch_2 = std::make_shared<LimitSwitch>(25, "limit_switch_2", RISING_EDGE);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(limit_switch_1);
    executor.add_node(limit_switch_2);

    executor.spin();
    rclcpp::shutdown();
    return 0;
}
