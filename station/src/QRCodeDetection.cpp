#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <open

class QRCodeDetection : public rclcpp :: Node{
public:
    QRCodeDetection() : Node("qr_detection_node"){

    }
private:
};

int main(int argc , char argv**){
    rclcpp::init(argc, argv);
    rclcpp::shutdown();
    return 0;
}