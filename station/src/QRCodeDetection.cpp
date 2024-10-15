#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <cv_bridge/cv_bridge.h> // cv_bridge converts between ROS 2 image messages and OpenCV image representations.
#include <image_transport/image_transport.hpp> // Using image_transport allows us to publish and subscribe to compressed image streams in ROS2
#include <opencv2/opencv.hpp> // We include everything about OpenCV as we don't care much about compilation time at the moment.
#include <chrono>
#include <iostream>


// class QRCodeDetection : public rclcpp :: Node{
// public:
//     QRCodeDetection() : Node("qr_detection_node"), count_(0){
//         image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>
//     }
// private:
// };

int main(int argc ,char **argv){
    rclcpp::init(argc, argv);
    rclcpp::shutdown();
    return 0;
}