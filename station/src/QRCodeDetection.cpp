#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <cv_bridge/cv_bridge.h> // cv_bridge converts between ROS 2 image messages and OpenCV image representations.
#include <image_transport/image_transport.hpp> // Using image_transport allows us to publish and subscribe to compressed image streams in ROS2
#include <opencv2/opencv.hpp> // We include everything about OpenCV as we don't care much about compilation time at the moment.
#include <chrono>
#include <iostream>


class QRCodeDetection : public rclcpp::Node {
public:
    int dst_to_go_h = 0, dst_to_go_v = 0;
    
    QRCodeDetection();
    ~QRCodeDetection();
    void calibrate();
    void measure();
    void enableDisplay(bool en);

private:
    int midSTN = 0, midMPV = 0, horizontal_bias = 0, vertical_bias = 0;
    bool display_enable = false;

    map<string, string> data_dictionary;
    VideoCapture cap;

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr height_sensor_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr linear_actuator_publisher_;

    void readFile();
    int readQR(const Mat &frame, map<string, BarcodeData> &scannedCodes);
    void writeFile(int B1, int B2);
    void processFrame(const Mat &frame);
    void publishData(int dstToGoH); // Updated to only one parameter

};
int main(int argc ,char **argv){
    rclcpp::init(argc, argv);
    rclcpp::shutdown();
    return 0;
}