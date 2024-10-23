// standard cpp libraries
#include <iostream>
#include <fstream>
#include <vector>
#include <thread>
#include <chrono>
#include <map>
// Ros2 and custom libraries
#include <opencv2/opencv.hpp>   // We include everything about OpenCV as we don't care much about compilation time at the moment.
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int32.hpp>
#include <cv_bridge/cv_bridge.h>    // cv_bridge converts between ROS 2 image messages and OpenCV image representations.
#include <image_transport/image_transport.hpp>  // Using image_transport allows us to publish and subscribe to compressed image streams in ROS2

using namespace std;
using namespace cv;

class QRCodeDetection : public rclcpp::Node {
public:
    // Variables representing the distance to go (horizontal and vertical)
    int dst_to_go_h = 0, dst_to_go_v = 0;

    // Constructor: Initializes the node and sets up the camera and publishers
    QRCodeDetection();
    
    // Destructor: Cleans up the camera resource
    ~QRCodeDetection();
    
    // Calibration routine to align QR codes and set horizontal/vertical biases
    void calibrate();
    
    // Measurement routine to continuously detect and process QR codes from the camera
    void measure();
    
    // Enable or disable the camera feed display
    void enableDisplay(bool en);

private:
        int midSTN = 0, midMPV = 0, horizontal_bias = 0, vertical_bias = 0; // Calibration data
    bool display_enable = false; // Toggle to show camera feed or not

    map<string, string> data_dictionary; // Stores calibration data from a file
    VideoCapture cap; // OpenCV VideoCapture object to capture frames from the camera

    // ROS2 Publishers to send data to HeightSensor and LinearActuator nodes
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr height_sensor_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr linear_actuator_publisher_;

    // Function to read calibration data from the file
    void readFile();

    // Function to detect and decode QR codes from a camera frame
    int readQR(const Mat &frame, map<string, BarcodeData> &scannedCodes);

    // Function to write updated calibration data (horizontal and vertical bias) to the file
    void writeFile(int B1, int B2);

    // Function to process the camera frame (optional image processing can be added here)
    void processFrame(const Mat &frame);

    // Function to publish processed data (like height or actuator control)
    void publishData(int dstToGoH);
};

QRCodeDetection::QRCodeDetection() : Node("qr_code_detection_node"), cap(0) {
    // Read calibration data from the file
    readFile();
    horizontal_bias = stoi(data_dictionary.at("B1")); // Load horizontal bias
    vertical_bias = stoi(data_dictionary.at("B2"));   // Load vertical bias

    // Set camera properties (resolution and format)
    cap.set(CAP_PROP_FRAME_WIDTH, 4608);
    cap.set(CAP_PROP_FRAME_HEIGHT, 2592);
    cap.set(CAP_PROP_MODE, 1);
    cap.set(CAP_PROP_FORMAT, 0);

    // Check if the camera was successfully opened
    if (!cap.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Error opening camera.");
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Brief delay for camera to initialize
    RCLCPP_INFO(this->get_logger(), "QRCodeDetection node initialized");

    // Initialize publishers for HeightSensor and LinearActuator
    height_sensor_publisher_ = this->create_publisher<std_msgs::msg::Int32>("height_sensor_data", 10);
    linear_actuator_publisher_ = this->create_publisher<std_msgs::msg::Int32>("linear_actuator_data", 10);
}

int main(int argc ,char **argv){
    rclcpp::init(argc, argv);
    rclcpp::shutdown();
    return 0;
}