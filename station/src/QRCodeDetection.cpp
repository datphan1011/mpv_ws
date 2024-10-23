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
// Destructor
QRCodeDetection::~QRCodeDetection() {
    cap.release(); // Release the camera resource when the node shuts down
}

// Main loop for the measurement process, continuously captures frames and detects QR codes
void QRCodeDetection::measure() {
    bool done = false;
    int falseCnt = 0; // Counter for how many frames failed to detect the required QR codes
    Mat frame; // Variable to store the captured frame

    while (!done) {
        map<string, BarcodeData> decodedCodes; // Holds the detected QR codes
        cap.read(frame); // Capture a frame from the camera
        if (frame.empty()) {
            RCLCPP_ERROR(this->get_logger(), "ERROR capturing frame.");
            continue;
        }

        processFrame(frame); // Optional image processing (can display the frame)
        if (readQR(frame, decodedCodes) == 4) { // Detect QR codes in the frame
            // Check if 4 specific QR codes are present (MPV_L, MPV_R, STN_L, STN_R)
            if (decodedCodes.find("MPV_L") != decodedCodes.end() && 
                decodedCodes.find("MPV_R") != decodedCodes.end() && 
                decodedCodes.find("STN_L") != decodedCodes.end() && 
                decodedCodes.find("STN_R") != decodedCodes.end()) {
                
                // Successfully detected all required QR codes, process further
                done = true;
            }
        }
        falseCnt++; // Increment the failure counter
        if (falseCnt > 100) {
            RCLCPP_ERROR(this->get_logger(), "4 QR codes not found.");
            break;
        }
    }
}
// Detect and decode QR codes in a camera frame, storing the results in a map
int QRCodeDetection::readQR(const Mat &frame, map<string, BarcodeData> &scannedCodes) {
    QRCodeDetector qrDetector;
    vector<Point> points;
    string qrData = qrDetector.detectAndDecode(frame, points); // Detect and decode the QR code

    if (!qrData.empty()) { // If QR data is successfully decoded
        BarcodeData bcData;
        bcData.data = qrData; // Store the decoded data
        scannedCodes[bcData.data] = bcData; // Add the QR code data to the map
        RCLCPP_INFO(this->get_logger(), "Read QR: found: %s at: (%d, %d)", bcData.data.c_str(), bcData.midPoint.x, bcData.midPoint.y);
    }
    return scannedCodes.size(); // Return the number of QR codes detected
}

// Calibration function to detect QR codes and adjust the horizontal/vertical bias
void QRCodeDetection::calibrate() {
    RCLCPP_INFO(this->get_logger(), "Starting calibration");
    bool done = false;
    int false_cnt = 0; // Counter for calibration failures
    Mat frame;

    while (!done) {
        cap.read(frame); // Capture a frame
        if (frame.empty()) {
            RCLCPP_ERROR(this->get_logger(), "ERROR capturing frame.");
            continue;
        }

        map<string, BarcodeData> decoded_codes;
        if (readQR(frame, decoded_codes) == 4) { // Check if 4 QR codes are detected
            // Calculate midpoints for MPV and STN (station) QR codes
            Point mid_point_mpv = Point(((decoded_codes.at("MPV_L").midPoint.x + decoded_codes.at("MPV_R").midPoint.x) / 2),
                                        ((decoded_codes.at("MPV_L").midPoint.y + decoded_codes.at("MPV_R").midPoint.y) / 2));
            Point mid_point_stn = Point(((decoded_codes.at("STN_L").midPoint.x + decoded_codes.at("STN_R").midPoint.x) / 2),
                                        ((decoded_codes.at("STN_L").midPoint.y + decoded_codes.at("STN_R").midPoint.y) / 2));
            horizontal_bias = mid_point_stn.x - mid_point_mpv.x; // Calculate horizontal bias
            vertical_bias = mid_point_stn.y - mid_point_mpv.y;   // Calculate vertical bias
            writeFile(horizontal_bias, vertical_bias); // Save biases to file
            done = true;
        }
        false_cnt++; // Increment the failure counter
        if (false_cnt > 100) {
            RCLCPP_ERROR(this->get_logger(), "4 QR codes not found.");
            break;
        }
    }
}

// Enable or disable the display of camera frames
void QRCodeDetection::enableDisplay(bool en) {
    display_enable = en;
}

// Publish the calculated data (height sensor, linear actuator) based on the detected QR codes
void QRCodeDetection::publishData(int dstToGoH) {
    auto height_sensor_msg = std_msgs::msg::Int32();
    auto linear_actuator_msg = std_msgs::msg::Int32();

    // Assign calculated data to messages and publish them
    height_sensor_msg.data = dstToGoH;
    height_sensor_publisher_->publish(height_sensor_msg);
    linear_actuator_msg.data = dst_to_go_v;
    linear_actuator_publisher_->publish(linear_actuator_msg);
}

// Main function to launch the QRCodeDetection node and either calibrate or measure
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto qrCodeNode = std::make_shared<QRCodeDetection>(); // Create QRCodeDetection node
    char choice;
    
    // Prompt user for choice between calibration or measurement
    std::cout << "Enter (C) to Calibrate or (M) to Measure: ";
    std::cin >> choice;
    if (choice == 'C' || choice == 'c') {
        qrCodeNode->calibrate(); // Start calibration
    } else if (choice == 'M' || choice == 'm') {
        qrCodeNode->measure(); // Start measurement
    }

    rclcpp::shutdown(); // Shut down ROS2
    return 0;
}