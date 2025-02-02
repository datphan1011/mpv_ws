//--------------------------------------------
//Standard library
#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <vector>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
//--------------------------------------------
//Include custom library
#include "mpv/CustomHeader/UserInterface.hpp"
//--------------------------------------------

class MPVControl : public rclcpp::Node{
    public:
        MPVControl() : Node("mpv_control_node"){ // Constructor initialize a node name MPV Control
            //Publish the user input into the /MPV_Control topic, limited by 10
            main_control_publisher_ = this->create_publisher<std_msgs::msg::String>(
            "mpv_control", 10); 
            // Timer to simulate publishing data periodically
            timer_publisher_ = this->create_wall_timer(std::chrono::seconds(1), 
            std::bind(&MPVControl::publishing_data, this));
        }
        ~MPVControl(){
            std::cout << "Thank you for using the MPV\n";
        }
    private:
        // Class members variable
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr main_control_publisher_;    //an object publisher that wrapped in the smart pointer
        rclcpp::TimerBase::SharedPtr timer_publisher_;  //an object timer 
        // This function get the user input
        char get_user_input(){
            char user_input;
            std::cout << "Enter which task to run: \n";
            std::cin >> user_input;
            return user_input;
        }
        // This function get the user input and publish the data into the client
        void publishing_data(){
            user_menu_interface_options(this->get_logger());
            char user_input = get_user_input();
            switch(user_input){
                case 'u':
                    publish_message("UNLOAD");   // this function publish UNLOAD message to the screen
                    RCLCPP_INFO(this->get_logger(), "Station unloading");
                    break;
                case 'l':
                    publish_message("LOAD");     // this function publish LOAD message to the screen
                    RCLCPP_INFO(this->get_logger(), "Station loading");
                    break;
                case 'p':
                    publish_message("INITIAL_POSE"); // this function publish INITIAL POSITION message to the screen
                    RCLCPP_INFO(this->get_logger(), "Exchange module is going to initial pose");
                    break;
                case 'o':
                    publish_message("LOCKOUT");  // this function publish LOCKOUT message to the screen
                    RCLCPP_INFO(this->get_logger(), "Moving lockpins out");
                    break;
                case 'i':
                    publish_message("LOCKIN");   // this function publish LOCKIN message to the screen
                    RCLCPP_INFO(this->get_logger(), "Moving lockpins in");
                    break;
                case 'h':
                    publish_message("HEIGHT");   // this function publish HEIGHT message to the screen
                    RCLCPP_INFO(this->get_logger(), "Adjusting height of the exchange station");
                    break;
                case 'q':
                    publish_message("QUIT"); // this function publish QUIT message to the screen
                    RCLCPP_INFO(this->get_logger(), "Quitting the program");
                    rclcpp::shutdown(); // Shutdown the application
                    exit(0);
                default:
                    RCLCPP_WARN(this->get_logger(), "Unknown task selected."); // other inputs that difference from switch will be sent here.
                    break;
                }
                std::this_thread::sleep_for(std::chrono::seconds(2));  // Sleep before next input
            }
        // This function will be call when the MPV Control to publishing the data into the /mpv_control topic
        void publish_message(const std::string &msg_content) {
            auto msg = std_msgs::msg::String();
            msg.data = msg_content;
            main_control_publisher_->publish(msg);
        }
};
int main(int argc, char **argv){
    rclcpp::init(argc,argv);    //initialize the node
    auto node = std::make_shared<MPVControl>(); // add a variable to the node
    rclcpp::spin(node); //Spin the node
    rclcpp::shutdown(); //shutdown
    return 0;
}
