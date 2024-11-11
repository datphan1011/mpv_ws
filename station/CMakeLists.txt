cmake_minimum_required(VERSION 3.8)
project(station)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(my_custom_service REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
# Include the directory where your custom library is located
include_directories(
  # ${CMAKE_SOURCE_DIR}/include/Adafruit-PWM-Servo-Driver-Library-master
  # ${CMAKE_SOURCE_DIR}/include/Adafruit_VL53L0X-master
  # ${CMAKE_SOURCE_DIR}/include/Adafruit_BusIO-master
  # ${CMAKE_SOURCE_DIR}/include/ArduinoCore-avr-master
  # ${CMAKE_SOURCE_DIR}/include/pigpio
)
# Find and link Pigpio library
find_library(PIGPIO_LIB pigpio REQUIRED PATHS /usr/local/lib)
find_library(PIGPIOD_IF2_LIB pigpiod_if2 REQUIRED PATHS /usr/local/lib)
# Specify the source files for the node
add_executable(height_sensor_node src/HeightSensorStation.cpp)
add_executable(server_node src/Server.cpp)
add_executable(station_control_node src/StationControl.cpp)
add_executable(gripper_node src/Gripper.cpp)
add_executable(limitswitch_station_node src/LimitSwitch.cpp)
add_executable(linear_actuator_node src/LinearActuator.cpp)
add_executable(lead_screw_node src/LeadScrew.cpp)
add_executable(qr_code_node src/QRCodeDetection.cpp)

# Link ROS2 libraries to the executable
ament_target_dependencies(height_sensor_node rclcpp std_msgs std_srvs)
ament_target_dependencies(server_node rclcpp std_msgs std_srvs my_custom_service)
ament_target_dependencies(station_control_node std_msgs rclcpp)
ament_target_dependencies(gripper_node rclcpp std_msgs)
ament_target_dependencies(limitswitch_station_node rclcpp std_msgs)
ament_target_dependencies(linear_actuator_node rclcpp std_msgs)
ament_target_dependencies(lead_screw_node rclcpp std_msgs)
ament_target_dependencies(qr_code_node rclcpp std_msgs rclcpp std_msgs sensor_msgs cv_bridge image_transport OpenCV)

# Link Pigpio library to the executable
target_link_libraries(height_sensor_node ${PIGPIO_LIB})
target_link_libraries(gripper_node ${PIGPIO_LIB})
target_link_libraries(limitswitch_station_node ${PIGPIO_LIB})
target_link_libraries(linear_actuator_node ${PIGPIO_LIB})
target_link_libraries(lead_screw_node ${PIGPIO_LIB})

# Install rules
install(TARGETS
  height_sensor_node
  server_node
  station_control_node
  gripper_node
  limitswitch_station_node
  linear_actuator_node
  lead_screw_node
  qr_code_node
DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)
ament_package()
