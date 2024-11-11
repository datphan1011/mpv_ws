cmake_minimum_required(VERSION 3.8)
project(mpv)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(my_custom_service REQUIRED)

# find library
find_library(PIGPIO_LIB pigpio REQUIRED PATHS /usr/local/lib)
find_library(PIGPIOD_IF2_LIB pigpiod_if2 REQUIRED PATHS /usr/local/lib)

# Specify the source files for the node
add_executable(mpv_control_node src/MPVControl.cpp)
add_executable(client_node src/Client.cpp)
add_executable(height_sensor_mpv_node src/HeightSensorMPV.cpp)
add_executable(lock_stepper_node src/LockStepper.cpp)
add_executable(limit_switch_node src/LimitSwitch.cpp)

# Link ROS2 libraries to the executable
ament_target_dependencies(mpv_control_node rclcpp std_msgs)
ament_target_dependencies(client_node rclcpp std_msgs std_srvs my_custom_service)
ament_target_dependencies(height_sensor_mpv_node rclcpp std_msgs)
ament_target_dependencies(lock_stepper_node rclcpp std_msgs)
ament_target_dependencies(limit_switch_node rclcpp std_msgs)

# Link library to the designated file
target_link_libraries(height_sensor_mpv_node ${PIGPIO_LIB} ${PIGPIOD_IF2_LIB})
target_link_libraries(mpv_control_node ${PIGPIO_LIB})
target_link_libraries(limit_switch_node ${PIGPIO_LIB})
target_link_libraries(lock_stepper_node ${PIGPIO_LIB})

# Install rule
INSTALL(TARGETS
  mpv_control_node
  client_node
  height_sensor_mpv_node
  lock_stepper_node
  limit_switch_node
DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)
ament_package()
