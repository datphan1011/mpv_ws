cmake_minimum_required(VERSION 3.8)
project(my_custom_service)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate the .srv files
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MPVAndStation.srv"
)

ament_package()
