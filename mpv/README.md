# Multi-purpose vehicle Package

This is the package contain the source code of the MPV and it dependency.
In this packages it will contain 5 ROS2 nodes that have been written in C++ and this is what their uses:
---
* MPVControl.cpp 
    - This node will take user input and send corresponding message to the /mpv_control topic.
    - This topic will publish the message to Client node where it subscribe to the /mpv_control topic.
* Client.cpp
    - This Client node will received the correspond message from the MPVControl node and ready to send it to the server via srv/cli of ROS2
    - Moreover, this Client node will also subscribe 2 other topics that is /left_height_sensor_mpv and /right_heigh_sensor_mpv
    - Those 2 topics will continuously publish message about the Height of the MPV to the Client so it can compares with the station height sensor
* LimitSwitch.cpp
    - This node will continuously publish the data about the state of switching of the MPV
* LockStepper.cpp
    - This node automatically control the state of the lock mechanism by receiving the state of LimitSwitch and send the data to the /mpv_control
* HeightSensorMPV.cpp
    - This node use VL53L0X flight time control to monitoring the height between MPV and the station.
    - It will publish those data onto 2 seperate topics that is /left_height_sensor_mpv and /right_heigh_sensor_mpv for the Client node.
