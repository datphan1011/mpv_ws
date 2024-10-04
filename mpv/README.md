# Multi-purpose vehicle Package

This is the package containing the source code of the MPV and its dependency.
In this package, it will contain 5 ROS2 nodes that have been written in C++ and this is what their uses:
---
* MPVControl.cpp 
    - This node will take user input and send a corresponding message to the /mpv_control topic.
    - This topic will publish the message to the Client node where it subscribes to the /mpv_control topic.
* Client.cpp
    - This Client node will receive the corresponding message from the MPVControl node and ready to send it to the server via srv/cli of ROS2
    - Moreover, this Client node will also subscribe 2 other topics that is /left_height_sensor_mpv and /right_heigh_sensor_mpv
    - Those 2 topics will continuously publish messages about the Height of the MPV to the Client so it can be compared with the station height sensor
* LimitSwitch.cpp
    - This node will continuously publish the data about the state of switching of the MPV
* LockStepper.cpp
    - This node automatically control the state of the lock mechanism by receiving the state of LimitSwitch and sending the data to the /mpv_control
* HeightSensorMPV.cpp
    - This node use VL53L0X flight time control to monitor the height between MPV and the station.
    - It will publish those data onto 2 separate topics that is /left_height_sensor_mpv and /right_heigh_sensor_mpv for the Client node.
