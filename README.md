# Tree-Sensorization
ROS2 and micro-ROS integration with BLE Transmission of IMU data from Arduino to measure vibratory responses and aid in picking for a robotic apple-picking platform at Oregon State University College of Engineering Intelligent Machines and Materials Lab.

This is part of a summer research fellowship through the AgAID Institute and Oregon State University College of Engineering Intelligent Machines and Materials Lab.

My goal is to improve picking models based on IMU data from an Arduino Nano 33 BLE. This IMU data will be transmitted wirelessly over Bluetooth Low Energy. This is done by publishing this data from BLE peripheral devices (Arduino Nano 33 BLE) and receiving the data on a central device (Arduino Nano RP2040 Connect). This repository hosts central nodes for data collection using a serial monitor and micro-ROS. This project is very much in the works and will continue to be updated as changes and improvements are made. 

The associated nodes posted here are designed to assist those using BLE to focus on data collection, publication, and ROS environment integration while cutting down on distracting features in most relevant examples.

**# Current Status**
Nano_33_BLE_IMU_Peripheral_Node: Stable and Functioning
RP2040_Central_BLE_Node: Stable and Functioning
raw_imu_data_micro_ros_arduino: Stable and Fully Operational for up to 3 Peripheral Devices

**# Setup**
