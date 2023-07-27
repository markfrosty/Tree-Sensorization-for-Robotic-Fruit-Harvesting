![OSU_horizontal_2C_O_over_B](https://github.com/markfrosty/Tree-Sensorization/assets/124550575/96075900-d82d-4ea0-b5f4-37118a41f9b5)
![AgAID_ExtendedLogo](https://github.com/markfrosty/Tree-Sensorization/assets/124550575/e2b1272c-e39a-4c16-8cd7-0012b450476d)

# Tree-Sensorization


ROS2 and micro-ROS integration with BLE Transmission of IMU data from Arduino to measure vibratory responses and aid in picking for a robotic apple-picking platform at Oregon State University College of Engineering Intelligent Machines and Materials Lab.

This is part of a summer research internship through the AgAID Institute and Oregon State University College of Engineering Intelligent Machines and Materials Lab.

My goal is to improve picking models based on IMU data from an Arduino Nano 33 BLE. This IMU data will be transmitted wirelessly over Bluetooth Low Energy. This is done by publishing this data from BLE peripheral devices (Arduino Nano 33 BLE) and receiving the data on a central device (Arduino Nano RP2040 Connect). This repository hosts central nodes for data collection using a serial monitor and micro-ROS. This project is very much in the works and will continue to be updated as changes and improvements are made. 

The associated nodes posted here are designed to assist those using BLE to focus on data collection, publication, and ROS environment integration while cutting down on distracting features in most relevant examples.

## Current-Status
Nano_33_BLE_IMU_Peripheral_Node: Stable and Functioning

RP2040_Central_BLE_Node: Stable and Functioning

raw_imu_data_micro_ros_arduino: Stable and Fully Operational for up to 3 Peripheral Devices

## Setup
A computer running Ubuntu 22.04 and a MacBook Pro running macOS Ventura 13.4 were used for development. The only requirements for this are a computer running Ubuntu and installations of the Arduino IDE (with required libraries) and Docker. 

IMPORTANT: Linux equipped computer is required to expose serial ports to the micro-ROS agent running in Docker.

### Arduino IDE
Follow the most up-to-date instructions for the installation on your OS of choice.

Download Link: https://www.arduino.cc/en/software

#### Required Libraries
In order for these boards to function as intended the following libraries are required:

Arduino_LSM9DS1.h -> Install through the libraries tab in the Arduino IDE

ArduinoBLE.h -> Install through the libraries tab in the Arduino IDE

MadgwickAHRS.h -> Install through the libraries tab in the Arduino IDE

micro_ros_arduino.h -> Install by following precompiled library instructions on https://github.com/micro-ROS/micro_ros_arduino

### micro-ROS Arduino Library
Due to default settings and limitations made out of the box, this library requires modification and re-building in order for full functionality to be achieved.

The Arduino Nano RP2040 Connect uses `colcon_verylowmem.meta` which limits publishers:


```
"rmw_microxrcedds": {
    "cmake-args": [
    "-DRMW_UXRCE_MAX_NODES=1",
    "-DRMW_UXRCE_MAX_PUBLISHERS=2",
    "-DRMW_UXRCE_MAX_SUBSCRIPTIONS=1",
    "-DRMW_UXRCE_MAX_SERVICES=0",
    "-DRMW_UXRCE_MAX_CLIENTS=1",
    "-DRMW_UXRCE_MAX_HISTORY=1",
    "-DRMW_UXRCE_TRANSPORT=custom"
    ]
}
```

In this case, we need 12 publishers so this file must be modified to look more like the `colcon.meta` file but with even more publishers. In order to edit the `colcon_verylowmem.meta` file, navigate to where your micro-ROS Arduino library is stored (`micro_ros_arduino`) and go to `extras/library_generation/` directory. Here you will find the `colcon_verylowmem.meta` file and will be able to change the parameters.
After modification, my `colcon_verylowmem.meta` file looks like this:


```
"rmw_microxrcedds": {
            "cmake-args": [
                "-DRMW_UXRCE_MAX_NODES=1",
                "-DRMW_UXRCE_MAX_PUBLISHERS=13",
                "-DRMW_UXRCE_MAX_SUBSCRIPTIONS=5",
                "-DRMW_UXRCE_MAX_SERVICES=1",
                "-DRMW_UXRCE_MAX_CLIENTS=1",
                "-DRMW_UXRCE_MAX_HISTORY=4",
                "-DRMW_UXRCE_TRANSPORT=custom"
            ]
        }
    }
```

After making this change rebuilding of the library must occur and can be achieved by doing the following:

1. If using Windows or macOS launch Docker engine. On Ubuntu you will not have to perform this step.
2. Open a terminal window.
3. Navigate to the micro-ROS Arduino Library folder. My path looked like this: `cd Documents/Arduino/libraries/micro_ros_arduino`
4. Once in the micro-ROS Arduino Library and having made the changes to your `colcon_verylowmem.meta` file run the following commands:
..1. `docker pull microros/micro_ros_static_library_builder:iron`
..2. `docker run -it --rm -v $(pwd):/project --env MICROROS_LIBRARY_FOLDER=extras microros/micro_ros_static_library_builder:iron`
5. This will take a while 
