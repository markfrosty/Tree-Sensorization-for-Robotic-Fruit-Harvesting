## Developed by: Mark Frost

# Tree-Sensorization: testing branch

This branch and it's contents set out to achieve the goal of further simplifying and improving the over all Tree-Sensorization project. This branch currently includes support for adding a custom message in order to simplify and improve transmission of data. Further simplification of data transmission is targeted to improve speed, timing, and enable easier visualization of data in apps such as RViz and Plot Juggler.

#### DISCLAIMER: This development was part of a project for the ROB599 class at Oregon State University.

## Current-Status
### Main Branch

All aspects are to be treated as unstable.

There are many issues.

Messages: 
1. There has been an issue where the subscriber will recieve data and the topics will show up when `ros2 topic list` is used but `ros2 topic echo /imu0` for example results in an error message: `The message type 'tree_sensorization_messages/msg/TreeSensor' is invalid`. I don't really understand where this is coming from as I do recieve the expected data in the subscriber.
2. The messages coming into the subscriber are extremely slow and lag inconsistently. When I used to have 12 topics and would echo them, there was a firehose of messages. Now this subscriber doesn't perform the same way. Clearly there is significant room for improvement.

## Setup
A computer running Ubuntu 22.04 and a MacBook Pro running macOS Ventura 13.4 were used for development. The only requirements for this are a computer running Ubuntu and installations of the Arduino IDE (with required libraries) and Docker. 

IMPORTANT: Linux equipped computer is required to expose serial ports to the micro-ROS agent running in Docker.
NOTE: The following information assumes you have gone through the required steps on the main branch [README](https://github.com/markfrosty/Tree-Sensorization-for-Robotic-Fruit-Harvesting/blob/main/README.md) before attempting these unstable implementations.

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

    1.`docker pull microros/micro_ros_static_library_builder:iron`
   
    2.`docker run -it --rm -v $(pwd):/project --env MICROROS_LIBRARY_FOLDER=extras microros/micro_ros_static_library_builder:iron`
5. This will take 10+ minutes to complete before you can recompile and upload to the boards with this change implemented.

## How To Use Locally
In order to use these scripts and 3 peripherals in a micro-ROS/ROS 2 environment follow the steps listed below:

1. Upload the `raw_imu_data_micro_ros_arduino.ino` file from the Arduino IDE to the Arduino Nano RP2040 Connect and unplug it when the upload has finished.
2. Open the `Nano_33_BLE_IMU_Peripheral_Node.ino` file in the Arduino IDE and ensure that the `BLE.setLocalName("IMUPeripheral1");` is set to 1 and upload to your first Arduino Nano 33 BLE. This step should be repeated with the modification, `BLE.setLocalName("IMUPeripheral2");` and `BLE.setLocalName("IMUPeripheral3");` for each respective Arduino Nano 33 BLE being used. Unplug them and leave them powered off after programs are uploaded.
3. Once all programs are uploaded to the boards you will open two terminal windows on your computer running Ubuntu
4. In one terminal run the micro-ROS agent with these following lines:

```
source /opt/ros/iron/setup.bash
cd microros_ws
source install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
```

5. In the other terminal window run the following: `source /opt/ros/iron/setup.bash`.
6. Plug in all 3 Arduino Nano 33 BLE (peripheral devices) to USB ports or provide them power in whatever way you see fit (if building my sensor array turn peripherals on). Subsequently, plug in the Arduino Nano RP2040 Connect (central device) and wait for large amounts of scrolling change to appear as a connection is established between all peripherals and the central and messages are being published. I have found that connection occurs quicker if the peripherals are powered first before the central is plugged in.
7. After you see that all are connected, return to your ROS 2 terminal window not running the micro-ROS Agent. Run the comand `ros2 topic list`. Here you should see 14 topics. It should look like this:

```
/accel0
/accel1
/accel2
/gyro0
/gyro1
/gyro2
/mag0
/mag1
/mag2
/orient0
/orient1
/orient2
/parameter_events
/rosout
```

8. You should now be able to view any of the data being published from those topics by using `ros2 topic echo` and then the respective topic. For example, if I wanted accelerometer data from Board 2 I would use `ros2 topic echo /accel1` and I should start seeing the data from that board.
9. You can also bag record data using `ros2 bag record --storage sqlite3 -a -o INSERT YOUR DESIRED FILE NAME HERE`.


## Known Issues
There will be issues and this repo will be updated as progress is made and things are ironed out.

One known issue is sometimes the connection is dropped on a board especially if powered by something like a USB battery bank. Most of the time waiting and ensuring power is being given to the boards will work otherwise hitting the central board reset button first followed by the peripheral reset buttons usually fixes things.

When first plugging in boards sometimes connection doesn't happen and this can be solved by hitting the central board reset button first followed by the peripheral reset buttons. This works 99% of the time in my experience. 

Please post any issues relating to my script and not the libraries used to the issues section and I will be sure to try and fix them.


## Thank You
Thank you for using my project to help solve any multi-device Arduino BLE issues you have or micro-ROS sensor data collection issues as well. I know it isn't perfect but I was unable to find anything on the internet that combined BLE communication, Arduinos, and micro-ROS so I hope it helps someone that is attempting something similar.
