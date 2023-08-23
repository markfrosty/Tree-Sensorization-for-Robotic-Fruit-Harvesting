![OSU_horizontal_2C_O_over_B](https://github.com/markfrosty/Tree-Sensorization/assets/124550575/96075900-d82d-4ea0-b5f4-37118a41f9b5)
![AgAID_ExtendedLogo](https://github.com/markfrosty/Tree-Sensorization/assets/124550575/e2b1272c-e39a-4c16-8cd7-0012b450476d)

## Advisor: Dr. Joe Davidson

## Mentor: Miranda Cravetz

## Intern, Engineer, and Developer: Mark Frost

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

### Installing Docker for Ubuntu
1. Update the `apt` package index and install packages to allow `apt` to use a repository over HTTPS:

```
sudo apt-get update
sudo apt-get install ca-certificates curl gnupg
```

2. Add Docker’s official GPG key:

```
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg
```

3. Use the following command to set up the repository:

```
echo \
  "deb [arch="$(dpkg --print-architecture)" signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  "$(. /etc/os-release && echo "$VERSION_CODENAME")" stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
```

4. Update the apt package index:

```
sudo apt-get update
```

5. Install Docker Engine, containerd, and Docker Compose.

```
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

6. Verify that the Docker Engine installation is successful by running the `hello-world` image.

```
sudo docker run hello-world
```

7. Create the `docker` group.

```
sudo groupadd docker
```

8. Add your user to the `docker` group.

```
sudo usermod -aG docker $USER
```

9. Run the following command to activate the changes to groups:

```
newgrp docker
```

10. Verify that you can run `docker` commands without `sudo`.

```
docker run hello-world
```


### Installing ROS2 in Docker
This should realistically be performed on your machine running Ubuntu as you will not be able to access the serial ports on anything but a Linux machine as previously discussed.

1. Open a terminal window and run `docker pull ros:iron`
2. After it completes the pull and creates the image run `docker run -it ros:iron` and ensure it is in the ros2 environment.
3. Run the line `ros2 topic list` and you should see two topics: `/parameter_events` and `/rosout`

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

## How To Use
In order to use these scripts and 3 peripherals in a micro-ROS/ROS2 environment follow the steps listed below:

1. Upload the `raw_imu_data_micro_ros_arduino.ino` file from the Arduino IDE to the Arduino Nano RP2040 Connect and unplug it when the upload has finished.
2. Open the `Nano_33_BLE_IMU_Peripheral_Node.ino` file in the Arduino IDE and ensure that the `BLE.setLocalName("IMUPeripheral1");` is set to 1 and upload to your first Arduino Nano 33 BLE. This step should be repeated with the modification, `BLE.setLocalName("IMUPeripheral2");` and `BLE.setLocalName("IMUPeripheral3");` for each respective Arduino Nano 33 BLE being used. Unplug them and leave them powered off after programs are uploaded.
3. Once all programs are uploaded to the boards you will open two terminal windows on your computer running Ubuntu
4. In one terminal window run the following line `docker run -it ros:iron`.
5. In the other run the micro-ROS agent using this docker command:

```
docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:iron serial --dev /dev/ttyACM0 -v6
```

6. Plug in all 3 Arduino Nano 33 BLE (peripheral devices) to USB ports or provide them power in whatever way you see fit subsequently plug in the Arduino Nano RP2040 Connect (central device) and wait for large amounts of scrolling change to appear as a connection is established between all peripherals and the central and messages are being published.
7. After you see that all are connected, return to your ROS2 terminal window. Run the comand `ros2 topic list`. Here you should see 14 topics. It should look like this:

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

8. You should now be able to view any of the data being published from those topics by using `ros2 topic echo` and then the respective topic. For example if I wanted accelerometer data from Board 2 I would use `ros2 topic echo /accel1` and I should start seeing the data from that board.


## Known Issues
There will be issues and this repo will be updated as progress is made and things are ironed out.

One known issue is sometimes the connection is dropped on a board especially if powered by something like a USB battery bank. Most of the time waiting and ensuring power is being given to the boards will work otherwise hitting the central board reset button first followed by the peripheral reset buttons usually fixes things.

When first plugging in boards sometimes connection doesn't happen and this can be solved by hitting the central board reset button first followed by the peripheral reset buttons. This works 99% of the time in my experience. 

Please post any issues relating to my script and not the libraries used to the issues section and I will be sure to try and fix them.


## Thank You
Thank you for using my project to help solve any multi-device Arduino BLE issues you have or micro-ROS sensor data collection issues as well. I know it isn't perfect but I was unable to find anything on the internet that combined BLE communication, Arduinos, and micro-ROS so I hope it helps someone that is attempting something similar.
