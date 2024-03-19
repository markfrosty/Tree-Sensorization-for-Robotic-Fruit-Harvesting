//Mark Frost, Oregon State University Intelligent Machines and Materials Lab, Summer 2023
//Part of The Tree Sensorization Data Collection Suite
//Arduino IMU Data Over BLE Subscription Node With micro-ROS Publisher
//Inspired and Derived from micro-ROS Arduino examples and https://github.com/arduino-libraries/ArduinoBLE/issues/185
//Version 4 August 22, 2023
//Central Device: Arduino Nano RP2040 Connect
//Peripheral Device(s): Arduino Nano 33 BLE
//Set Up For 3 Peripherals

//BEGINING OF MICRO-ROS CENTRAL SCRIPT

//Library Intializations
#include <micro_ros_arduino.h>
#include <ArduinoBLE.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/vector3.h>
#include <tree_sensorization_message/msg/tree_sensor.h>

//Definition of Time Intervals and Number of Peripherals
#define BLE_MAX_PERIPHERALS 3
#define BLE_SCAN_INTERVALL 10000
#define BLE_SCAN_new_devices 10000

//BLE Peripheral and Characteristic Establishment
BLEDevice peripherals[BLE_MAX_PERIPHERALS];
BLECharacteristic accelerations[BLE_MAX_PERIPHERALS];
BLECharacteristic gyroscopes[BLE_MAX_PERIPHERALS];
BLECharacteristic magnetometers[BLE_MAX_PERIPHERALS];
BLECharacteristic orientations[BLE_MAX_PERIPHERALS];

//Establishment of Data Variables
float aData[3];
float gData[3];
float mData[3];
float oData[3];

//Logic Variables
bool peripheralsConnected[BLE_MAX_PERIPHERALS] = { 0 };
bool peripheralsToConnect[BLE_MAX_PERIPHERALS] = { 0 };
bool ok = true;
int peripheralCounter = 0;

//Timing Variable
unsigned long control_time;

//Establishment of ROS Parameters 
tree_sensorization_message__msg__TreeSensor imu_msg[BLE_MAX_PERIPHERALS] = { 0 };

rcl_publisher_t imu_publisher[BLE_MAX_PERIPHERALS] = { 0 };
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

//Sets LED Pin
#define LED_PIN 13

//Establishes Checking for Issues and Re-Direction to Error Loop if Present
#define RCCHECK(fn) {rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { error_loop(); }}
#define RCSOFTCHECK(fn) {rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) {}}

//Error Loop
void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(500);
  }
}

void setup() {
  //Initializes BLE Device
  BLE.begin();

  //Sets the name of the central node
  BLE.setDeviceName("IMU Central Node Subscriber");

  //Set the local name the central node advertises
  BLE.setLocalName("IMUCentralService");

  //Start advertising as central node
  BLE.advertise();

  //Begins the Peripheral Counter at 0
  peripheralCounter = 0;

  //Establishes the Transport Type
  set_microros_transports();

  //Establishes micro-ROS Allocator
  allocator = rcl_get_default_allocator();

  //Initializes rcl and Attempts Re-Connection if Necessary
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  //Creates Node for Topics to be Published
  RCCHECK(rclc_node_init_default(&node, "imu_publisher_node", "", &support));

  //Topic Creation
  RCCHECK(rclc_publisher_init_best_effort(
    &imu_publisher[0],
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(tree_sensorization_message, msg, TreeSensor),
    "imu0"));

  RCCHECK(rclc_publisher_init_best_effort(
    &imu_publisher[1],
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(tree_sensorization_message, msg, TreeSensor),
    "imu1"));

  RCCHECK(rclc_publisher_init_best_effort(
    &imu_publisher[2],
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(tree_sensorization_message, msg, TreeSensor),
    "imu2"));
}

void loop() {
  //Scans for BLE Services
  BLE.scanForUuid("19A10010-E8F2-537E-4F6C-D104768A1214");
  BLE.scanForUuid("20A10010-E8F2-537E-4F6C-D104768A1214");
  BLE.scanForUuid("30A10010-E8F2-537E-4F6C-D104768A1214");
  BLE.scanForUuid("40A10010-E8F2-537E-4F6C-D104768A1214");

  unsigned long startMillis = millis();
  while (millis() - startMillis < BLE_SCAN_INTERVALL && peripheralCounter < BLE_MAX_PERIPHERALS) {
    BLEDevice peripheral = BLE.available();
    if (peripheral) {
      //If device has name of interest, is not already in the list to be connected or in the list of connected devices
      if (peripheral.localName() == "IMUPeripheral1" && !peripheralsToConnect[0] && !peripheralsConnected[0]) {
        peripherals[0] = peripheral;
        peripheralCounter++;
        peripheralsToConnect[0] = true;
      }
      if (peripheral.localName() == "IMUPeripheral2" && !peripheralsToConnect[1] && !peripheralsConnected[1]) {
        peripherals[1] = peripheral;
        peripheralCounter++;
        peripheralsToConnect[1] = true;
      }
      if (peripheral.localName() == "IMUPeripheral3" && !peripheralsToConnect[2] && !peripheralsConnected[2]) {
        peripherals[2] = peripheral;
        peripheralCounter++;
        peripheralsToConnect[2] = true;
      }
    }
  }
  BLE.stopScan();

  //Connects to Desired Devices That are NOT Already Connected
  for (int i = 0; i < BLE_MAX_PERIPHERALS; i++) {

    if (peripheralsToConnect[i]) {

      peripherals[i].connect();
      peripherals[i].discoverAttributes();

      //Characteristic Variables are Set
      BLECharacteristic acceleration = peripherals[i].characteristic("19A10011-E8F2-537E-4F6C-D104768A1214");

      BLECharacteristic gyroscope = peripherals[i].characteristic("20A10011-E8F2-537E-4F6C-D104768A1214");

      BLECharacteristic magnetometer = peripherals[i].characteristic("30A10011-E8F2-537E-4F6C-D104768A1214");

      BLECharacteristic orientation = peripherals[i].characteristic("40A10011-E8F2-537E-4F6C-D104768A1214");

      //Subscriptions to Characteristics are Managed
      if (acceleration) {
        accelerations[i] = acceleration;
        accelerations[i].subscribe();
      }

      if (gyroscope) {
        gyroscopes[i] = gyroscope;
        gyroscopes[i].subscribe();
      }

      if (magnetometer) {
        magnetometers[i] = magnetometer;
        magnetometers[i].subscribe();
      }

      if (orientation){
        orientations[i] = orientation;
        orientations[i].subscribe();
      }
    }

    peripheralsConnected[i] = true;
    peripheralsToConnect[i] = false;
  }


  control_time = millis();
  ok = true;
  while (ok) {
    if (peripheralCounter < BLE_MAX_PERIPHERALS) {
      //if not all devices connected, redo search after BLE_SCAN_new_devices time
      if (millis() - control_time > BLE_SCAN_new_devices) {
        ok = false;
      }
    }
    for (int i = 0; i < BLE_MAX_PERIPHERALS; i++) {
      //If the i_th device is supposed to be connected
      if (peripheralsConnected[i]) {
        //Check if it disconnected in the meantime
        if (!peripherals[i].connected()) {
          ok = false;
          peripheralsConnected[i] = false;
          peripheralCounter--;
        } else {
          //If Peripherals are Still Connected and Values are Updated, Values Read and Published
          if (accelerations[i].valueUpdated() || gyroscopes[i].valueUpdated() || magnetometers[i].valueUpdated() || orientations[i].valueUpdated()) {
            accelerations[i].readValue(aData, 12);
            gyroscopes[i].readValue(gData, 12);
            magnetometers[i].readValue(mData, 12);
            orientations[i].readValue(oData, 12);

            imu_msg[i].linear_acceleration.x = aData[0];
            imu_msg[i].linear_acceleration.y = aData[1];
            imu_msg[i].linear_acceleration.z = aData[2];

            imu_msg[i].angular_velocity.x = gData[0];
            imu_msg[i].angular_velocity.y = gData[1];
            imu_msg[i].angular_velocity.z = gData[2];

            imu_msg[i].magnetometer_data.x = mData[0];
            imu_msg[i].magnetometer_data.y = mData[1];
            imu_msg[i].magnetometer_data.z = mData[2];

            imu_msg[i].orientation.x = oData[0];
            imu_msg[i].orientation.y = oData[1];
            imu_msg[i].orientation.z = oData[2];
            imu_msg[i].orientation.w = 1.0;

            if (i == 0) {
              RCSOFTCHECK(rcl_publish(&imu_publisher[0], &imu_msg[0], NULL));
            } else if (i == 1) {
              RCSOFTCHECK(rcl_publish(&imu_publisher[1], &imu_msg[1], NULL));
            } else {
              RCSOFTCHECK(rcl_publish(&imu_publisher[2], &imu_msg[2], NULL));
            }
          }
        }
      }
    }
  }
}
