//Mark Frost, Oregon State University Intelligent Machines and Materials Lab, Summer 2023
//Part of The Tree Sensorization Data Collection Suite
//Arduino IMU Data Over BLE Subscription Node With micro-ROS Publisher
//Version 2 July 20, 2023
//Central Device: Arduino Nano RP2040 Connect
//Peripheral Device: Arduino Nano 33 BLE
//IMU: LSM9DS1
//    DataSheet: https://www.st.com/resource/en/datasheet/lsm9ds1.pdf
//    Standard Specification Cheat Sheet:
//      Accelerometer range is set at ±4 g with a resolution of 0.122 mg. -> converted to m/s^2 on peripheral
//      Gyroscope range is set at ±2000 dps with a resolution of 70 mdps.
//      Magnetometer range is set at ±400 uT with a resolution of 0.014 uT.
//      Accelerometer and gyrospcope output data rate is fixed at 119 Hz.
//      Magnetometer output data rate is fixed at 20 Hz.

//BEGINING OF MICRO-ROS CENTRAL SCRIPT

//Micro-ROS library
#include <micro_ros_arduino.h>

//Bluetooth Low Energy Library for Wireless Communication
#include <ArduinoBLE.h>

//IMU Library (unncessary but included)
#include <Arduino_LSM9DS1.h>

//ROS client librarys for publication to topic
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/vector3.h>

//Publication variables establishment
geometry_msgs__msg__Vector3 a_msg;
geometry_msgs__msg__Vector3 g_msg;
geometry_msgs__msg__Vector3 m_msg;
geometry_msgs__msg__Vector3 q_msg;
rcl_publisher_t accel_publisher;
rcl_publisher_t gyro_publisher;
rcl_publisher_t mag_publisher;
rcl_publisher_t quat_publisher;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

//Establishes flag for connection and continuous data collection
bool connected = false;

#define LED_PIN 13

//Checks executor and redirects to error loop if there's an issue
#define RCCHECK(fn) {rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { error_loop(); }}
#define RCSOFTCHECK(fn) {rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) {}}

//error loop
void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(10);
  }
}

void timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
}

void setup() {
  //Enables IMU and BLE systems 
  IMU.begin();
  BLE.begin();

  //Sets the name of the central node
  BLE.setDeviceName("IMU Central Node Subscriber");

  //Set the local name the central node advertises
  BLE.setLocalName("IMUCentralService");

  //Start advertising as central node
  BLE.advertise();

  BLE.scanForUuid("19B10010-E8F2-537E-4F6C-D104768A1214");  //scan for Accel Service
  BLE.scanForUuid("20B10010-E8F2-537E-4F6C-D104768A1214");  //scan for Gyro Service
  BLE.scanForUuid("30B10010-E8F2-537E-4F6C-D104768A1214");  //scan for Mag service
  BLE.scanForUuid("40B10010-E8F2-537E-4F6C-D104768A1214");  //scan for Quaternion service

  set_microros_transports();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  delay(100);

  allocator = rcl_get_default_allocator();

  //Initialization
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  //Establishment of publication node
  RCCHECK(rclc_node_init_default(&node, "imu_publisher_node", "", &support));

  //Publisher creation and ROS topic establishment 
  RCCHECK(rclc_publisher_init_default(
    &accel_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
    "accel_data_topic"));

  RCCHECK(rclc_publisher_init_default(
    &gyro_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
    "gyro_data_topic"));

  RCCHECK(rclc_publisher_init_default(
    &mag_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
    "mag_data_topic"));

  RCCHECK(rclc_publisher_init_default(
    &quat_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
    "quat_data_topic"));

  //Timer Creation
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  //Creates executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
}

void loop() {
  delay(100);

  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

  BLEDevice peripheral = BLE.available();
  if (peripheral) {

    //If the name doesn't have "IMUPeripheralService" it is ignored
    if (peripheral.localName().indexOf("IMUPeripheralService") < 0) {
      return;
    }
    BLE.stopScan();

    //Reads info from peripheral device(s)
    readPeripheral(peripheral);

    BLE.scanForUuid("19B10010-E8F2-537E-4F6C-D104768A1214");  //scan for Accel Service
    BLE.scanForUuid("20B10010-E8F2-537E-4F6C-D104768A1214");  //scan for Gyro Service
    BLE.scanForUuid("30B10010-E8F2-537E-4F6C-D104768A1214");  //scan for Mag service
    BLE.scanForUuid("40B10010-E8F2-537E-4F6C-D104768A1214");  //scan for Quaternion service
  }
}

void readPeripheral(BLEDevice peripheral) {
  if (peripheral.connect()) {
    connected = true;
  } else {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(500);
  }

  if (!peripheral.discoverAttributes()) {
    peripheral.disconnect();
    return;
  }

  //If it finds all the desired IMU services it begins reading them
  if (peripheral.hasService("19B10010-E8F2-537E-4F6C-D104768A1214") && peripheral.hasService("20B10010-E8F2-537E-4F6C-D104768A1214") && peripheral.hasService("30B10010-E8F2-537E-4F6C-D104768A1214") && peripheral.hasService("40B10010-E8F2-537E-4F6C-D104768A1214")) {

    BLECharacteristic acceleration = peripheral.characteristic("19B10011-E8F2-537E-4F6C-D104768A1214");

    BLECharacteristic gyroscope = peripheral.characteristic("20B10011-E8F2-537E-4F6C-D104768A1214");

    BLECharacteristic magnetometer = peripheral.characteristic("30B10011-E8F2-537E-4F6C-D104768A1214");

    BLECharacteristic quaternion = peripheral.characteristic("40B10011-E8F2-537E-4F6C-D104768A1214");

    while (connected) {
      if (acceleration.canRead() && gyroscope.canRead() && magnetometer.canRead() && quaternion.canRead()) {
        //ROS messages are published to topics here
        RCSOFTCHECK(rcl_publish(&accel_publisher, &a_msg, NULL) && rcl_publish(&gyro_publisher, &g_msg, NULL) && rcl_publish(&mag_publisher, &m_msg, NULL) && rcl_publish(&quat_publisher, &q_msg, NULL));

        //Data is read
        float aData[3];
        acceleration.readValue(aData, 12);

        float gData[3];
        gyroscope.readValue(gData, 12);

        float mData[3];
        magnetometer.readValue(mData, 12);

        float qData[3];
        quaternion.readValue(qData, 12);

        //IMU data is ingested and passed through to ROS message
        a_msg.x = aData[0];
        a_msg.y = aData[1];
        a_msg.z = aData[2];

        g_msg.x = gData[0];
        g_msg.y = gData[1];
        g_msg.z = gData[2];

        m_msg.x = mData[0];
        m_msg.y = mData[1];
        m_msg.z = mData[2];

        q_msg.x = qData[0];
        q_msg.y = qData[1];
        q_msg.z = qData[2];

      }
    }
  }
  peripheral.disconnect();
  connected = false;
}
