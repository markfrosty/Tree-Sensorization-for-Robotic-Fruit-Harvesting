//Mark Frost, Oregon State University IMML
//Based off and adapted from https://github.com/ashok-kummar/microros_imu_publisher/blob/main/imu_publisher.ino

//Designed for Arduino Nano RP2040 Connect and LSM6SOX IMU
//Collects raw IMU data in order to construct models

//Micro-ROS library
#include <micro_ros_arduino.h>

//included for future planned expansion
#include <WiFiNINA.h>
#include <ArduinoBLE.h>

//Sensor librarys to cover bases
#include <Arduino_LSM6DSOX.h>
#include <LSM6DSOXSensor.h>
#include <Wire.h>

//ROS client librarys for publication to topic
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/imu.h>

//Publication establishment
sensor_msgs__msg__Imu imu_msg;
rcl_publisher_t publisher;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;


#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &imu_msg, NULL));

    //IMU data is ingested and passed through to ROS message 
    float Ax, Ay, Az;
    IMU.readAcceleration(Ax, Ay, Az);
    imu_msg.linear_acceleration.x = Ax;
    imu_msg.linear_acceleration.y = Ay;
    imu_msg.linear_acceleration.z = Az;

    float Gx, Gy, Gz;
    IMU.readGyroscope(Gx, Gy, Gz);
    imu_msg.angular_velocity.x = Gx;
    imu_msg.angular_velocity.y = Gy;
    imu_msg.angular_velocity.z = Gz;
  }
}

void setup() {
  set_microros_transports();

  //Lets get this show on the road
  if (!IMU.begin()) {
    Serial.println("Failure");
    while (1) {
      delay(10);
    }
  }
  Serial.println("Success");

  allocator = rcl_get_default_allocator();

  //Initialization
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  //Establishment of publication node
  RCCHECK(rclc_node_init_default(&node, "imu_publisher_node", "", &support));

  //Publisher creation and topic establishment
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu_data_topic"));

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


//BEWARE: SERIAL PRINT MAY NOT WORK. TREAT THIS WITH CAUTION WHEN USING MICRO-ROS
//agent may not recieve all the data. encouraged to publish this data along with other IMU.data
  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println("Hz");
  Serial.println();

  Serial.print("Gyroscope sample rate = ");  
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println("Hz");
  Serial.println();
}

void loop() {
  delay(100);
  
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
