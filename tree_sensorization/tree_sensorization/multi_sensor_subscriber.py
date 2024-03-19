#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from tree_sensorization_message.msg import TreeSensor

class MultiSensorSubscriber(Node):
    def __init__(self):
        super().__init__('multi_sensor_subscriber')
        
        qos_profile = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST)
        
        # Create a subscription for each IMU topic
        self.imu0 = self.create_subscription(TreeSensor, 'imu0', self.imu_callback0, qos_profile)
        self.imu1 = self.create_subscription(TreeSensor, 'imu1', self.imu_callback1, qos_profile)
        self.imu2 = self.create_subscription(TreeSensor, 'imu2', self.imu_callback2, qos_profile)


    def imu_callback0(self, msg):
        #self.get_logger().info(f'IMU data from sensor 0: {msg}')
        print(f'Sensor 0 - Linear Acc: {msg.linear_acceleration.x}, {msg.linear_acceleration.y}, {msg.linear_acceleration.z}, Angular Vel: {msg.angular_velocity.x}, {msg.angular_velocity.y}, {msg.angular_velocity.z}, Magnetometer: {msg.magnetometer_data.x}, {msg.magnetometer_data.y}, {msg.magnetometer_data.z}, Orientation: {msg.orientation.x}, {msg.orientation.y}, {msg.orientation.z}')
        
    def imu_callback1(self, msg):
       #self.get_logger().info(f'IMU data from sensor 1: {msg}')
       print(f'Sensor 1 - Linear Acc: {msg.linear_acceleration.x}, {msg.linear_acceleration.y}, {msg.linear_acceleration.z}, Angular Vel: {msg.angular_velocity.x}, {msg.angular_velocity.y}, {msg.angular_velocity.z}, Magnetometer: {msg.magnetometer_data.x}, {msg.magnetometer_data.y}, {msg.magnetometer_data.z}, Orientation: {msg.orientation.x}, {msg.orientation.y}, {msg.orientation.z}')
       
    def imu_callback2(self, msg):
        #self.get_logger().info(f'IMU data from sensor 2: {msg}')
        print(f'Sensor 2 - Linear Acc: {msg.linear_acceleration.x}, {msg.linear_acceleration.y}, {msg.linear_acceleration.z}, Angular Vel: {msg.angular_velocity.x}, {msg.angular_velocity.y}, {msg.angular_velocity.z}, Magnetometer: {msg.magnetometer_data.x}, {msg.magnetometer_data.y}, {msg.magnetometer_data.z}, Orientation: {msg.orientation.x}, {msg.orientation.y}, {msg.orientation.z}')

def main(args=None):
    rclpy.init(args=args)

    multi_sensor_subscriber = MultiSensorSubscriber()
    rclpy.spin(multi_sensor_subscriber)
    multi_sensor_subscriber.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()