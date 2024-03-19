from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'serial', '--dev', '/dev/ttyACM0'],
            output='screen'
        ),
        Node(
            package='tree_sensorization',
            executable='multi_sensor_subscriber',
            output='screen'
        ),
    ])