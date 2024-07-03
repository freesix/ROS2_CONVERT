from launch import LaunchDescription
from launch_ros.actions import Node 
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    log_level = 'info'

    return LaunchDescription([
        DeclareLaunchArgument(
            'input/path',
            description="Input data Path" 
            ),
        DeclareLaunchArgument(
            'output/bag_path',
            description="Output Rosbag Path"
            ),
        DeclareLaunchArgument(
            'input/imu_topic',
            description="Input IMU Topic",
            ),
                         
        Node(
            package="data_to_bag",
            executable="imu_to_bag_node",
            name="data_to_bag_node",
            parameters=[{'input/path':LaunchConfiguration('input/path')},
                        {'output/bag_path':LaunchConfiguration('output/bag_path')},
                        {'input/imu_topic':LaunchConfiguration('input/imu_topic')}
                        ],
            ros_arguments=['--log-level', log_level]
            )
        ])