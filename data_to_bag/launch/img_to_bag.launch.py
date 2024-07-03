from launch import LaunchDescription
from launch_ros.actions import Node 
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    log_level = 'info'

    return LaunchDescription([
        DeclareLaunchArgument(
            'input/path',
            description="Input Rosbag Path" 
            ),
        DeclareLaunchArgument(
            'output/bag_path',
            description="Output Rosbag Path"
            ),
                 
        DeclareLaunchArgument(
            'input/img_topics',
            description="Input Image Topics",
            ),
        
        Node(
            package="data_to_bag",
            executable="img_to_bag_node",
            name="img_to_bag_node",
            parameters=[{'input/path':LaunchConfiguration('input/path')},
                        {'output/bag_path':LaunchConfiguration('output/bag_path')},
                        {'input/img_topics':LaunchConfiguration('input/img_topics')}
                        ],
            ros_arguments=['--log-level', log_level]
            )
        ])