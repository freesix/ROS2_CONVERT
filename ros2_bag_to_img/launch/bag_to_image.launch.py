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
            'input/bag_format',
            default_value="cdr",
            description="Rosbag format"
            ),    

    
        DeclareLaunchArgument(
            'input/bag_storage_id',
            default_value="sqlite3",
            description="Rosbag storage_id"
            ),
        DeclareLaunchArgument(
            'input/topics',
            description="List of Input Topics"
            ),
        DeclareLaunchArgument(
            'output/path',
            default_value="/tmp",
            description="Output path imu files"
            ),
    
        Node(
            package="ros2_bag_to_image",
            executable="bag_to_image_node",
            name="bag_to_image_node",
            parameters=[{'input/path':LaunchConfiguration('input/path')},
                        {'input/bag_format':LaunchConfiguration('input/bag_format')},
                        {'input/bag_storage_id':LaunchConfiguration('input/bag_storage_id')},
                        {'input/topics':LaunchConfiguration('input/topics')},
                        {'output/path':LaunchConfiguration('output/path')}],
            ros_arguments=['--log-level', log_level]
            )
        ])