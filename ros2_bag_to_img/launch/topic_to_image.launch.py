from launch import LaunchDescription
from launch_ros.actions import Node 
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    log_level = 'info'
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'input/topic',
            default_value="/points_raw",
            description="Input Image Topic"
            ),
         
        DeclareLaunchArgument(
            'output/prefix',
            default_value="image",
            description="Output path PNG files"
            ),
        DeclareLaunchArgument(
            'output/path',
            default_value="/tmp",
            description="Output path imu files"
            ),
 
        DeclareLaunchArgument(
            'compressed',
            default_value="True",
            description="Use Compressed Image Transport"
            ),
    
        Node(
            package="ros2_bag_to_image",
            executable="topic_to_image_node",
            name="topic_to_image_node",
            parameters=[{'input/topic':LaunchConfiguration('input/topic')},
                        {'output/prefix':LaunchConfiguration('output/prefix')},
                        {'output/path':LaunchConfiguration('output/path')},
                        {'compressed':LaunchConfiguration('compressed')}
                        ],
            ros_arguments=['--log-level', log_level]
            )
        ])