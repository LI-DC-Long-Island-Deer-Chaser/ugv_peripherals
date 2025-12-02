from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Check if apm.launch is XML or Python
    mavros_launch_file = os.path.join(
        get_package_share_directory('mavros'),
        'launch',
        'apm.launch'
    )
    
    # Use XMLLaunchDescriptionSource for .launch files (ROS1 style XML)
    # Use PythonLaunchDescriptionSource for .launch.py files
    mavros_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(mavros_launch_file)
    )
    
    return LaunchDescription([
        # run mavros launch file
        mavros_launch,
        
        # run serial controller node
        Node(
            package='serial_controller',
            executable='serial_controller_node',
            name='serial_controller_node',
            output='screen',
        ),
        
        # run battery node
        Node(
            package='battery_node',  # Changed from 'battery_test' to match your package
            executable='battery_test',
            name='battery_test',
            output='screen',
        ),
            
        # run audio node
        Node(
            package='speaker_controller',
            executable='speaker_node',
            name='speaker_node',
            output='screen',
        ),
        
        # run periodic_service_caller node
        Node(
            package='periodic_growl',
            executable='periodic_growl_node',
            name='periodic_growl_node',
            output='screen',
        ),
        
        # run flashing lights node
        Node(
            package='lights',
            executable='flashing_node',
            name='flashing_node',
            output='screen',
        ),
    ])
