# launch/full_system_launch.py
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to MAVROS apm.launch
    mavros_launch_file = os.path.join(
        get_package_share_directory('mavros'),
        'launch',
        'apm.launch'  # XML-style launch file
    )

    mavros_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(mavros_launch_file)
    )

    return LaunchDescription([
        # Include MAVROS launch
        mavros_launch,

        # Serial controller node
        Node(
            package='ugv_peripherals',
            executable='serial_controller',
            name='serial_controller',
            output='screen'
        ),

        # Lights package battery_node
        Node(
            package='ugv_peripherals',
            executable='battery',
            name='battery',
            output='screen'
        ),
      
        # run audio node
        Node(
            package='ugv_peripherals',
            executable='speaker',
            name='speaker',
            output='screen',
        ),

        # run periodic_service_caller node
        Node(
            package='ugv_peripherals',
            executable='periodic_growl',
            name='periodic_growl',
            output='screen',
        ),

        # run flashing lights node
        Node(
            package='ugv_peripherals',
            executable='flashing',
            name='flashing',
            output='screen',
        )
    ])
