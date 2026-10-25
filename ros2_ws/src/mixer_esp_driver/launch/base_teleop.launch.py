"""Low-level robot bringup: ESP driver (load cells, mixer motors, door servo).

The Create 3 base needs no launch file: its firmware already publishes
/Team2/* and listens on /Team2/cmd_vel. Drive it with keyboard teleop:

  ros2 launch mixer_esp_driver base_teleop.launch.py
  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/Team2/cmd_vel

or let this launch file open the teleop in its own xterm window (needs a
display, e.g. on the Jetson desktop, not over plain ssh):

  ros2 launch mixer_esp_driver base_teleop.launch.py teleop:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'teleop', default_value='false',
            description='Open teleop_twist_keyboard in an xterm window.'),
        DeclareLaunchArgument(
            'cmd_vel_topic', default_value='/Team2/cmd_vel',
            description='Create 3 velocity command topic.'),
        DeclareLaunchArgument(
            'web', default_value='true',
            description='Start the web control panel on port 8000.'),
        Node(
            package='mixer_esp_driver',
            executable='esp_driver_node',
            name='esp_driver_node',
            output='screen',
        ),
        Node(
            package='mixer_esp_driver',
            executable='web_control',
            name='web_control',
            output='screen',
            condition=IfCondition(LaunchConfiguration('web')),
        ),
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            output='screen',
            prefix='xterm -e',
            remappings=[('cmd_vel', LaunchConfiguration('cmd_vel_topic'))],
            condition=IfCondition(LaunchConfiguration('teleop')),
        ),
    ])
