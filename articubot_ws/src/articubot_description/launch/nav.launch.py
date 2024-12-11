from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    joy_params = os.path.join(get_package_share_directory('articubot_description'), 'config', 'joystick.yaml')
    twist_mux_params = os.path.join(get_package_share_directory('articubot_description'), 'config', 'twist_mux.yaml')

    joy_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[joy_params]
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        parameters=[joy_params],
        remappings=[('/cmd_vel', '/cmd_vel_joy')]
    )

    tmux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[twist_mux_params],
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel')]
    )

    return LaunchDescription([
        tmux_node,
        joy_node,
        teleop_node
    ])