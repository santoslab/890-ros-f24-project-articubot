# one of several possible launch files for our robot 
#  - this one sets up a gazebo simulation in an empty world

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    #  get share directory of my robot description
    pkg_my_robot_description = get_package_share_directory('articubot_description')

    # Parse robot description from xacro
    robot_description_file = os.path.join(pkg_my_robot_description, 'urdf', 'my_robot.urdf.xacro')
    robot_description_config = xacro.process_file(
        robot_description_file
    )
    robot_description = {'robot_description': robot_description_config.toxml()}

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',  # ???? ToDo - what is "both"
        parameters=[robot_description,{"use_sim_time":False}],
    )

    diff_cont = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont']
    )

    joint_broad = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad']
    )

    # Rviz
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(pkg_my_robot_description, 'rviz', 'urdf_config.rviz')],
    )

    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])

    controller_params = os.path.join(
        pkg_my_robot_description,
        'config',
        'my_controllers.yaml'
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description},
                controller_params],
        )

    




    print(os.path.join(pkg_my_robot_description, 'rviz', 'urdf_config.rviz'))
    return LaunchDescription([
        rviz,
        robot_state_publisher,
        diff_cont,
        joint_broad
    ])