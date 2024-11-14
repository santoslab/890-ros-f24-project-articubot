# one of several possible launch files for our robot 
#  - this one sets up a gazebo simulation in an empty world

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
import xacro


def generate_launch_description():

    # Get package directories for later use
    #  get share directory of ros_gz_sim library
    #pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    #  get share directory of my robot description
    pkg_my_robot_description = get_package_share_directory('articubot_description')

    # Start Gazebo with an empty world, using ros_gz_sim library
    #  ...use an existing launch file (gz_sim.launch.py) from the library
    # gz_sim = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
    #     launch_arguments={
    #         'gz_args': '-r ' + os.path.join(pkg_my_robot_description,'config','camera_sensor.sdf')
    #     }.items(),
    # )

    # Parse robot description from xacro
    robot_description_file = os.path.join(pkg_my_robot_description, 'urdf', 'my_robot.urdf.xacro')
    robot_description_config = xacro.process_file(
        robot_description_file
    )
    robot_description = {'robot_description': robot_description_config.toxml()}

    controller_params = os.path.join(
        pkg_my_robot_description,
        'config',
        'my_controllers.yaml'
    )

    # Spawn the robot in the world, using ros_gz_sim library
    # spawn = Node(
    #     package='ros_gz_sim',
    #     executable='create',
    #     arguments=[
    #         '-name', 'my_robot',
    #         '-topic', 'robot_description',  # robot description topic (e.g., for RVIZ)
    #     ],
    #     output='screen',
    # )

    robot_description_from_node = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description_from_node, controller_params]
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',  # ???? ToDo - what is "both"
        parameters=[robot_description, {"use_sim_time": False}],
    )

    # ROS2-Gazebo Bridge
    # bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     arguments=[
    #         '/world/camera_sensor/model/my_robot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
    #         '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
    #         '/model/my_robot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
    #         '/model/my_robot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
    #         '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
    #         '/camera@sensor_msgs/msg/Image@gz.msgs.Image',
    #         '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'
    #     ],
    #     remappings=[
    #         ('/world/camera_sensor/model/my_robot/joint_state', 'joint_states'),
    #         ('/model/my_robot/tf', 'tf'),
    #         ('/cmd_vel', 'diff_cont/cmd_vel_unstamped'),
    #         ('/model/my_robot/odometry', '/diff_cont/odom')

    #     ],
    #     output='screen'
    # )

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

    delayed_diff_cont_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_cont]
        )
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad]
        )
    )

    # Rviz
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(pkg_my_robot_description, 'rviz', 'urdf_config.rviz')],
    )
    print(os.path.join(pkg_my_robot_description, 'rviz', 'urdf_config.rviz'))
    return LaunchDescription([
        #rviz,
        robot_state_publisher,
        delayed_controller_manager,
        delayed_diff_cont_spawner,
        delayed_joint_broad_spawner
    ])