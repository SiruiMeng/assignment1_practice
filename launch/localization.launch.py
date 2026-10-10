"""Start the simulation, ground-truth bridge, odometry estimator and CSV logger.

Usage:
    ros2 launch odom_localization localization.launch.py
    ros2 launch odom_localization localization.launch.py output_file:=baseline.csv
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # The TurtleBot3 launch files read this variable when they are loaded.
    os.environ.setdefault('TURTLEBOT3_MODEL', 'burger')

    pkg_share = get_package_share_directory('odom_localization')
    tb3_gazebo_share = get_package_share_directory('turtlebot3_gazebo')

    arguments = [
        DeclareLaunchArgument(
            'joint_topic', default_value='/joint_states',
            description='JointState topic used as the wheel-encoder input'),
        DeclareLaunchArgument(
            'wheel_radius', default_value='0.033',
            description='Wheel radius assumed by the estimator [m]'),
        DeclareLaunchArgument(
            'wheel_separation', default_value='0.160',
            description='Wheel separation assumed by the estimator [m]'),
        DeclareLaunchArgument(
            'output_file', default_value='data.csv',
            description='CSV file written by the logger node'),
    ]

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_share, 'launch', 'empty_world.launch.py')))

    # Ground truth is used for evaluation only. The script needs the robot to
    # exist in Gazebo, so it is started a few seconds after the simulation.
    ground_truth = TimerAction(
        period=10.0,
        actions=[ExecuteProcess(
            cmd=['bash', os.path.join(pkg_share, 'scripts', 'ground_truth.sh')],
            output='screen')])

    odometry = Node(
        package='odom_localization',
        executable='odometry_node',
        output='screen',
        parameters=[{
            'joint_topic': LaunchConfiguration('joint_topic'),
            'wheel_radius': ParameterValue(
                LaunchConfiguration('wheel_radius'), value_type=float),
            'wheel_separation': ParameterValue(
                LaunchConfiguration('wheel_separation'), value_type=float),
        }])

    logger = Node(
        package='odom_localization',
        executable='logger_node',
        output='screen',
        parameters=[{'output_file_1': LaunchConfiguration('output_file')}])

    return LaunchDescription(
        arguments + [simulation, ground_truth, odometry, logger])
