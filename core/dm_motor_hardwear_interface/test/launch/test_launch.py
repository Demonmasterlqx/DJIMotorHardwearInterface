import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PythonExpression, Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import xacro
import os
from launch.actions import ExecuteProcess
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)
from srdfdom.srdf import SRDF


def generate_launch_description():
    robot_description_pkg_dir = get_package_share_directory('dm_motor_hardwear_interface')
    xacro_file = os.path.join(robot_description_pkg_dir, "test", 'config', 'test_robot.urdf.xacro')
    robot_description_content = Command(['xacro', ' ', xacro_file])

    controller_config_pkg_dir = get_package_share_directory('dm_motor_hardwear_interface')
    controller_config_file = os.path.join(controller_config_pkg_dir, 'test', 'config', 'ros2_controller.yaml')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content}],
        output='screen'
    )

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controller_config_file],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        output='screen'
    )

    spawner = Node(
        package='controller_manager',
        executable='spawner',
        name="arm_controller",
        arguments=[
            "arm_controller",
            '--param-file',
            controller_config_file,
            "--controller-manager-timeout", "1200", "--switch-timeout", "1000"
            ],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        controller_manager_node,
        spawner
    ])