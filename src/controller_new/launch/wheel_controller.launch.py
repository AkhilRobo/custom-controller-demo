from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_path = get_package_share_directory('controller_new')
    urdf_file_path = os.path.join(pkg_path, 'urdf', 'wheel.urdf.xacro')
    rviz_config_path = os.path.join(pkg_path, 'rviz', 'display.rviz')
    controller_config = os.path.join(pkg_path, 'config', 'wheel_controllers.yml')

    robot_description = {"robot_description": Command(['xacro ', urdf_file_path])}

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_config],
        output="screen",
    )

    spawn_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_velocity_controller", "-c", "/controller_manager"],
        output="screen",
    )

    spawn_jsb_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_path],
        output="screen",
    )

    return LaunchDescription([
        controller_manager_node,
        robot_state_publisher_node,
        rviz_node,
        spawn_controller_node,
        spawn_jsb_node
    ])