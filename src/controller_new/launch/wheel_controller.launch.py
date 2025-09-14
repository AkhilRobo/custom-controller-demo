from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("controller_new"), "urdf", "wheel.urdf.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}


    controller_config = PathJoinSubstitution(
        [FindPackageShare("controller_new"), "config", "wheel_controllers.yaml"]
    )


    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

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

    return LaunchDescription([
        robot_state_publisher_node,
        controller_manager_node,
        spawn_controller_node,
    ])