import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("excavator_kinematics")
    params_file = os.path.join(pkg_share, "config", "excavator_params.yaml")
    urdf_file = os.path.join(pkg_share, "urdf", "excavator.urdf")

    with open(urdf_file, "r", encoding="utf-8") as urdf_handle:
        robot_description = urdf_handle.read()

    # WSL fallback to software rendering for Gazebo GUI stability.
    set_sw_rendering = [
        SetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE", "1"),
        SetEnvironmentVariable("GALLIUM_DRIVER", "llvmpipe"),
        SetEnvironmentVariable("MESA_GL_VERSION_OVERRIDE", "3.3"),
    ]

    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": "-r empty.sdf"}.items(),
    )

    enable_keyboard = LaunchConfiguration("enable_keyboard")

    state_node = Node(
        package="excavator_kinematics",
        executable="excavator_state_node",
        name="excavator_state_node",
        output="screen",
        parameters=[params_file, {"use_sim_time": True}],
    )

    visualization_node = Node(
        package="excavator_kinematics",
        executable="excavator_visualization_node",
        name="excavator_visualization_node",
        output="screen",
        parameters=[params_file, {"frame_id": "excavator_base"}, {"use_sim_time": True}],
    )

    keyboard_node = Node(
        package="excavator_kinematics",
        executable="excavator_keyboard_node",
        name="excavator_keyboard_node",
        output="screen",
        parameters=[params_file],
        condition=IfCondition(enable_keyboard),
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": True},
        ],
    )

    clock_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    spawn_entity_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "excavator",
            "-topic",
            "robot_description",
            "-z",
            "0.05",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_keyboard",
                default_value="false",
                description="Start keyboard teleop node for desired angle commands",
            ),
            *set_sw_rendering,
            gz_sim_launch,
            clock_bridge_node,
            robot_state_publisher_node,
            state_node,
            visualization_node,
            keyboard_node,
            spawn_entity_node,
        ]
    )
