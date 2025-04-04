import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    mobi_description_dir = get_package_share_directory("mobi_description")
    is_sim_arg = DeclareLaunchArgument(name="is_sim", default_value="true", description="Flag to indicate simulation mode")

    model_arg = DeclareLaunchArgument(name="model", default_value=os.path.join(
                                        mobi_description_dir, "urdf", "mobi.urdf.xacro"
                                        ),
                                      description="Absolute path to robot urdf file")

    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model"), " is_sim:=", LaunchConfiguration("is_sim")]),
                                       value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(mobi_description_dir, "rviz", "display.rviz")],
    )

    return LaunchDescription([
        model_arg,
        is_sim_arg,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])