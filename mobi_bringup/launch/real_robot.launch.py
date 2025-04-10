import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_slam = LaunchConfiguration("use_slam")

    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="false"
    )

    hardware_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("mobi_firmware"),
                "launch",
                "hardware_interface.launch.py"
            )
        )
    )

    laser_driver = Node(
        package="rplidar_ros",
        executable="rplidar_node",
        name="rplidar_node",
        parameters=[os.path.join(
            get_package_share_directory("mobi_bringup"),
            "config",
            "rplidar_a1.yaml"
        )],
        output="screen"
    )

    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("mobi_controller"),
                "launch",
                "controller.launch.py"
            )
        ),
        launch_arguments={"use_sim_time": "False"}.items()
    )

    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("mobi_controller"),
                "launch",
                "joystick.launch.py"
            )
        ),
        launch_arguments={"use_sim_time": "False"}.items()
    )

    imu_driver_node = Node(
        package="mobi_firmware",
        executable="mpu6050_driver.py",
        output="screen"
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("mobi_localization"),
                "launch",
                "global_localization.launch.py"
            )
        ),
        launch_arguments={"use_sim_time": "False"}.items(),
        condition=UnlessCondition(use_slam)
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("mobi_mapping"),
                "launch",
                "slam.launch.py"
            )
        ),
        launch_arguments={"use_sim_time": "False"}.items(),
        condition=IfCondition(use_slam)
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("mobi_navigation"),
                "launch",
                "navigation.launch.py"
            )
        ),
        launch_arguments={"use_sim_time": "False"}.items()
    )

    ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("mobi_localization"),
                "launch",
                "ekf.launch.py"
            )
        ),
        launch_arguments={"use_sim_time": "False"}.items()
    )

    return LaunchDescription([
        use_slam_arg,
        hardware_interface,
        laser_driver,
        controller,
        joystick,
        imu_driver_node,
        localization,
        slam,
        navigation,
        # ekf,
    ])
