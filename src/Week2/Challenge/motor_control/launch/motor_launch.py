import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    motor_config = os.path.join(
        get_package_share_directory("motor_control"), "config", "motor_params.yaml"
    )

    motor_node = Node(
        name="motor_sys",
        package="motor_control",
        executable="dc_motor",
        emulate_tty=True,
        output="screen",
        parameters=[motor_config],
    )

    sp_node = Node(
        name="sp_gen",
        package="motor_control",
        executable="set_point",
        emulate_tty=True,
        output="screen",
    )

    l_d = LaunchDescription([motor_node, sp_node])

    return l_d
