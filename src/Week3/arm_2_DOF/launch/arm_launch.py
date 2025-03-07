from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    sp_node = Node(
        name="sp_gen",
        package="arm_2_DOF",
        executable="set_point",
        emulate_tty=True,
        output="screen",
        parameters=[
            {
                "signal_type_q1": "step",
                "amplitude_q1": 0.8,
                "signal_type_q2": "step",
                "amplitude_q2": 0.5,
            }
        ],
    )

    ctrl_node = Node(
        name="ctrl",
        package="arm_2_DOF",
        executable="controller",
        emulate_tty=True,
        output="screen",
    )

    arm_sys_node = Node(
        package="arm_2_DOF",
        executable="arm_sys",
        emulate_tty=True,
        output="screen",
    )

    foxglove_studio = ExecuteProcess(cmd=["foxglove-studio"])

    foxglove_bridge = ExecuteProcess(
        cmd=["ros2", "launch", "foxglove_bridge", "foxglove_bridge_launch.xml"]
    )

    l_d = LaunchDescription(
        [
            sp_node,
            ctrl_node,
            arm_sys_node,
            foxglove_studio,
            foxglove_bridge,
        ]
    )

    return l_d
