from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    sp_node_carro = Node(
        name="sp_gen",
        package="inverted_pendulum",
        executable="set_point",
        emulate_tty=True,
        output="screen",
        namespace="carro",
        parameters=[
            {
                "signal_type": "step",
                "amplitude": 5.0,
            }
        ],
    )

    sp_node_angle = Node(
        name="sp_gen",
        package="inverted_pendulum",
        executable="set_point",
        emulate_tty=True,
        output="screen",
        namespace="angle",
        parameters=[
            {
                "signal_type": "step",
                "amplitude": 0.0,
            }
        ],
    )

    ctrl_node_carro = Node(
        name="ctrl",
        package="inverted_pendulum",
        executable="controller",
        emulate_tty=True,
        output="screen",
        namespace="carro",
        parameters=[
            {
                "Kp": 1.0,
                "Ki": 0.0,
                "Kd": 0.0,
            }
        ],
    )

    ctrl_node_angle = Node(
        name="ctrl",
        package="inverted_pendulum",
        executable="controller",
        emulate_tty=True,
        output="screen",
        namespace="angle",
        parameters=[
            {
                "Kp": 1.0,
                "Ki": 0.0,
                "Kd": 0.0,
            }
        ],
    )

    sum_node = Node(
        name="sumador",
        package="inverted_pendulum",
        executable="sumador",
        emulate_tty=True,
        output="screen",
    )

    inv_pendulum_node = Node(
        package="inverted_pendulum",
        executable="inverted_pendulum",
        emulate_tty=True,
        output="screen",
    )

    l_d = LaunchDescription(
        [
            sp_node_carro,
            sp_node_angle,
            ctrl_node_carro,
            ctrl_node_angle,
            sum_node,
            inv_pendulum_node,
        ]
    )

    return l_d
