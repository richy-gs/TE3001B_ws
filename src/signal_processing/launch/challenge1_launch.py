# Import your libraries here
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


# write your launch here
def generate_launch_description():
    talker_node = Node(
        package="signal_processing", executable="signal_generator", output="screen"
    )
    listener_node = Node(
        package="signal_processing", executable="proc_signal", output="screen"
    )
    # rqt_node = Node(
    #     name="rqt_plot",
    #     package="rqt_plot",
    #     executable="rqt_plot",
    #     arguments=["/signal/data", "/proc_signal/data"],
    # )

    foxglove_studio = ExecuteProcess(cmd=["foxglove-studio"])
    foxglove_launch = ExecuteProcess(
        cmd=["ros2", "launch", "foxglove_bridge", "foxglove_bridge_launch.xml"],
    )

    i_d = LaunchDescription(
        [talker_node, listener_node, foxglove_studio, foxglove_launch]
    )
    return i_d
