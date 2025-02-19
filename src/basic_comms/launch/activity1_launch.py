# Import your libraries here
from launch import LaunchDescription
from launch_ros.actions import Node


# write your launch here
def generate_launch_description():
    talker_node = Node(package="basic_comms", executable="talker", output="screen")
    listener_node = Node(package="basic_comms", executable="listener", output="screen")

    i_d = LaunchDescription([talker_node, listener_node])
    return i_d
