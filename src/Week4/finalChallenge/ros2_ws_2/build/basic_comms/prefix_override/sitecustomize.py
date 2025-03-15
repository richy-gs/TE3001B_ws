import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/roli_005/Downloads/ros2_ws_2/install/basic_comms'
