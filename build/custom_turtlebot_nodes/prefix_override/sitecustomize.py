import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/atr-lab/Desktop/main_workstation/ROS_autonomous_delivery/install/custom_turtlebot_nodes'
