import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/mrl_6534/ros2_ws/src/encoder_bridge/install/encoder_bridge'
