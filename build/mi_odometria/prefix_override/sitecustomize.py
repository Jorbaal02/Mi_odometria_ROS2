import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jorbaal/ros2_ws/src/mi_odometria/install/mi_odometria'
