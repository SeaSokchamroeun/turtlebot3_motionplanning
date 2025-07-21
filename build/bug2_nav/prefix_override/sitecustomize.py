import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/roeunsea/turtlebot3_ws/src/motion_planning/install/bug2_nav'
