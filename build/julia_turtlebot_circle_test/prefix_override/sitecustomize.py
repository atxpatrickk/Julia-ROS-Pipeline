import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/atxpatrickk/ros2_ws/src/julia_turtlebot_circle_test/install/julia_turtlebot_circle_test'
