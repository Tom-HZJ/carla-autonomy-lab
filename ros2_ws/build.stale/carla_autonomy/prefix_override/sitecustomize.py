import sys
if sys.prefix == '/home/tom/Desktop/ROS2/envs/ros2':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/tom/Desktop/ROS2/ros2_ws/install/carla_autonomy'
