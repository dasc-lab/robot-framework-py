# Run this on terminal once
# export PYTHONPATH=/root/px4_ros_com_ros2/src/robot-framework-py/:${PYTHONPATH}


from dasc_robots.robot import Robot
from dasc_robots.ros_functions import *
import rclpy

def main(args=None):
    # rclpy.init(args=args)

    ros_init("test_run")

    robot1 = Robot("rover1", 1)
    robot2 = Robot("rover2", 1)
    print("Robot Initialized")

    robot1.init()
    robot2.init()

    start_ros_nodes([robot1, robot2])