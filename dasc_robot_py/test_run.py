# Run this on terminal once
# export PYTHONPATH=/root/px4_ros_com_ros2/src/robot-framework-py/:${PYTHONPATH}


from dasc_robots.robot import Robot
import rclpy

def main(args=None):
    rclpy.init(args=args)

    robot_1 = Robot('rover1', 1)