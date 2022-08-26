# Run this on terminal once
# export PYTHONPATH=/root/px4_ros_com_ros2/src/robot-framework-py/:${PYTHONPATH}

# see this page: https://answers.ros.org/question/358343/rate-and-sleep-function-in-rclpy-library-for-ros2/
# https://answers.ros.org/question/320474/ros2-correct-usage-of-spin_once-to-receive-multiple-callbacks-synchronized/
# https://stackoverflow.com/questions/1775612/python-threads-do-not-run-in-c-application-embedded-interpreter

from dasc_robots.robot import Robot
from dasc_robots.ros_functions import *
import rclpy
from rclpy.node import Node
import numpy as np

import threading

from std_msgs.msg import String

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('minimal_publisher')

    # Spin in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()
    rate = node.create_rate(30)

    publisher = node.create_publisher(String, 'topic', 10)
    
    msg = String()

    ros_init("test_run")

    robot = Robot("drone1", 1)
    print("Robot Initialized")

    robot.init()

    robots = [robot]

    threads = start_ros_nodes(robots)
    
    robot.set_command_mode( 'position' )

    for i in range(100):
        robot.command_position( np.array([0,0,0.5,0,0]) )
        rate.sleep()
        
    robot.cmd_offboard_mode()
    robot.arm()

    i = 0
    while rclpy.ok():
        robot.command_position( np.array([0,0,0.5,0,0]) )
        rate.sleep()


    node.destroy_node()
    rclpy.shutdown() 

