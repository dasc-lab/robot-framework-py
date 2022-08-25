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

    robot3 = Robot("rover3", 3)
    robot7 = Robot("rover7", 7)
    print("Robot Initialized")

    robot3.init()
    robot7.init()

    robots = [robot3, robot7]
    threads = start_ros_nodes(robots)

    robot3.set_command_mode( 'velocity' )
    robot7.set_command_mode( 'velocity' )

    vx = 0.0
    wz = 2.0
    for i in range(100):
        robot3.command_velocity( np.array([0,vx,0,0,wz]) )
        robot7.command_velocity( np.array([0,vx,0,0,wz]) )
        rate.sleep()
        
    robot3.cmd_offboard_mode()
    robot3.arm()

    robot7.cmd_offboard_mode()
    robot7.arm()

    i = 0
    while rclpy.ok():
        msg.data = 'Hello World: %d' % i
        i += 1
        # node.get_logger().info('Publishing: "%s"' % msg.data)
        publisher.publish(msg)
        robot3.command_velocity( np.array([0,vx,0,0,wz]) )
        robot7.command_velocity( np.array([0,vx,0,0,wz]) )
        quat = robot3.get_body_quaternion()
        acc = robot3.get_body_acceleration()
        acc_world = robot3.get_world_acceleration()
        vel = robot3.get_world_velocity()
        brate = robot3.get_body_rate()
        # print(f"quat:{quat}, acc:{acc}, wacc:{acc_world}, vel:{vel}, brate:{brate}")   

        rate.sleep()

    node.destroy_node()
    rclpy.shutdown() 

