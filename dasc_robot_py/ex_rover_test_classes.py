# Run this on terminal once
# export PYTHONPATH=/root/px4_ros_com_ros2/src/robot-framework-py/:${PYTHONPATH}


from dasc_robots.robot import Robot
from dasc_robots.ros_functions import *
import rclpy
from rclpy.node import Node
import numpy as np

from std_msgs.msg import String

class MinimalPublisher(Node):

    def __init__(self, robots):
        super().__init__('minimal_publisher')
        self.robots = robots
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

        # print(" acc data: ", self.robots[0].get_body_quaternion())

        vx = 0.0
        wz = 1.0
        self.robots[0].command_velocity( np.array([0,vx,0,0,wz]) )

def main(args=None):
    rclpy.init(args=args)

    ros_init("test_run")

    robot3 = Robot("rover3", 3)
    robot4 = Robot("rover4", 4)
    print("Robot Initialized")

    robot3.init()
    robot4.init()

    robots = [robot3, robot4]

    robot3.set_command_mode( 'velocity' )
    robot3.cmd_offboard_mode()
    robot3.arm()

    threads = start_ros_nodes(robots)

    minimal_publisher = MinimalPublisher(robots)

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

    print("hello")

