import ctypes
from typing import List
import platform

def load_library(filename: str = 'libdasc_robot_lib.so',#'./lib/libdascBots.a',
                 mode: dict = None) -> ctypes.cdll:
    """Loads a C++ library specified by filename."""
    if mode is not None:
        return ctypes.CDLL(filename, **mode)  # Maybe this is incorrect
    else:
        return ctypes.cdll.LoadLibrary(filename)

l_mode = dict(winmode=0) if platform.python_version() >= '3.8' else dict()
path = '/root/px4_ros_com_ros2/install/dasc_robot/lib/dasc_robot/libdasc_robot_lib.so' #'/root/px4_ros_com_ros2/src/robot-framework-py/dasc_robots/lib/libdasc_robot_lib.so'
_lib = load_library(path, l_mode)

# max size of node_name: 100 characters
def ros_init(node_name: str) -> None: #lib: ctypes.CDLL
    """Initializes ROS Nodes in C++.

    ARGUMENTS
        lib: C++ shared object

    RETURNS
        None

    """
    _lib.rosInit(ctypes.c_wchar_p(node_name)) #c_wchar_p


def ros_ok() -> bool:
    """Checks to see whether the ROS nodes are functioning properly.

    ARGUMENTS
        lib: C++ shared object

    RETURNS
        ok: True if okay, False otherwise

    """
    _lib.restype = ctypes.c_bool
    return bool(_lib.rosOk())


def start_ros_nodes(robots: List) -> None: #def start_ros_nodes(lib: ctypes.CDLL, robots: List) -> None:
    """Starts each robot on a separate ROS node using multiple C++ threads.

    ARGUMENTS
        lib: C++ shared object
        robots: list of Robot objects (found in robot.py)

    RETURNS
        None

    """
    # Get the relevant arguments
    nr = len(robots)
    
    robot_objs = [robot._obj for robot in robots]
    print(robot_objs)
    # Format the list as an array of pointers
    RobotArrayNr = ctypes.c_void_p * nr
    # robot_array = RobotArrayNr(*robot_objs)
    robot_array = RobotArrayNr(*robot_objs)

    # Call to C++ library
    return _lib.startNodes(robot_array, ctypes.c_int(nr))


def close_ros_nodes() -> None:
    """Closes ROS nodes.

    ARGUMENTS
        lib: C++ shared object

    RETURNS
        None

    """
    _lib.closeNodes()

def start_main_thread() -> None:
    """ Starts the thread for main function so that it is independent from all the robot object nodes

    ARGUMENTS
        lib: C++ shared object

    RETURNS
        None
    
    """
    _lib.join_global_thread()

def ros_thread_sleep(time: int) -> None:
    """ ROS sleep function that can be used inside loops to execute them at a fixed frequency

    ARGUMENTS
        lib: C++ shared object
        time: time in milliseconds

    RETURNS
        None
    
    """
    _lib.ros_thread_sleep(time)
