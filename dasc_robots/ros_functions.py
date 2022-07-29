import ctypes


def ros_init(lib: ctypes.CDLL) -> None:
    """Initializes ROS Nodes in C++.

    ARGUMENTS
        lib: C++ shared object

    RETURNS
        None

    """
    lib.rosInit()


def ros_ok(lib: ctypes.CDLL) -> bool:
    """Checks to see whether the ROS nodes are functioning properly.

    ARGUMENTS
        lib: C++ shared object

    RETURNS
        ok: True if okay, False otherwise

    """
    lib.restype = ctypes.c_bool
    return bool(lib.rosOk())


def start_ros_nodes(lib: ctypes.CDLL,
                    robots: List) -> None:
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

    # Format the list as an array of pointers
    RobotArrayNr = ctypes.c_void_p * nr
    robot_array = RobotArrayNr(*robot_objs)

    # Call to C++ library
    lib.startNodes(robot_array, ctypes.c_int(nr))


def close_ros_nodes(lib: ctypes.CDLL) -> None:
    """Closes ROS nodes.

    ARGUMENTS
        lib: C++ shared object

    RETURNS
        None

    """
    lib.closeNodes()
