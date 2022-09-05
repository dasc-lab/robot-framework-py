import ctypes
import platform
import numpy as np
import numpy.typing as npt


def load_library(filename: str = 'libdasc_robot_lib.so',#'./lib/libdascBots.a',
                 mode: dict = None) -> ctypes.cdll:
    """Loads a C++ library specified by filename."""
    if mode is not None:
        return ctypes.CDLL(filename, **mode)  # Maybe this is incorrect
    else:
        return ctypes.cdll.LoadLibrary(filename)

l_mode = dict(winmode=0) if platform.python_version() >= '3.8' else dict()
path = '/root/px4_ros_com_ros2/install/dasc_robot/lib/dasc_robot/libdasc_robot_lib.so'  #'/root/px4_ros_com_ros2/src/robot-framework-py/dasc_robots/lib/libdasc_robot_lib.so'
robot_lib = load_library(path, l_mode)

class Robot:
    """Interface to C++ DASCRobot object."""

    def __init__(self,
                 name: str,
                 robot_id: int):
        """Initializes Robot in C++ and obtains corresponding object functions."""
        # From Python 3.8 onwards, there is a reported bug in CDLL.__init__()
        self._obj = robot_lib.make_robot_object(name, ctypes.c_uint8(robot_id))
        print(f"python: { self._obj }")
        self.set_arg_res_types()

    def init(self) -> bool:
        """Initializes the robot.

        Arguments:
            None

        Returns:
            Success / error

        """
        return robot_lib.init(self._obj)

    def arm(self) -> bool:
        """Arms the robot.

        Arguments:
            None

        Returns:
            Success / error

        """
        return robot_lib.arm(self._obj)

    def get_current_timestamp_us(self, 
                            px4_sync: bool = True) -> np.float128:
        """Get the times in microseconds for which robot has been working

        Arguments:
            px4_sync: True if want time to be synced with px4. False is just want ROS time back
                      Note that px4 syncing is done per robot. time across multiple pixhawks is not synced and as such for multi robot experiments
                      with time specifications, it might be better to use ROS time

        Returns:
            time in microseconds

        """
        return np.float128( robot_lib.get_current_timestamp_us(self._obj, px4_sync) )

    def disarm(self) -> bool:
        """Disarms the robot.

        Arguments:
            None

        Returns:
            None

        """
        return robot_lib.disarm(self._obj)

    def cmd_offboard_mode(self) -> bool:
        """Sets the robot control to offboard mode.

        Arguments
            None

        Returns
            Success / error
            """
        return robot_lib.cmdOffboardMode(self._obj)

    def get_world_position(self) -> npt.NDArray:
        """Retrieves the world position from the robot.

        Arguments
            None

        Returns
            x, y, z position in world coordinates

        """
        pos_array = (3 * ctypes.c_double)()
        robot_lib.getWorldPosition(self._obj, pos_array)
        return np.array(pos_array)


    def get_world_velocity(self) -> npt.NDArray:
        """Retrieves the world velocity from the robot.

        Arguments
            None

        Returns
            vx, vy, vz velocity in world coordinates

        """
        vel_array = (3 * ctypes.c_double)()
        robot_lib.getWorldVelocity(self._obj, vel_array)
        return np.array(vel_array)

    def get_world_acceleration(self) -> npt.NDArray:
        """Retrieves the world acceleration from the robot.

        Arguments
            None

        Returns
            ax, ay, az acceleration in world coordinates

        """
        acc_array = (3 * ctypes.c_double)()
        robot_lib.getWorldAcceleration(self._obj, acc_array)
        return np.array(acc_array)

    def get_body_acceleration(self) -> npt.NDArray:
        """Retrieves the body acceleration from the robot.

        Arguments
            None

        Returns
            ax, ay, az acceleration in body coordinates

        """
        acc_array = (3 * ctypes.c_double)()
        robot_lib.getBodyAcceleration(self._obj, acc_array)
        return np.array(acc_array)

    def get_body_rate(self) -> npt.NDArray:
        """Retrieves the body angular rate from the robot.

        Arguments
            None

        Returns
            wx, wy, wz body angular rates

        """
        body_rate_array = (3 * ctypes.c_double)()
        robot_lib.getBodyRate(self._obj, body_rate_array)
        return np.array(body_rate_array)

    def emergency_stop(self) -> None:
        """Brings the robot to an emergency stop.

        Arguments
            None

        Returns
            None

        """
        return robot_lib.emergencyStop(self._obj)

    def get_body_quaternion(self,
                            blocking: bool = False) -> npt.NDArray:
        """Gets the body quaternion state from the robot.

        Arguments
            blocking (optional) -- whether or not the call should be blockiing

        Returns
            array containing a, b, c, d quaternions

        """
        quaternion_array = (4 * ctypes.c_double)()
        robot_lib.getBodyQuaternion(self._obj, quaternion_array, blocking)

        return np.array(quaternion_array)

    def set_command_mode(self,
                         mode: str = "position") -> bool:
        """Sets the control mode for the robot (position, velocity, acceleration).

        Arguments
            mode: control mode for robot

        Returns
            Success / error flag

        """
        if mode.lower() == 'position' or mode.lower() == 'pos':
            cmode = 0
        elif mode.lower() == 'velocity' or mode.lower() == 'vel':
            cmode = 1
        elif mode.lower() == 'acceleration' or mode.lower() == 'accel':
            cmode = 2
        else:
            raise ValueError("Unknown vehicle control mode!")

        return robot_lib.setCmdMode(self._obj, cmode)

    def use_external_controller(self,
                         mode: bool = True) -> bool:
        """Sets the flag for using the Geometric Control module inside PX4 bypassing the default PID controller

        Arguments
            mode: whether or not it should use the custom Geometric Controller. True: use Geometric, False: use default PID

        Returns
            Success / error flag

        """

        return robot_lib.useExternalController(self._obj, mode)

    def command_position(self,
                         pos: npt.NDArray) -> bool:
        """Commands the new position setpoint for the robot.

        Arguments
            pos: (x, y, z)

        Returns
            success / error

        """
        x = ctypes.c_double(pos[0])
        y = ctypes.c_double(pos[1])
        z = ctypes.c_double(pos[2])
        yaw = ctypes.c_double(pos[3])
        yaw_rate = ctypes.c_double(pos[4])

        return robot_lib.cmdWorldPosition(self._obj, x, y, z, yaw, yaw_rate)

    def command_velocity(self,
                         vel: npt.NDArray) -> bool:
        """Commands the new velocity setpoint for the robot.

        Arguments
            vel: (vx, vy, vz)

        Returns
            success / error

        """
        vx = ctypes.c_double(vel[0])
        vy = ctypes.c_double(vel[1])
        vz = ctypes.c_double(vel[2])
        yaw = ctypes.c_double(vel[3])
        yaw_rate = ctypes.c_double(vel[4])

        return robot_lib.cmdWorldVelocity(self._obj, vx, vy, vz, yaw, yaw_rate)

    def command_acceleration(self,
                             acc: npt.NDArray) -> bool:
        """Commands the new acceleration setpoint for the robot.

        Arguments
            acc: (ax, ay, az)

        Returns
            success / error

        """
        ax = ctypes.c_double(acc[0])
        ay = ctypes.c_double(acc[1])
        az = ctypes.c_double(acc[2])

        return robot_lib.cmdWorldAcceleration(self._obj, ax, ay, az, 0, 0)

    def command_atttitude(self,
                          quaternions: npt.NDArray,
                          thrust: float) -> bool:
        """Commands the new attitude setpoint for the robot and body thrust.

        Arguments
            quaternions: (qw, qx, qy, qz)
            thrust: body thrust

        Returns
            success / error

        """
        qw = ctypes.c_double(quaternions[0])
        qx = ctypes.c_double(quaternions[1])
        qy = ctypes.c_double(quaternions[2])
        qz = ctypes.c_double(quaternions[2])

        return robot_lib.cmdAttitude(self._obj, qw, qx, qy, qz, ctypes.c_double(thrust))

    def command_angular_rates(self,
                              rates: npt.NDArray,
                              thrust: float) -> bool:
        """Commands the new attitude setpoint for the robot and body thrust.

        Arguments
            rates: (roll, pitch, yaw) rates
            thrust: body thrust

        Returns
            success / error

        """
        roll = ctypes.c_double(rates[0])
        pitch = ctypes.c_double(rates[1])
        yaw = ctypes.c_double(rates[2])

        return robot_lib.cmdAttitude(self._obj, roll, pitch, yaw, ctypes.c_double(thrust))

    def set_gps_origin(self,
                       origin: npt.NDArray) -> None:
        """Sets the GPS origin point for robot localization.

        Arguments
            origin: (lat, lon, alt)

        Returns
            None

        """
        lat = ctypes.c_double(origin[0])
        lon = ctypes.c_double(origin[1])
        alt = ctypes.c_double(origin[2])

        robot_lib.setGPSGlobalOrigin(self._obj, lat, lon, alt)
        
    def set_arg_res_types(self) -> None:
        """Sets the argument and return types for the Robot library."""
        # robot_lib.init.argtypes = [ctypes.c_char_p, ctypes.c_uint8]
        robot_lib.init.restype = ctypes.c_void_p

        robot_lib.arm.argtypes = [ctypes.c_void_p]
        robot_lib.arm.restype = ctypes.c_bool
        robot_lib.disarm.argtypes = [ctypes.c_void_p]
        robot_lib.disarm.restype = ctypes.c_bool
        robot_lib.cmdOffboardMode.argtypes = [ctypes.c_void_p]
        robot_lib.cmdOffboardMode.restype = ctypes.c_bool
        robot_lib.get_current_timestamp_us.argtypes = [ctypes.c_void_p, ctypes.c_bool]
        robot_lib.get_current_timestamp_us.restype = ctypes.c_ulonglong
        # robot_lib.getWorldPosition.argtypes = [ctypes.c_void_p]
        # robot_lib.getWorldPosition.restype = ctypes.POINTER(ctypes.c_double)
        # robot_lib.getWorldVelocity.argtypes = [ctypes.c_void_p]
        # robot_lib.getWorldVelocity.restype = ctypes.POINTER(ctypes.c_double)
        # robot_lib.getWorldAcceleration.argtypes = [ctypes.c_void_p]
        # robot_lib.getWorldAcceleration.restype = ctypes.POINTER(ctypes.c_double)
        # robot_lib.getBodyAcceleration.argtypes = [ctypes.c_void_p]
        # robot_lib.getBodyAcceleration.restype  = ctypes.POINTER(ctypes.c_double)
        # robot_lib.getBodyRate.argtypes = [ctypes.c_void_p]
        # robot_lib.getBodyRate.restype = ctypes.POINTER(ctypes.c_double)
        robot_lib.emergencyStop.argtypes = [ctypes.c_void_p]
        robot_lib.emergencyStop.restype = ctypes.c_void_p

        # Functions with arguments
        # robot_lib.getBodyQuaternion.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_double), ctypes.c_bool]
        # robot_lib.getBodyQuaternion.restype = ctypes.c_bool
        robot_lib.setCmdMode.argtypes = [ctypes.c_void_p, ctypes.c_uint]
        robot_lib.setCmdMode.restype = ctypes.c_bool
        robot_lib.cmdWorldPosition.argtypes = [ctypes.c_void_p, ctypes.c_double, ctypes.c_double, ctypes.c_double,
                                              ctypes.c_double, ctypes.c_double]
        robot_lib.cmdWorldPosition.restype = ctypes.c_bool
        robot_lib.cmdWorldVelocity.argtypes = [ctypes.c_void_p, ctypes.c_double, ctypes.c_double, ctypes.c_double,
                                              ctypes.c_double, ctypes.c_double]
        robot_lib.cmdWorldVelocity.restype = ctypes.c_bool
        robot_lib.cmdLocalVelocity.argtypes = [ctypes.c_void_p, ctypes.c_double, ctypes.c_double, ctypes.c_double,
                                              ctypes.c_double, ctypes.c_double]
        robot_lib.cmdLocalVelocity.restype = ctypes.c_bool
        robot_lib.cmdWorldAcceleration.argtypes = [ctypes.c_void_p,ctypes.c_double, ctypes.c_double, ctypes.c_double,
                                                  ctypes.c_double, ctypes.c_double]
        robot_lib.cmdWorldAcceleration.restype = ctypes.c_bool
        robot_lib.cmdAttitude.argtypes = [ctypes.c_void_p, ctypes.c_double, ctypes.c_double, ctypes.c_double,
                                         ctypes.c_double, ctypes.c_double]
        robot_lib.cmdAttitude.restype = ctypes.c_bool
        robot_lib.cmdRates.argtypes = [ctypes.c_void_p, ctypes.c_double, ctypes.c_double, ctypes.c_double,
                                      ctypes.c_double]
        robot_lib.cmdRates.restype = ctypes.c_bool
        robot_lib.setGPSGlobalOrigin.argtypes = [ctypes.c_void_p, ctypes.c_double, ctypes.c_double, ctypes.c_double]
        robot_lib.setGPSGlobalOrigin.restype = ctypes.c_void_p





