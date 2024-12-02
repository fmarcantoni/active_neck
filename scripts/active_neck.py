#!/usr/bin/env python
"""
Implements 2-axis active neck control to mirror the operator's head movements to the robot's reference frame.

Author(s):
    1. Nikita Boguslavskii (bognik3@gmail.com), Human-Inspired Robotics (HiRo)
       lab, Worcester Polytechnic Institute (WPI), 2024.
    2. Filippo Marcantoni (fmarcantoni@wpi.edu), RBE, Worcester Polytechnic
       Institute (WPI), 2024.

"""

# # Standart libraries:
import rospy
import numpy as np

# # Third party libraries:
from dynamixel_sdk import *

# # Standart messages and services:
from std_msgs.msg import (
    Bool,
    Float64,
)

class ActiveNeck:
    """
    The ActiveNeck class uses the ROS PID package to implement two PID controllers, for pitch and yaw motors, to control the motors velocity 
    to properly map the operator's headset movements to the robot's active neck.
    """

    def __init__(
        self,
        node_name,
    ):
        """
        The __init__ function intializes several constants related to the Dynamixel SDK, initialize motors and ports handler and parameters, 
        and defines the necessary ROS Publishers and Subscribers to use the PID controllers.
        """

        # Initializes several constants related to the Dynamixel SDK
        self.__NODE_NAME = node_name

        # Control table address:
        self.__ADDR_TORQUE_ENABLE = 64
        self.__ADDR_OPERATING_MODE = 11
        self.__ADDR_GOAL_POSITION = 116
        self.__ADDR_GOAL_VELOCITY = 104
        self.__ADDR_PRESENT_POSITION = 132
        self.__ADDR_PRESENT_VELOCITY = 128

        self.__POSITION_MODE = 3
        self.__VELOCITY_MODE = 1
        self.__TORQUE_ENABLE = 1
        self.__TORQUE_DISABLE = 0

        # Data Byte Length
        self.__LEN_GOAL_POSITION = 4
        self.__LEN_PRESENT_POSITION = 4

        # Protocol version
        self.__PROTOCOL_VERSION = 2.0
        self.__BAUDRATE = 57600

        # Default settings for both motors
        self.__MOTORS = {
            'pitch':
                {
                    'id': 2,
                    'limits': (-10, 80),
                    'max_speed': 445 - 100,
                    'direction': 1,
                },
            'yaw':
                {
                    'id': 1,
                    'limits': (-75, 75),
                    'max_speed': 445 - 100,
                    'direction': 1,
                }
        }

        # # Public CONSTANTS:
        self.PUBLIC_CONTANT = 1

        # # Private variables:
        # Initializes the port handler and packet handler which will be used to intialize the port and motors.
        self.__device_name = '/dev/ttyUSB0'
        self.__port_handler = PortHandler(self.__device_name)
        self.__packet_handler = PacketHandler(self.__PROTOCOL_VERSION)

        # Checking flags to see if port and motors are ready, and a dictionary to check if the axis limits are reached.
        self.__port_is_ready = False
        self.__motor_is_ready = {
            'pitch': False,
            'yaw': False,
        }

        self.__limit_status = {
            'pitch': {
                'lower': False,
                'upper': False,
            },
            'yaw': {
                'lower': False,
                'upper': False,
            }
        }

        # Dictionaries used to keep track of current velocity, position and target velocity.
        self.__current_velocity_fraction = {
            'pitch': 0.0,
            'yaw': 0.0,
        }

        self.__current_position_deg = {
            'pitch': 0.0,
            'yaw': 0.0,
        }

        self.__target_velocity_fraction = {
            'pitch': 0.0,
            'yaw': 0.0,
        }

        # # Public variables:
        self.public_variable = 1

        # # Initialization and dependency status topics:
        self.__is_initialized = False
        self.__dependency_initialized = False

        # Defines __node_is_initialized publisher which will publish to the topic is_initialized if the system is ready to run or not.
        self.__node_is_initialized = rospy.Publisher(
            f'{self.__NODE_NAME}/is_initialized',
            Bool,
            queue_size=1,
        )

        self.__dependency_status = {}
        self.__dependency_status_topics = {}

        # # Publishers that publish the motors state/position in degrees to calculate the appropriate velocity by the PID controllers.
        self.__pitch_state = rospy.Publisher(
            '/pitch_motor_pid/state',
            Float64,
            queue_size=1,
        )
        self.__yaw_state = rospy.Publisher(
            '/yaw_motor_pid/state',
            Float64,
            queue_size=1,
        )

        # # Subscribers that subscribe to the control_effort topics which will indicate the target velocity.
        rospy.Subscriber(
            '/pitch_motor_pid/control_effort',
            Float64,
            self.__pitch_velocity_fraction_callback,
        )
        rospy.Subscriber(
            '/yaw_motor_pid/control_effort',
            Float64,
            self.__yaw_velocity_fraction_callback,
        )

        # # Timer used to alternate the reading of the current position of the motors and writing the target velocity to the motors. 
        rospy.Timer(
            rospy.Duration(1.0 / 200),
            self.__read_write_timer,
        )

    # # Topics Callback Functions: used to set the motors to the target velocity.

    def __pitch_velocity_fraction_callback(self, message):
        """
        Callback function of the '/pitch_motor_pid/control_effort' topic, extracts the velocity fraction calculated by the pitch PID controller, 
        checks the error between the current and target velocity, and if it's larger than a certain tolerance it sets the __current_velocity_fraction and the __target_velocity_fraction for the pitch motor.
        """

        target_velocity_fraction = message.data

        if (
            abs(
                target_velocity_fraction
                - self.__current_velocity_fraction['pitch']
            ) > 0.005
        ):
            self.__target_velocity_fraction['pitch'] = target_velocity_fraction
            self.__current_velocity_fraction['pitch'] = target_velocity_fraction

    def __yaw_velocity_fraction_callback(self, message):
        """
        Callback function of the '/yaw_motor_pid/control_effort' topic, extracts the velocity fraction calculated by the yaw PID controller, 
        checks the error between the current and target velocity, and if it's larger than a certain tolerance it sets the __current_velocity_fraction and the __target_velocity_fraction for the yaw motor.
        """

        target_velocity_fraction = message.data

        if (
            abs(
                target_velocity_fraction
                - self.__current_velocity_fraction['yaw']
            ) > 0.005
        ):
            self.__target_velocity_fraction['yaw'] = target_velocity_fraction
            self.__current_velocity_fraction['yaw'] = target_velocity_fraction

    # # Timer callback function: reads the motors current positions and writes the motors target velocity.

    def __read_write_timer(self, event):
        """
        Calls <__read_current_position> and <__set_motor_velocity> on each timer callback with 200 Hz frequency. 
        Manages when reading the motors current positions and when writing the motors target velocity.
        """

        if not self.__is_initialized or not self.__port_is_ready:
            return

        self.__read_current_position('pitch')
        self.__read_current_position('yaw')

        self.__set_motor_velocity(
            'pitch',
            self.__target_velocity_fraction['pitch'],
        )
        self.__set_motor_velocity(
            'yaw',
            self.__target_velocity_fraction['yaw'],
        )

    # # Private methods:

    def __check_initialization(self):
        """Monitors required criteria and sets is_initialized variable.

        Monitors nodes' dependency status by checking if dependency's
        is_initialized topic has at most one publisher (this ensures that
        dependency node is alive and does not have any duplicates) and that it
        publishes True. If dependency's status was True, but get_num_connections
        is not equal to 1, this means that the connection is lost and emergency
        actions should be performed.

        Once all dependencies are initialized and additional criteria met, the
        nodes' is_initialized status changes to True. This status can change to
        False any time to False if some criteria are no longer met.
        
        """

        self.__dependency_initialized = True

        for key in self.__dependency_status:
            if self.__dependency_status_topics[key].get_num_connections() != 1:
                if self.__dependency_status[key]:
                    rospy.logerr(
                        (f'{self.__NODE_NAME}: '
                         f'lost connection to {key}!')
                    )

                    # # Emergency actions on lost connection:
                    # NOTE (optionally): Add code, which needs to be executed if
                    # connection to any of dependencies was lost.

                self.__dependency_status[key] = False

            if not self.__dependency_status[key]:
                self.__dependency_initialized = False

        if not self.__dependency_initialized:
            waiting_for = ''
            for key in self.__dependency_status:
                if not self.__dependency_status[key]:
                    waiting_for += f'\n- waiting for {key} node...'

            rospy.logwarn_throttle(
                15,
                (
                    f'{self.__NODE_NAME}:'
                    f'{waiting_for}'
                    # f'\nMake sure those dependencies are running properly!'
                ),
            )

        if not self.__port_is_ready:
            self.__port_is_ready = self.__initialize_port()

        if self.__port_is_ready:
            for motor_name, motor_config in self.__MOTORS.items():
                if not self.__motor_is_ready[motor_name]:
                    self.__motor_is_ready[motor_name] = self.__initialize_motor(
                        motor_name,
                        motor_config['id'],
                    )

        # NOTE (optionally): Add more initialization criterea if needed.
        if (
            self.__dependency_initialized and self.__port_is_ready
            and self.__motor_is_ready['pitch'] and self.__motor_is_ready['yaw']
        ):
            if not self.__is_initialized:
                rospy.loginfo(f'\033[92m{self.__NODE_NAME}: ready.\033[0m',)

                self.__is_initialized = True

        else:
            if self.__is_initialized:
                # NOTE (optionally): Add code, which needs to be executed if the
                # nodes's status changes from True to False.

                pass

            self.__is_initialized = False

        self.__node_is_initialized.publish(self.__is_initialized)

    def __initialize_port(self):
        """
        Using the PortHandler instance of the class, it opens the port device and sets the specified baud rate.
        """

        # Open port.
        if not self.__port_handler.openPort():
            rospy.logerr("Failed to open port")
            rospy.signal_shutdown("Cannot open port")
            return False
        else:
            rospy.loginfo("Port opened successfully")

        # Set port baudrate.
        if not self.__port_handler.setBaudRate(self.__BAUDRATE):
            rospy.logerr("Failed to set baudrate")
            rospy.signal_shutdown("Cannot set baudrate")
            return False
        else:
            rospy.loginfo("Baudrate set successfully")

        return True

    def __initialize_motor(self, name, motor_id):
        """
        Using the PacketHandler instance of the class, it sets the operating mode to VELOCITY Mode and enables the Torque. 
        """

        rospy.loginfo(f"Initializing motor: {name} (ID: {motor_id})")

        # Set operating mode to velocity control.
        result, error = self.__packet_handler.write1ByteTxRx(
            self.__port_handler,
            motor_id,
            self.__ADDR_OPERATING_MODE,
            self.__VELOCITY_MODE,
        )
        if result != COMM_SUCCESS:
            rospy.logerr(
                f"Failed to set operating mode for motor {name}: {self.__packet_handler.getTxRxResult(result)}"
            )
            return False

        elif error != 0:
            rospy.logerr(
                f"Error setting operating mode for motor {name}: {self.__packet_handler.getRxPacketError(error)}"
            )
            # Disable torque.
            self.__packet_handler.write1ByteTxRx(
                self.__port_handler,
                motor_id,
                self.__ADDR_TORQUE_ENABLE,
                self.__TORQUE_DISABLE,
            )

            return False

        else:
            rospy.loginfo(
                f"Operating mode set to velocity control for motor {name}"
            )

        # Enable torque.
        result, error = self.__packet_handler.write1ByteTxRx(
            self.__port_handler,
            motor_id,
            self.__ADDR_TORQUE_ENABLE,
            self.__TORQUE_ENABLE,
        )

        if result != COMM_SUCCESS:
            rospy.logerr(
                f"Failed to enable torque for motor {name}: {self.__packet_handler.getTxRxResult(result)}"
            )
            return False

        elif error != 0:
            rospy.logerr(
                f"Error enabling torque for motor {name}: {self.__packet_handler.getRxPacketError(error)}"
            )
            return False

        else:
            rospy.loginfo(f"Torque enabled for motor {name}")

        return True

    def __set_motor_velocity(self, name, velocity_fraction):
        """
        Calculates the velocity to set the motors to, it writes the velocity, and checks that the axis limits are not reached.
        """

        motor = self.__MOTORS[name]
        motor_id = motor['id']
        max_speed = motor['max_speed']

        # Clamp velocity fraction.
        velocity_fraction = np.clip(velocity_fraction, -1.0, 1.0)

        # Compute velocity.
        velocity = int(max_speed * abs(velocity_fraction))
        if velocity_fraction < 0:
            velocity = -velocity

        if velocity < 0 and self.__limit_status[name]['lower']:
            velocity = 0

        if velocity > 0 and self.__limit_status[name]['upper']:
            velocity = 0

        # Write velocity.
        result, error = self.__packet_handler.write4ByteTxRx(
            self.__port_handler,
            motor_id,
            self.__ADDR_GOAL_VELOCITY,
            velocity,
        )

        if result != COMM_SUCCESS:
            rospy.logerr(
                f"Failed to set velocity for motor {name}: {self.__packet_handler.getTxRxResult(result)}"
            )
            return

    def __read_current_position(self, name):
        """
        Read the current position of the motors and converts it in degrees.
        """

        motor = self.__MOTORS[name]
        motor_id = motor['id']

        # Read current position.
        dxl_present_position, dxl_comm_result, dxl_error = (
            self.__packet_handler.read4ByteTxRx(
                self.__port_handler,
                motor_id,
                self.__ADDR_PRESENT_POSITION,
            )
        )
        if dxl_comm_result != COMM_SUCCESS:
            rospy.logerr(
                f"Failed to read position for motor {name}: {self.__packet_handler.getTxRxResult(dxl_comm_result)}"
            )
            return
        elif dxl_error != 0:
            rospy.logerr(
                f"Error reading position for motor {name}: {self.__packet_handler.getRxPacketError(dxl_error)}"
            )
            return

        # Convert position to degrees.
        position_degrees = (dxl_present_position * 360.0 / 4095.0) - 180
        self.__current_position_deg[name] = round(position_degrees, 2)

    def __check_axis_limits(self, name, current_position):
        """
        Checks that the current position of the motors is over the axis limits, if it is so, a flag is set to True and the velocity will be set to 0.
        """

        motor = self.__MOTORS[name]
        lower_limit = motor['limits'][0]
        upper_limit = motor['limits'][1]

        if (current_position > lower_limit and current_position < upper_limit):
            self.__limit_status[name]['lower'] = False
            self.__limit_status[name]['upper'] = False

        elif current_position <= lower_limit:
            self.__limit_status[name]['lower'] = True

        elif current_position >= upper_limit:
            self.__limit_status[name]['upper'] = True

    # # Public methods:

    def main_loop(self):
        """
        In a loop, once the intialization was successfull, it checks the axis limits and publish the current motors positions to the PIDs controllers.
        """

        self.__check_initialization()

        if not self.__is_initialized:
            return

        self.__check_axis_limits(
            'pitch',
            self.__current_position_deg['pitch'],
        )
        self.__check_axis_limits(
            'yaw',
            self.__current_position_deg['yaw'],
        )

        # Publish PID state:
        float64_message = Float64()
        float64_message.data = self.__current_position_deg['pitch']
        self.__pitch_state.publish(float64_message)

        float64_message = Float64()
        float64_message.data = self.__current_position_deg['yaw']
        self.__yaw_state.publish(float64_message)

    def node_shutdown(self):
        """
        Disable the motor's torque, closes the port and shut the node down.
        """

        rospy.loginfo_once(f'{self.__NODE_NAME}: node is shutting down...',)

        self.__port_is_ready = False

        rospy.sleep(1)

        for motor_name, motor_config in self.__MOTORS.items():
            motor_id = motor_config['id']
            self.__packet_handler.write1ByteTxRx(
                self.__port_handler,
                motor_id,
                self.__ADDR_TORQUE_ENABLE,
                self.__TORQUE_DISABLE,
            )
        rospy.loginfo_once(f'{self.__NODE_NAME}: motors were turned off.',)

        self.__port_handler.closePort()
        rospy.loginfo_once(f'{self.__NODE_NAME}: the port was closed.',)

        rospy.loginfo_once(f'{self.__NODE_NAME}: node has shut down.',)


def main():
    """
    Initialize the active_neck node with its frquency, initialize an ActiveNeck object, sets the rate and shuts down.
    """

    # # Default node initialization.
    # This name is replaced when a launch file is used.
    rospy.init_node(
        'active_neck',
        log_level=rospy.INFO,  # rospy.DEBUG to view debug messages.
    )

    rospy.loginfo('\n\n\n\n\n')  # Add whitespaces to separate logs.

    # # ROS launch file parameters:
    node_name = rospy.get_name()

    node_frequency = rospy.get_param(
        param_name=f'{rospy.get_name()}/node_frequency',
        default=100,
    )

    class_instance = ActiveNeck(node_name=node_name,)

    rospy.on_shutdown(class_instance.node_shutdown)
    node_rate = rospy.Rate(node_frequency)

    while not rospy.is_shutdown():
        class_instance.main_loop()
        node_rate.sleep()


if __name__ == '__main__':
    main()
