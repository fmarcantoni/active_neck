import rospy
from dynamixel_sdk import *  # Uses Dynamixel SDK
from std_msgs.msg import Float64

# Control table address
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132
ADDR_OPERATING_MODE = 11
ADDR_PROFILE_VELOCITY = 112  # Address for setting profile velocity

# Data Byte Length
LEN_GOAL_POSITION = 4
LEN_PRESENT_POSITION = 4

# Protocol version
PROTOCOL_VERSION = 2.0

# Default settings for both motors
MOTORS = {
    'pitch':
        {
            'id': 2,
            'limits': (-20, 90),
            'speed': int(512 / 2),
            'direction': 1,
        },
    'yaw':
        {
            'id': 1,
            'limits': (-90, 90),
            'speed': int(512 / 2),
            'direction': 1,
        }
}

BAUDRATE = 57600  # Dynamixel default baudrate
device_name = '/dev/ttyUSB0'  # Port name
torque_enable = 1  # Value to enable the torque
torque_disable = 0  # Value to disable the torque
operating_mode = 3  # Position Control Mode
MAX_POSITION = 4095  # Maximum position value for 12-bit resolution
POSITION_DEGREES_RATIO = 360.0 / MAX_POSITION  # Conversion ratio from position to degrees


class DynamixelController:

    def __init__(self):
        rospy.init_node('dynamixel_controller', anonymous=True)

        # Initialize PortHandler instance
        self.portHandler = PortHandler(device_name)

        # Initialize PacketHandler instance
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        # Open port
        if self.portHandler.openPort():
            rospy.loginfo("Succeeded to open the port")
        else:
            rospy.logerr("Failed to open the port")
            rospy.signal_shutdown("Failed to open port")

        # Set port baudrate
        if self.portHandler.setBaudRate(BAUDRATE):
            rospy.loginfo("Succeeded to set the baudrate")
        else:
            rospy.logerr("Failed to set the baudrate")
            rospy.signal_shutdown("Failed to set baudrate")

        # Initialize both motors
        for axis, motor in MOTORS.items():
            self.initialize_motor(motor['id'], motor['speed'])

        # Subscribers for each motor
        self.pitch_subscriber = rospy.Subscriber(
            '/dynamixel/pitch', Float64, self.set_pitch_position
        )
        self.yaw_subscriber = rospy.Subscriber(
            '/dynamixel/yaw', Float64, self.set_yaw_position
        )

    def initialize_motor(self, motor_id, speed):
        # Set operating mode
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
            self.portHandler, motor_id, ADDR_OPERATING_MODE, operating_mode
        )
        if dxl_comm_result != COMM_SUCCESS:
            rospy.logerr(self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            rospy.logerr(self.packetHandler.getRxPacketError(dxl_error))
        else:
            rospy.loginfo(
                f"Operating mode set to position control for motor {motor_id}"
            )

        # Set profile velocity
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
            self.portHandler, motor_id, ADDR_PROFILE_VELOCITY, speed
        )
        if dxl_comm_result != COMM_SUCCESS:
            rospy.logerr(self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            rospy.logerr(self.packetHandler.getRxPacketError(dxl_error))
        else:
            rospy.loginfo(f"Velocity set to {speed} for motor {motor_id}")

        # Enable Dynamixel Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
            self.portHandler, motor_id, ADDR_TORQUE_ENABLE, torque_enable
        )
        if dxl_comm_result != COMM_SUCCESS:
            rospy.logerr(self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            rospy.logerr(self.packetHandler.getRxPacketError(dxl_error))
        else:
            rospy.loginfo(f"Dynamixel torque enabled for motor {motor_id}")

    def set_pitch_position(self, msg):
        self.set_motor_position('pitch', msg.data)

    def set_yaw_position(self, msg):
        self.set_motor_position('yaw', msg.data)

    def set_motor_position(self, axis, angle):
        motor = MOTORS[axis]

        # Clamp angle to motor limits
        clamped_angle = max(min(angle, motor['limits'][1]), motor['limits'][0])

        # Convert degrees to Dynamixel units
        goal_position = int((clamped_angle + 180) / POSITION_DEGREES_RATIO
                           ) * motor['direction']

        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
            self.portHandler, motor['id'], ADDR_GOAL_POSITION, goal_position
        )
        if dxl_comm_result != COMM_SUCCESS:
            rospy.logerr(self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            rospy.logerr(self.packetHandler.getRxPacketError(dxl_error))
        else:
            rospy.loginfo(
                f"{axis.capitalize()} position set to {clamped_angle} degrees ({goal_position} units)"
            )

    def shutdown(self):
        # Disable Dynamixel Torque for all motors
        for motor in MOTORS.values():
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
                self.portHandler, motor['id'], ADDR_TORQUE_ENABLE,
                torque_disable
            )
            if dxl_comm_result != COMM_SUCCESS:
                rospy.logerr(self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                rospy.logerr(self.packetHandler.getRxPacketError(dxl_error))
            else:
                rospy.loginfo(
                    f"Dynamixel torque disabled for motor {motor['id']}"
                )

        # Close port
        self.portHandler.closePort()


if __name__ == '__main__':
    try:
        controller = DynamixelController()
        rospy.on_shutdown(controller.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
