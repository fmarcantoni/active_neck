import rospy
from dynamixel_sdk import *  # Uses Dynamixel SDK
from std_msgs.msg import Float64

# Control table addresses
ADDR_TORQUE_ENABLE = 64
ADDR_OPERATING_MODE = 11
ADDR_PROFILE_VELOCITY = 104

# Protocol version
PROTOCOL_VERSION = 2.0

# Motor settings
MOTORS = {
    'pitch': {
        'id': 1,
        'max_speed': 400,
    },
    'yaw': {
        'id': 2,
        'max_speed': 400,
    }
}

BAUDRATE = 57600
DEVICE_NAME = '/dev/ttyUSB0'
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
VELOCITY_MODE = 1


class VelocityController:

    def __init__(self):
        rospy.init_node('velocity_controller', anonymous=True)

        # Initialize PortHandler and PacketHandler
        self.port_handler = PortHandler(DEVICE_NAME)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

        # Open port
        if not self.port_handler.openPort():
            rospy.logerr("Failed to open port")
            rospy.signal_shutdown("Cannot open port")
        else:
            rospy.loginfo("Port opened successfully")

        # Set port baudrate
        if not self.port_handler.setBaudRate(BAUDRATE):
            rospy.logerr("Failed to set baudrate")
            rospy.signal_shutdown("Cannot set baudrate")
        else:
            rospy.loginfo("Baudrate set successfully")

        # Initialize motors
        for motor_name, motor_config in MOTORS.items():
            self.initialize_motor(
                motor_name, motor_config['id'], motor_config['max_speed']
            )

        # Subscribers for velocity
        self.pitch_sub = rospy.Subscriber(
            '/dynamixel/pitch_velocity', Float64, self.set_pitch_velocity
        )
        self.yaw_sub = rospy.Subscriber(
            '/dynamixel/yaw_velocity', Float64, self.set_yaw_velocity
        )

    def initialize_motor(self, name, motor_id, max_speed):
        rospy.loginfo(f"Initializing motor: {name} (ID: {motor_id})")

        # Set operating mode to velocity control
        result, error = self.packet_handler.write1ByteTxRx(
            self.port_handler,
            motor_id,
            ADDR_OPERATING_MODE,
            VELOCITY_MODE,
        )
        if result != COMM_SUCCESS:
            rospy.logerr(
                f"Failed to set operating mode for motor {name}: {self.packet_handler.getTxRxResult(result)}"
            )
        elif error != 0:
            rospy.logerr(
                f"Error setting operating mode for motor {name}: {self.packet_handler.getRxPacketError(error)}"
            )
        else:
            rospy.loginfo(
                f"Operating mode set to velocity control for motor {name}"
            )

        # Enable torque
        result, error = self.packet_handler.write1ByteTxRx(
            self.port_handler,
            motor_id,
            ADDR_TORQUE_ENABLE,
            TORQUE_ENABLE,
        )
        if result != COMM_SUCCESS:
            rospy.logerr(
                f"Failed to enable torque for motor {name}: {self.packet_handler.getTxRxResult(result)}"
            )
        elif error != 0:
            rospy.logerr(
                f"Error enabling torque for motor {name}: {self.packet_handler.getRxPacketError(error)}"
            )
        else:
            rospy.loginfo(f"Torque enabled for motor {name}")

    def set_pitch_velocity(self, msg):
        self.set_motor_velocity('pitch', msg.data)

    def set_yaw_velocity(self, msg):
        self.set_motor_velocity('yaw', msg.data)

    def set_motor_velocity(self, name, velocity_fraction):
        motor = MOTORS[name]
        motor_id = motor['id']
        max_speed = motor['max_speed']

        # Clamp velocity fraction
        velocity_fraction = max(min(velocity_fraction, 1.0), -1.0)

        # Compute velocity
        velocity = int(max_speed * abs(velocity_fraction))
        if velocity_fraction < 0:
            velocity = -velocity

        rospy.loginfo(velocity)

        # Write velocity
        result, error = self.packet_handler.write4ByteTxRx(
            self.port_handler,
            motor_id,
            ADDR_PROFILE_VELOCITY,
            abs(velocity),
        )
        if result != COMM_SUCCESS:
            rospy.logerr(
                f"Failed to set velocity for motor {name}: {self.packet_handler.getTxRxResult(result)}"
            )
        elif error != 0:
            rospy.logerr(
                f"Error setting velocity for motor {name}: {self.packet_handler.getRxPacketError(error)}"
            )
        else:
            rospy.loginfo(
                f"{name.capitalize()} motor velocity set to {velocity} ({velocity_fraction * 100}% of max)"
            )

    def shutdown(self):
        rospy.loginfo("Shutting down motors")
        for motor_name, motor_config in MOTORS.items():
            motor_id = motor_config['id']
            self.packet_handler.write1ByteTxRx(
                self.port_handler, motor_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE
            )
        self.port_handler.closePort()


if __name__ == '__main__':
    try:
        controller = VelocityController()
        rospy.on_shutdown(controller.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
