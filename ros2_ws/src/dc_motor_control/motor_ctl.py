import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time

class MotorSerialNode(Node):
    def __init__(self):
        super().__init__('motor_serial_node')

        # Parameters (port and baudrate)
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 9600)

        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

        try:
            self.arduino = serial.Serial(port, baudrate, timeout=1)
            time.sleep(2)  # wait for Arduino reset
            self.get_logger().info(f"Connected to Arduino on {port} at {baudrate} baud.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            self.arduino = None

        # Subscribe to motor command topic
        self.subscription = self.create_subscription(
            String,
            'motor_cmd',
            self.command_callback,
            10
        )

    def command_callback(self, msg):
        if self.arduino is None:
            self.get_logger().warn("Arduino not connected.")
            return

        cmd = msg.data.strip().lower()
        if cmd in ['f', 'b', 'l', 'r']:
            self.arduino.write(cmd.encode())
            self.get_logger().info(f"Sent command: '{cmd}' to Arduino.")
        else:
            self.get_logger().warn(f"Invalid command '{cmd}' ignored.")

def main(args=None):
    rclpy.init(args=args)
    node = MotorSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.arduino:
            node.arduino.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
