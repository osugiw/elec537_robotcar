import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial
import re

class UltrasonicPublisher(Node):
    def __init__(self):
        super().__init__('ultrasonic_publisher')
        self.publisher_ = self.create_publisher(Float32, 'ultrasonic_distance', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        try:
            self.serial_port = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
            self.get_logger().info("Connected to Arduino on /dev/ttyUSB0")
        except serial.SerialException:
            self.get_logger().error("Could not open serial port /dev/ttyUSB0")
            self.serial_port = None

    def timer_callback(self):
        if self.serial_port and self.serial_port.in_waiting > 0:
            try:
                line = self.serial_port.readline().decode(errors='ignore').strip()
                if line:
                    match = re.search(r'Distance:(\d+)', line)
                    if match:
                        distance = float(match.group(1))
                        msg = Float32()
                        msg.data = distance
                        self.publisher_.publish(msg)
                        self.get_logger().info(f"Published distance: {distance} cm")
                    else:
                        self.get_logger().warn(f"Unexpected line: {line}")
            except Exception as e:
                self.get_logger().error(f"Serial read error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
