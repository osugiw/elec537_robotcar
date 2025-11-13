#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time
import sys
import tty
import termios
import threading

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
        
        # Publisher for keyboard input
        self.publisher = self.create_publisher(String, 'motor_cmd', 10)
        
    def command_callback(self, msg):
        if self.arduino is None:
            self.get_logger().warn("Arduino not connected.")
            return
        cmd = msg.data.strip().lower()
        if cmd in ['w', 's', 'a', 'd']:
            self.arduino.write(cmd.encode())
            self.get_logger().info(f"Sent command: '{cmd}' to Arduino.")
        else:
            self.get_logger().warn(f"Invalid command '{cmd}' ignored.")
    
    def publish_command(self, command):
        msg = String()
        msg.data = command
        self.publisher.publish(msg)

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def keyboard_thread(node, settings):
    """Thread to handle keyboard input"""
    print("\n=== WASD Keyboard Control Active ===")
    print("W - Forward")
    print("S - Backward")
    print("A - Left")
    print("D - Right")
    print("Q - Quit")
    print("====================================\n")
    
    while rclpy.ok():
        try:
            key = get_key(settings)
            
            if key == '\x03' or key.lower() == 'q':  # Ctrl+C or Q
                print("\nQuitting...")
                rclpy.shutdown()
                break
            elif key.lower() in ['w', 'a', 's', 'd']:
                node.publish_command(key.lower())
        except Exception as e:
            print(f"Error in keyboard thread: {e}")
            break

def main(args=None):
    rclpy.init(args=args)
    node = MotorSerialNode()
    
    # Save terminal settings
    settings = termios.tcgetattr(sys.stdin)
    
    # Start keyboard input thread
    kb_thread = threading.Thread(target=keyboard_thread, args=(node, settings), daemon=True)
    kb_thread.start()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        if node.arduino:
            node.arduino.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
