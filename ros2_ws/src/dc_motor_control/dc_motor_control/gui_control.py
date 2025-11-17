#!/usr/bin/env python3

from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QShortcut
from PyQt5.QtGui import QKeySequence
from PyQt5.QtCore import Qt
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


class Control_GUI(QWidget):
    def __init__(self, node):
        super().__init__()
        self.setWindowTitle("Robotic Control")
        self.setGeometry(100, 100, 300, 200)
    
        print("\n=== W,A,S,D Keyboard Control Active ===")
        print("W - Forward")
        print("S - Backward")
        print("A - Left")
        print("D - Right")
        print("Q - Quit")
        print("====================================\n")
    
        layout = QVBoxLayout()    
        # Buttons declaration
        self.bt_forward = QPushButton("W")
        layout.addWidget(self.bt_forward)
        self.bt_backward = QPushButton("S")
        layout.addWidget(self.bt_backward)
        self.bt_turn_left = QPushButton("A")
        layout.addWidget(self.bt_turn_left)
        self.bt_turn_right = QPushButton("D")
        layout.addWidget(self.bt_turn_right)
        self.bt_quit = QPushButton("CTRL + C")
        layout.addWidget(self.bt_quit)
        self.setLayout(layout)

        # Connect the button's clicked signal to a slot
        self.bt_forward.clicked.connect(self.clicked_move_forward)
        self.bt_backward.clicked.connect(self.clicked_move_backward)
        self.bt_turn_left.clicked.connect(self.clicked_turn_left)
        self.bt_turn_right.clicked.connect(self.clicked_turn_right)
        self.bt_quit.clicked.connect(self.clicked_quit)

        # Create a QShortcut
        self.shortcut_w = QShortcut(Qt.Key_W, self)
        self.shortcut_s = QShortcut(Qt.Key_S, self)
        self.shortcut_a = QShortcut(Qt.Key_A, self)
        self.shortcut_d = QShortcut(Qt.Key_D, self)
        self.shortcut_quit = QShortcut(QKeySequence(Qt.CTRL + Qt.Key_C), self)

        # Connect the activated signal of the shortcut to the button's click method
        self.shortcut_w.activated.connect(self.bt_forward.click)
        self.shortcut_s.activated.connect(self.bt_backward.click)
        self.shortcut_a.activated.connect(self.bt_turn_left.click)
        self.shortcut_d.activated.connect(self.bt_turn_right.click)
        self.shortcut_quit.activated.connect(self.bt_quit.click)

    def clicked_move_forward(self):
        node.publish_command("w")
        print("Publish node to Move forward")

    def clicked_move_backward(self):
        node.publish_command("s")
        print("Publish node to Move backward")
    
    def clicked_turn_left(self):
        node.publish_command("a")
        print("Publish node to Turn left")
    
    def clicked_turn_right(self):
        node.publish_command("d")
        print("Publish node to Turn right")

    def clicked_quit(self):
        rclpy.shutdown()
        self.close()
        print("Quitting application")

def main(args=None):
    rclpy.init(args=args)
    node = MotorSerialNode()
    
    # Launch GUI
    while rclpy.ok():
        try:
            app = QApplication(sys.argv)
            window = Control_GUI(node)
            window.show()
            sys.exit(app.exec_())

            try:
                rclpy.spin(node)
            except KeyboardInterrupt:
                pass
            finally:
                if node.arduino:
                    node.arduino.close()
                node.destroy_node()
                rclpy.shutdown()
        except Exception as e:
            print(f"Error in keyboard thread: {e}")
            break

if __name__ == '__main__':
    main()
