#!/usr/bin/env python3

from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QShortcut, QGridLayout, QHBoxLayout, QLabel, QFrame
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
        self.setGeometry(100, 100, 500, 500)
        self.ros_node = node
        
        # --- Status Rectangle ---
        self.status_rect = QFrame()
        self.status_rect.setFixedSize(200, 80)
        self.status_rect.setStyleSheet("""
            QFrame {
                background-color: #28a745;   /* green */
                border-radius: 10px;
                border: 2px solid #1e7e34;
            }
        """)

        # --- Text inside the rectangle ---
        self.status_label = QLabel("No Obstacle Detected")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet("""
            QLabel {
                color: white;
                font-size: 14px;
                font-weight: bold;
            }
        """)

        # Put the label inside the frame using a layout
        status_layout = QVBoxLayout()
        status_layout.addWidget(self.status_label, alignment=Qt.AlignCenter)
        self.status_rect.setLayout(status_layout)


        layout = QVBoxLayout()    
        layout.setSpacing(5)   # default is ~10–15, so 5 is tighter

        # --- Container for side-by-side layout ---
        hbox = QHBoxLayout()
        hbox.setAlignment(Qt.AlignCenter)
        hbox.setSpacing(20)   # space between instructions and buttons

        # ===== Instruction label =====
        instructions = QLabel(
            "======== Keyboard Control ========\n"
            "W - Forward\n"
            "S - Backward\n"
            "A - Left\n"
            "D - Right\n"
            "CTRL+C - Quit\n"
            "===================================="
        )

        instructions.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        instructions.setStyleSheet("""
            QLabel {
                font-family: Consolas, monospace;
                font-size: 12px;
            }
        """)

        hbox.addWidget(instructions)

        # ===== Directional buttons layout =====
        arrow_layout = QGridLayout()
        arrow_layout.setContentsMargins(0, 0, 0, 0)

        self.bt_forward = QPushButton("↑")
        self.bt_forward.setFixedSize(50, 50)
        arrow_layout.addWidget(self.bt_forward, 0, 1, Qt.AlignCenter)

        self.bt_turn_left = QPushButton("←")
        self.bt_turn_left.setFixedSize(50, 50)
        arrow_layout.addWidget(self.bt_turn_left, 1, 0, Qt.AlignCenter)

        self.bt_turn_right = QPushButton("→")
        self.bt_turn_right.setFixedSize(50, 50)
        arrow_layout.addWidget(self.bt_turn_right, 1, 2, Qt.AlignCenter)

        self.bt_backward = QPushButton("↓")
        self.bt_backward.setFixedSize(50, 50)
        arrow_layout.addWidget(self.bt_backward, 2, 1, Qt.AlignCenter)

        self.bt_quit = QPushButton("Quit")
        self.bt_quit.setFixedSize(80, 40)
        self.bt_quit.setStyleSheet("""
            QPushButton {
                background-color: #d9534f;
                color: white;
                border-radius: 10px;
                font-weight: bold;
            }
            QPushButton:hover { background-color: #c9302c; }
            QPushButton:pressed { background-color: #ac2925; }
        """)
        arrow_layout.addWidget(self.bt_quit, 3, 1, Qt.AlignCenter)

        # Wrap arrow pad
        arrow_widget = QWidget()
        arrow_widget.setLayout(arrow_layout)
        arrow_widget.setFixedSize(200, 220)

        hbox.addWidget(arrow_widget)

        # Add the children widgets
        layout.addWidget(self.status_rect, alignment=Qt.AlignCenter)
        layout.addSpacing(10)  # small spacing below the rectangle
        layout.addLayout(hbox)
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
        self.ros_node.publish_command("w")
        print("Publish node to Move forward")
        # self.object_status(True)          # For testing only

    def clicked_move_backward(self):
        self.ros_node.publish_command("s")
        print("Publish node to Move backward")
        # self.object_status(False)         # For testing only
    
    def clicked_turn_left(self):
        self.ros_node.publish_command("a")
        print("Publish node to Turn left")
    
    def clicked_turn_right(self):
        self.ros_node.publish_command("d")
        print("Publish node to Turn right")

    def clicked_quit(self):
        rclpy.shutdown()
        self.close()
        print("Quitting application")

    def object_status(self, isObstacleDetected = False):
        if isObstacleDetected == False:
            self.status_rect.setStyleSheet("""
                QFrame {
                    background-color: #28a745;
                    border-radius: 10px;
                    border: 2px solid #1e7e34;
                }
            """)
            self.status_label.setText("No Obstacle Detected")
            self.bt_forward.setEnabled(True)
            
        elif isObstacleDetected ==  True:
            self.status_rect.setStyleSheet("""
                QFrame {
                    background-color: #dc3545;
                    border-radius: 10px;
                    border: 2px solid #b21f2d;
                }
            """)
            self.status_label.setText("Obstacle Detected")
            self.bt_forward.setEnabled(False)


def main(args=None):
    rclpy.init(args=args)

    node = MotorSerialNode()

    # Run ROS spinning in background thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    # Start GUI in main thread
    app = QApplication(sys.argv)
    window = Control_GUI(node)
    window.show()

    try:
        sys.exit(app.exec_())
    finally:
        rclpy.shutdown()
        if node.arduino:
            node.arduino.close()
        node.destroy_node()


if __name__ == '__main__':
    main()