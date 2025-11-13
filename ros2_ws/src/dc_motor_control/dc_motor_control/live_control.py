#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import tty
import termios
import select

class KeyboardController(Node):
    def __init__(self):
        super().__init__('keyboard_controller')
        self.publisher = self.create_publisher(String, 'motor_cmd', 10)
        self.get_logger().info('Keyboard Controller Started')
        self.get_logger().info('Controls: W=Forward, S=Backward, A=Left, D=Right, Q=Quit')
        
    def publish_command(self, command):
        msg = String()
        msg.data = command
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {command}')

def get_key():
    """Get a single keypress from the terminal"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        if select.select([sys.stdin], [], [], 0.1)[0]:
            key = sys.stdin.read(1)
        else:
            key = ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

def main(args=None):
    rclpy.init(args=args)
    controller = KeyboardController()
    
    print("\n=== RC Car Keyboard Controller ===")
    print("W - Forward")
    print("S - Backward")
    print("A - Left")
    print("D - Right")
    print("Q - Quit (or Ctrl+C)")
    print("==================================\n")
    
    try:
        while rclpy.ok():
            key = get_key()
            
            # Check for Ctrl+C (ASCII value 3)
            if key == '\x03':  # Ctrl+C
                print("\nCtrl+C detected, quitting...")
                break
            elif key.lower() == 'q':
                print("\nQuitting...")
                break
            elif key.lower() in ['w', 'a', 's', 'd']:
                controller.publish_command(key.lower())
            
            rclpy.spin_once(controller, timeout_sec=0)
    
    except KeyboardInterrupt:
        print("\nKeyboard interrupt detected")
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        # Make sure terminal is restored
        try:
            fd = sys.stdin.fileno()
            termios.tcsetattr(fd, termios.TCSADRAIN, termios.tcgetattr(fd))
        except:
            pass
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
