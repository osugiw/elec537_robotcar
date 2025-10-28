import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MotorCommandPublisher(Node):
    def __init__(self):
        super().__init__('motor_cmd_publisher')
        self.publisher_ = self.create_publisher(String, 'motor_cmd', 10)
        self.get_logger().info("Motor Command Publisher started.")
        self.run()

    def run(self):
        try:
            while rclpy.ok():
                cmd = input("Enter command (f/b/l/r/stop/exit): ").strip().lower()
                if cmd == "exit":
                    self.get_logger().info("Exiting publisher...")
                    break
                elif cmd in ['f', 'b', 'l', 'r']:
                    msg = String()
                    msg.data = cmd
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"Published: '{cmd}'")
                else:
                    self.get_logger().warn("Invalid command. Use f/b/l/r or exit.")
        except KeyboardInterrupt:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = MotorCommandPublisher()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
