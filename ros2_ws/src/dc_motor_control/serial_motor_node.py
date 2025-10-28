import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
import serial
import serial.tools.list_ports
import time
import threading

class SerialMotorController(Node):
    """
    ROS 2 Node to send motor commands to an Arduino via serial port.
    It subscribes to a topic and forwards the Int8 value as a character 
    over the serial connection. It also reads confirmation messages.
    """
    def __init__(self):
        super().__init__('serial_motor_controller')
        
        # Default parameters for the CH340 chip on Linux/Raspberry Pi
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 9600)
        
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        
        # Mapping ROS Int8 command (0-4) to Arduino char ('s', 'f', 'b', 'l', 'r')
        self.command_map = {
            0: 's',  # Stop
            1: 'f',  # Forward
            2: 'b',  # Backward
            3: 'l',  # Left (assuming 'l' is full speed for both motors on the left ports)
            4: 'r'   # Right (assuming 'r' is reverse speed for both motors on the right ports)
        }
        
        self.ser = None
        self._initialize_serial()

        if self.ser and self.ser.is_open:
            # Start a separate thread to continuously read data from the Arduino
            self.reader_thread = threading.Thread(target=self._serial_reader, daemon=True)
            self.reader_thread.start()

            # ROS 2 Subscriber: listens for motor commands
            self.subscription = self.create_subscription(
                Int8,
                'cmd_motor',
                self.command_callback,
                10
            )
            self.get_logger().info(f'Subscribed to topic: /cmd_motor. Ready to send commands. Valid commands: {list(self.command_map.keys())}')

    def _initialize_serial(self):
        """Initializes the serial connection to the Arduino and waits for READY."""
        try:
            # Initialize serial connection
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1)
            # Give the Arduino time to reset after connection (common for AVR boards)
            time.sleep(2) 
            self.get_logger().info(f'Successfully connected to Arduino on {self.serial_port} at {self.baud_rate} baud.')
            
            # Wait for the 'READY' confirmation from the Arduino
            self._wait_for_ready()

        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port {self.serial_port}: {e}')
            self.get_logger().error('Ensure the Arduino is plugged in and the port name is correct.')
            self.ser = None
        
    def _wait_for_ready(self):
        """Waits for the 'READY' string sent by the Arduino upon boot."""
        self.ser.flushInput()
        self.get_logger().info("Waiting for 'READY' confirmation from Arduino...")
        start_time = time.time()
        
        while rclpy.ok() and (time.time() - start_time < 10): # Timeout after 10 seconds
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            if line == "READY":
                self.get_logger().info("Arduino Ready: READY. Communication established.")
                return
            time.sleep(0.1) # Don't spin too fast

        if rclpy.ok():
            self.get_logger().warn("Timed out waiting for 'READY'. Proceeding anyway, but check Arduino sketch and baud rate.")

    def _serial_reader(self):
        """Continuously reads data from the serial port in a separate thread."""
        while rclpy.ok() and self.ser and self.ser.is_open:
            try:
                # Read line from Arduino
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    # Print any incoming data (like the echo or status messages)
                    self.get_logger().info(f"Arduino Echo: {line}")
                
            except serial.SerialException as e:
                self.get_logger().error(f"Serial Read Error: {e}")
                break
            except Exception as e:
                self.get_logger().error(f"Unexpected error in serial reader thread: {e}")
                break
            time.sleep(0.01) # Small delay to yield to other threads

    def command_callback(self, msg):
        """Receives an Int8 command, maps it to a character, and sends it over serial."""
        command_int = msg.data
        
        if command_int in self.command_map:
            command_char = self.command_map[command_int]
            char_to_send = command_char.encode('ascii')
            
            if self.ser and self.ser.is_open:
                try:
                    # Send the single character command (e.g., b'f')
                    self.ser.write(char_to_send)
                    self.get_logger().info(f'Sent command: {command_int} (byte: {char_to_send.decode()})')
                except serial.SerialTimeoutException:
                    self.get_logger().warn('Serial write timeout.')
                except serial.SerialException as e:
                    self.get_logger().error(f'Error writing to serial: {e}. Closing port.')
                    self.ser.close()
            else:
                self.get_logger().error('Serial port is not open.')
        else:
            self.get_logger().warn(f'Received invalid command: {command_int}. Must be one of {list(self.command_map.keys())}.')

def main(args=None):
    rclpy.init(args=args)
    motor_controller = SerialMotorController()
    
    # Spin the node so the callback function is called when a message is received
    try:
        rclpy.spin(motor_controller)
    except KeyboardInterrupt:
        pass
    finally:
        # Close serial port and shutdown ROS 2
        if motor_controller.ser and motor_controller.ser.is_open:
            motor_controller.ser.close()
            motor_controller.get_logger().info('Serial port closed.')
        motor_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
