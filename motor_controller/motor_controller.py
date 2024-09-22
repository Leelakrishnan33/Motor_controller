import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',  # Default topic for teleop
            self.cmd_callback,
            10
        )

        self.serial_port = None  # Initialize the serial port variable
        self.initialize_serial()  # Attempt to initialize the serial port

        # Stop motors initially
        self.stop_motors()

    def initialize_serial(self):
        try:
            self.serial_port = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)  # Adjust as needed
            time.sleep(2)  # Wait for the connection to establish
        except serial.SerialException as e:
            self.get_logger().error(f'Serial port error: {e}')
            self.serial_port = None  # Ensure serial_port is None on error

    def cmd_callback(self, msg):
        if self.serial_port is not None:  # Check if serial port is open
            if msg.linear.x > 0:  # Move forward
                self.move_forward()
            elif msg.linear.x < 0:  # Move backward
                self.move_backward()
            elif msg.angular.z > 0:  # Move right
                self.move_right()
            elif msg.angular.z < 0:  # Move left
                self.move_left()
            else:  # Stop motors
                self.stop_motors()

    def move_forward(self):
        self.get_logger().info('Moving forward')
        self.serial_port.write(b'w')

    def move_backward(self):
        self.get_logger().info('Moving backward')
        self.serial_port.write(b's')

    def move_left(self):
        self.get_logger().info('Moving left')
        self.serial_port.write(b'a')

    def move_right(self):
        self.get_logger().info('Moving right')
        self.serial_port.write(b'd')

    def stop_motors(self):
        self.get_logger().info('Stopping motors')
        if self.serial_port is not None:  # Check before writing
            self.serial_port.write(b'x')

    def __del__(self):
        if self.serial_port is not None:  # Check if it was initialized
            self.serial_port.close()  # Close the serial port

def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()
    rclpy.spin(motor_controller)
    motor_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
