import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.subscriber_ = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.serial_port = serial.Serial('/dev/ttyUSB0', 9600)  # Adjust as necessary

    def cmd_vel_callback(self, msg):
        # Process the incoming Twist message
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        if linear_x > 0:  # Moving forward
            self.send_command('i')  # Forward (i)
        elif linear_x < 0:  # Moving backward
            self.send_command(',')  # Backward (m)
        elif angular_z > 0:  # Turning right
            if linear_x > 0:  # Forward with right turn
                self.send_command('o')  # Forward right turn (o)
            elif linear_x < 0:  # Backward with right turn
                self.send_command('.')  # Backward right turn (.)
            else:
                self.send_command('l')  # Right turn (l)
        elif angular_z < 0:  # Turning left
            if linear_x > 0:  # Forward with left turn
                self.send_command('u')  # Forward left turn (u)
            elif linear_x < 0:  # Backward with left turn
                self.send_command('m')  # Backward left turn (j)
            else:
                self.send_command('j')  # Left turn (s)
        else:  # No movement
            self.send_command('s')  # Stop (s)

    def send_command(self, command):
        self.serial_port.write(command.encode())
        self.get_logger().info(f'Sent command: {command}')

    def close_serial(self):
        self.serial_port.close()

def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()

    try:
        rclpy.spin(motor_controller)
    except KeyboardInterrupt:
        motor_controller.get_logger().info('Keyboard interrupt detected. Shutting down...')
    finally:
        motor_controller.close_serial()
        motor_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
