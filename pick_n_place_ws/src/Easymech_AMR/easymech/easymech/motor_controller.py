import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time
import threading
from std_msgs.msg import Int32MultiArray
import argparse


class MotorController(Node):
    def __init__(self, enable_robo=True):
        super().__init__('motor_controller')

        self.enable_robo = enable_robo  # Whether to send serial commands

        # Subscribe to cmd_vel topic
        self.subscription = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # Publisher for encoder values
        self.encoder_publisher = self.create_publisher(Int32MultiArray, 'encoder_data', 10)

        # Robot parameters
        self.wheel_separation = 0.5  # Distance between left and right wheels
        self.max_motor_speed = 100   # 100 for testing [max is 255]
        self.max_linear_speed = 100

        # Timer for detecting command timeout
        self.last_cmd_time = time.time()
        self.create_timer(0.1, self.check_timeout)  # Run every 100ms (0.1 sec)

        # Open serial port only if robo mode is enabled
        if self.enable_robo:
            try:
                self.serial_port = serial.Serial('/dev/ttyACM0', 9600, timeout=1)  # Adjust port as needed
                self.encoder_thread = threading.Thread(target=self.read_encoder_data, daemon=True)
                self.encoder_thread.start()
            except serial.SerialException:
                self.get_logger().error("Failed to open serial port. Running in non-hardware mode.")
                self.enable_robo = False  # Switch to non-hardware mode

        # Timer to publish fake encoder values when robot is disabled
        if not self.enable_robo:
            self.create_timer(0.1, self.publish_dummy_encoder_data)

    def cmd_vel_callback(self, msg):
        """ Callback function for cmd_vel messages """
        self.last_cmd_time = time.time()  # Update last received time

        linear_x = msg.linear.x  # Desired linear velocity (m/s)
        angular_z = msg.angular.z  # Desired angular velocity (rad/s)

        # Skid-steer calculations
        left_speed = linear_x - (angular_z * self.wheel_separation / 2)
        right_speed = linear_x + (angular_z * self.wheel_separation / 2)

        # Scale speed to motor range (-255 to 255) using max linear speed
        left_motor_speed = int((left_speed / self.max_linear_speed) * self.max_motor_speed)
        right_motor_speed = int((right_speed / self.max_linear_speed) * self.max_motor_speed)

        # Ensure values are within valid range
        left_motor_speed = max(-self.max_motor_speed, min(self.max_motor_speed, left_motor_speed))
        right_motor_speed = max(-self.max_motor_speed, min(self.max_motor_speed, right_motor_speed))

        if self.enable_robo:
            command = f"FL{left_motor_speed} BL{left_motor_speed} FR{right_motor_speed} BR{right_motor_speed}\n"
            self.serial_port.write(command.encode())
            self.get_logger().info(f'Sent command: {command.strip()}')
        else:
            self.get_logger().info(f"Robo mode disabled, received cmd_vel: FL{left_motor_speed}, FR{right_motor_speed}")

    def read_encoder_data(self):
        """ Continuously read encoder data from the serial port and publish to ROS2 """
        while rclpy.ok() and self.enable_robo:
            try:
                if self.serial_port.in_waiting > 0:
                    data = self.serial_port.readline().decode('utf-8').strip()
                    if data.startswith("ENC"):  # Expected format: ENC FL:<val> BL:<val> FR:<val> BR:<val>
                        parts = data.split()
                        encoders = [int(part.split(':')[1]) for part in parts[1:]]

                        # Publish encoder ticks
                        msg = Int32MultiArray()
                        msg.data = encoders
                        self.encoder_publisher.publish(msg)

                        self.get_logger().info(f'Published encoder ticks: {msg.data}')
            except Exception as e:
                self.get_logger().error(f"Error reading encoder data: {str(e)}")

    def publish_dummy_encoder_data(self):
        """ Publishes zero values when no real encoder data is available """
        msg = Int32MultiArray()
        msg.data = [0, 0, 0, 0]  # Default to zeros for FL, BL, FR, BR
        self.encoder_publisher.publish(msg)
        self.get_logger().info("Published dummy encoder data: [0, 0, 0, 0]")

    def check_timeout(self):
        """ Check if cmd_vel message has timed out, stop motors if needed """
        if time.time() - self.last_cmd_time > 1.0 and self.enable_robo:  # No message for 1 sec
            stop_command = "FL0 BL0 FR0 BR0\n"  # Stop motors
            self.serial_port.write(stop_command.encode())
            self.get_logger().warn("No cmd_vel received, stopping motors.")

def main(args=None):
    rclpy.init(args=args)

    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="Motor Controller for ROS2")
    parser.add_argument('--robo', type=bool, default=True, help="Enable or disable robot communication (default: True)")
    parsed_args, _ = parser.parse_known_args()

    node = MotorController(enable_robo=parsed_args.robo)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
