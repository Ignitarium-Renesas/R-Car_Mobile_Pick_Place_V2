import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from encoder_msgs.msg import EncoderData  # Import custom message
import serial
import time
import re


class MotorController(Node):
    def __init__(self):
        super().__init__("motor_controller")

        # Subscribe to cmd_vel topic
        self.subscription = self.create_subscription(
            Twist, "cmd_vel", self.cmd_vel_callback, 10
        )

        # Publisher for encoder data
        self.encoder_publisher = self.create_publisher(EncoderData, "encoder_data", 10)

        # Open serial port
        self.serial_port = serial.Serial(
            "/dev/ttyUSB0", 115200, timeout=2
        )  # Adjust port as needed

        # Robot parameters
        self.wheel_separation = 0.5  # Distance between left and right wheels
        self.max_motor_speed = 500  # Max motor speed (adjust as needed)

        # Timer for detecting command timeout
        self.last_cmd_time = time.time()

        # Timer to publish encoder data
        self.create_timer(0.1, self.publish_encoder_data)  # Publish every 100ms

    def cmd_vel_callback(self, msg):
        """Callback function for cmd_vel messages"""
        self.last_cmd_time = time.time()  # Update last received time

        linear_x = msg.linear.x
        if -0.015 < linear_x < 0:
            linear_x = -0.015
        elif 0 < linear_x < 0.015:
            linear_x = 0.015

        angular_z = msg.angular.z

        # Skid-steer calculations
        left_speed = linear_x - (angular_z * self.wheel_separation / 2)
        right_speed = linear_x + (angular_z * self.wheel_separation / 2)

        # Scale speed to motor range
        left_motor_speed = int(left_speed * 359)
        right_motor_speed = int(right_speed * 359)

        # Ensure values are within valid range
        left_motor_speed = max(
            -self.max_motor_speed, min(self.max_motor_speed, left_motor_speed)
        )
        right_motor_speed = max(
            -self.max_motor_speed, min(self.max_motor_speed, right_motor_speed)
        )

        # Send motor command
        command = f"$FR:{right_motor_speed},FL:{left_motor_speed},BR:{right_motor_speed},BL:{left_motor_speed}#"
        self.serial_port.write(command.encode())

        self.get_logger().info(f"Sent command: {command.strip()}")

    def check_timeout(self):
        """Check if cmd_vel message has timed out, stop motors if needed"""
        if time.time() - self.last_cmd_time > 1.0:  # No message for 1 sec
            stop_command = "$FR:0,FL:0,BR:0,BL:0#\n"  # Stop motors
            self.serial_port.write(stop_command.encode())
            self.get_logger().warn("No cmd_vel received, stopping motors.")

    def publish_encoder_data(self):
        """Read encoder data from serial and publish it"""
        if self.serial_port.in_waiting > 0:
            raw_data = self.serial_port.readline().decode("utf-8").strip()
            self.get_logger().warn("serial_data_reader")
            self.get_logger().info(f"data {raw_data}")
            match = re.match(
                r"\$BR:(-?\d+),BL:(-?\d+),FL:(-?\d+),FR:(-?\d+)#", raw_data
            )

            if match:
                fr = float(match.group(1))
                fl = float(match.group(2))
                br = float(match.group(3))
                bl = float(match.group(4))
                # Publish encoder data
                encoder_msg = EncoderData()
                encoder_msg.enc_left_front = br  #
                encoder_msg.enc_left_back = fl  #
                encoder_msg.enc_right_front = bl
                encoder_msg.enc_right_back = fr
                self.encoder_publisher.publish(encoder_msg)

                self.get_logger().info(f"Published encoder data: Left={encoder_msg}")
            else:
                self.get_logger().warn("string not mathched")


def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
