import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Header
import serial
import threading
import queue
import re
from math import pi


class IMUSerialPublisher(Node):
    def __init__(self):
        super().__init__("imu_serial_publisher")

        # Parameters
        self.declare_parameter("port", "/dev/ttyACM1")
        self.declare_parameter("baudrate", 115200)

        port = self.get_parameter("port").get_parameter_value().string_value
        baudrate = self.get_parameter("baudrate").get_parameter_value().integer_value

        # Serial & thread setup
        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.1)
            self.get_logger().info(f"Opened serial port {port} at {baudrate}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            return

        self.line_queue = queue.Queue()
        self.keep_reading = True
        self.reader_thread = threading.Thread(target=self.serial_reader)
        self.reader_thread.start()

        # Publishers
        self.imu_pub = self.create_publisher(Imu, "imu/data_raw", 10)
        self.mag_pub = self.create_publisher(MagneticField, "imu/mag", 10)

        self.timer = self.create_timer(0.02, self.process_lines)  # 50 Hz

        self.pattern = re.compile(
            r"Scaled\. Acc \(mg\) \[ *([-\d.]+), *([-\d.]+), *([-\d.]+) \], Gyr \(DPS\) \[ *([-\d.]+), *([-\d.]+), *([-\d.]+) \], Mag \(uT\) \[ *([-\d.]+), *([-\d.]+), *([-\d.]+) \], Tmp \(C\) \[ *([-\d.]+) \]"
        )

    def serial_reader(self):
        buffer = ""
        while self.keep_reading:
            try:
                chunk = self.ser.read(self.ser.in_waiting or 1).decode(
                    "utf-8", errors="ignore"
                )
                buffer += chunk
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    self.line_queue.put(line.strip())
            except Exception as e:
                self.get_logger().warn(f"Serial read error: {e}")

    def process_lines(self):
        while not self.line_queue.empty():
            line = self.line_queue.get()
            match = self.pattern.match(line)
            if not match:
                self.get_logger().warn("Incomplete or malformed data line, skipping.")
                continue

            try:
                ax, ay, az, gx, gy, gz, mx, my, mz, temp = map(float, match.groups())

                imu_msg = Imu()
                imu_msg.header = Header()
                imu_msg.header.stamp = self.get_clock().now().to_msg()
                imu_msg.header.frame_id = "imu_link"

                # Convert mg to m/s²
                imu_msg.linear_acceleration.x = ax * 9.80665 / 1000.0
                imu_msg.linear_acceleration.y = ay * 9.80665 / 1000.0
                imu_msg.linear_acceleration.z = az * 9.80665 / 1000.0

                # deg/s to rad/s
                imu_msg.angular_velocity.x = gx * pi / 180.0
                imu_msg.angular_velocity.y = gy * pi / 180.0
                imu_msg.angular_velocity.z = gz * pi / 180.0

                imu_msg.orientation_covariance[0] = -1  # Orientation not provided

                self.imu_pub.publish(imu_msg)

                mag_msg = MagneticField()
                mag_msg.header = imu_msg.header
                mag_msg.magnetic_field.x = mx * 1e-6
                mag_msg.magnetic_field.y = my * 1e-6
                mag_msg.magnetic_field.z = mz * 1e-6

                self.mag_pub.publish(mag_msg)

            except Exception as e:
                self.get_logger().error(f"Error parsing or publishing: {e}")

    def destroy_node(self):
        self.keep_reading = False
        if self.reader_thread.is_alive():
            self.reader_thread.join()
        self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = IMUSerialPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
