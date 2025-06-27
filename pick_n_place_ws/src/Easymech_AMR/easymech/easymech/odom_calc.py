import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
import transforms3d as tf_transformations
import tf2_ros
import math

class EncoderOdometry(Node):
    def __init__(self):
        super().__init__('encoder_odometry')

        # ROS2 QoS settings
        qos_profile = QoSProfile(depth=10)

        # Subscribe to encoder ticks
        self.subscription = self.create_subscription(
            Int32MultiArray,
            'encoder_data',
            self.encoder_callback,
            qos_profile)

        # Publisher for odometry
        self.odom_publisher = self.create_publisher(Odometry, 'odom', qos_profile)

        # TF broadcaster for odom->base_link
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Robot parameters
        self.wheel_radius = 0.05  # Wheel radius in meters
        self.wheel_base = 0.5     # Distance between left and right wheels
        self.ticks_per_rev = 1000  # Encoder ticks per full wheel rotation
        self.meters_per_tick = (2 * math.pi * self.wheel_radius) / self.ticks_per_rev

        # Initial pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Last received encoder values
        self.last_ticks = [0, 0, 0, 0]
        self.last_time = self.get_clock().now()

    def encoder_callback(self, msg):
        """Processes encoder tick data and updates odometry."""
        current_time = self.get_clock().now()

        # Read encoder values
        ticks = msg.data  # [FL, BL, FR, BR]

        # Print received encoder ticks
        self.get_logger().info(f"Received Encoder Ticks: FL={ticks[0]}, BL={ticks[1]}, FR={ticks[2]}, BR={ticks[3]}")

        # Compute delta ticks
        dticks = [ticks[i] - self.last_ticks[i] for i in range(4)]

        # Convert ticks to distance traveled per wheel
        d_left = (dticks[0] + dticks[1]) / 2 * self.meters_per_tick  # Left side
        d_right = (dticks[2] + dticks[3]) / 2 * self.meters_per_tick  # Right side

        # Compute linear and angular displacement
        d_center = (d_left + d_right) / 2
        d_theta = (d_right - d_left) / self.wheel_base

        # Update pose estimate
        self.theta += d_theta
        self.x += d_center * math.cos(self.theta)
        self.y += d_center * math.sin(self.theta)

        # Compute velocities
        dt = (current_time - self.last_time).nanoseconds / 1e9  # Convert ns to sec
        if dt > 0:
            linear_velocity = d_center / dt
            angular_velocity = d_theta / dt
        else:
            linear_velocity = 0.0
            angular_velocity = 0.0

        # Print computed odometry
        self.get_logger().info(f"Odometry: X={self.x:.4f}, Y={self.y:.4f}, Theta={math.degrees(self.theta):.2f}°")
        self.get_logger().info(f"Velocity: Linear={linear_velocity:.4f} m/s, Angular={angular_velocity:.4f} rad/s")

        # Publish odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Convert yaw to quaternion
        q = tf_transformations.quaternions.axangle2quat([0, 0, 1], self.theta)
        odom_msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        # Set velocities
        odom_msg.twist.twist.linear.x = linear_velocity
        odom_msg.twist.twist.angular.z = angular_velocity

        self.odom_publisher.publish(odom_msg)

        # Publish TF transform (odom -> base_link)
        transform = TransformStamped()
        transform.header.stamp = current_time.to_msg()
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_link"

        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation = odom_msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(transform)

        # Store last values
        self.last_ticks = ticks
        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = EncoderOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
