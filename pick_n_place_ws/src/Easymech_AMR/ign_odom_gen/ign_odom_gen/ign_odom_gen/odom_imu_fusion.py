#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf_transformations import quaternion_from_euler
from encoder_msgs.msg import EncoderData  # Replace with actual encoder message type
import math
import tf2_ros

class SkidSteerOdometry(Node):
    def __init__(self):
        super().__init__('skid_steer_odometry')

        # Robot parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('wheel_radius', 0.1),   
                ('wheel_separation', 0.5),  
                ('encoder_resolution', 306),  # Encoder ticks per revolution
                ('slip_factor_linear', 0.5),  # Adjust empirically
                ('slip_factor_angular', 0.25)   # Adjust for better rotation accuracy
            ]
        )

        # Load parameters
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.encoder_resolution = self.get_parameter('encoder_resolution').value
        self.k_v = self.get_parameter('slip_factor_linear').value
        self.k_omega = self.get_parameter('slip_factor_angular').value

        # Robot pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_time = self.get_clock().now()
        self.first_update = True

        # Previous encoder values
        self.prev_enc_left_front = None
        self.prev_enc_left_back = None
        self.prev_enc_right_front = None
        self.prev_enc_right_back = None

        # ROS 2 Interfaces
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.encoder_sub = self.create_subscription(EncoderData, 'encoder_data', self.encoder_callback, 10)

        # TF Broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.get_logger().info("Skid-Steer Odometry Node Initialized")

    def encoder_callback(self, msg):
        """
        Callback for encoder readings.
        """
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds * 1e-9
        if dt == 0:
            return

        if self.first_update:
            self.prev_enc_left_front = msg.enc_left_front
            self.prev_enc_left_back = msg.enc_left_back
            self.prev_enc_right_front = msg.enc_right_front
            self.prev_enc_right_back = msg.enc_right_back
            self.first_update = False
            return

        vL = self.compute_wheel_velocity(msg.enc_left_front, msg.enc_left_back, self.prev_enc_left_front, self.prev_enc_left_back, dt)
        vR = self.compute_wheel_velocity(msg.enc_right_front, msg.enc_right_back, self.prev_enc_right_front, self.prev_enc_right_back, dt)

        self.prev_enc_left_front = msg.enc_left_front
        self.prev_enc_left_back = msg.enc_left_back
        self.prev_enc_right_front = msg.enc_right_front
        self.prev_enc_right_back = msg.enc_right_back

        v, omega = self.compute_corrected_odometry(vL, vR)
        self.x, self.y, self.theta = self.update_pose(v, omega, dt)
        self.publish_odometry(v, omega, current_time)
        self.publish_transform(current_time)
        
        self.prev_time = current_time

    def compute_wheel_velocity(self, front_ticks, back_ticks, prev_front, prev_back, dt):
        delta_front = front_ticks - prev_front
        delta_back = back_ticks - prev_back
        delta_ticks = (delta_front + delta_back) / 2.0
        wheel_circumference = 2 * math.pi * self.wheel_radius
        velocity = (delta_ticks / self.encoder_resolution) * wheel_circumference / dt
        return velocity

    def compute_corrected_odometry(self, vL, vR):
        v = ((vR + vL) / 2) * self.k_v
        omega = ((vR - vL) / self.wheel_separation) * self.k_omega
        return v, omega

    def update_pose(self, v, omega, dt):
        x = self.x + v * math.cos(self.theta) * dt
        y = self.y + v * math.sin(self.theta) * dt
        theta = self.theta + omega * dt
        return x, y, theta

    def publish_odometry(self, v, omega, current_time):
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_footprint"

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        quat = quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])

        odom_msg.twist.twist.linear.x = v
        odom_msg.twist.twist.angular.z = omega

        self.odom_pub.publish(odom_msg)

    def publish_transform(self, current_time):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = current_time.to_msg()
        tf_msg.header.frame_id = "odom"
        tf_msg.child_frame_id = "base_footprint"

        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = 0.0

        quat = quaternion_from_euler(0, 0, self.theta)
        tf_msg.transform.rotation.x = quat[0]
        tf_msg.transform.rotation.y = quat[1]
        tf_msg.transform.rotation.z = quat[2]
        tf_msg.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SkidSteerOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
