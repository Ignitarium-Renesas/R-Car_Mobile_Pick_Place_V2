import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler


class StaticTFPublisher(Node):
    def __init__(self):
        super().__init__("static_tf_publisher")
        self.broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_transforms()

    def create_static_transform(
        self, parent_frame, child_frame, x, y, z, roll, pitch, yaw
    ):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame
        transform.transform.translation.x = float(x)
        transform.transform.translation.y = float(y)
        transform.transform.translation.z = float(z)

        try:
            # Convert Euler angles to quaternion
            q = quaternion_from_euler(float(roll), float(pitch), float(yaw))
            if any(map(lambda v: v != v, q)):  # Check for NaN values
                raise ValueError("Invalid quaternion computed (NaN values detected).")

            transform.transform.rotation.x = q[0]
            transform.transform.rotation.y = q[1]
            transform.transform.rotation.z = q[2]
            transform.transform.rotation.w = q[3]

        except Exception as e:
            self.get_logger().error(
                f"Error computing quaternion for {child_frame}: {e}"
            )
            return None  # Skip invalid transform

        return transform

    def publish_static_transforms(self):
        static_transforms = [
            self.create_static_transform(
                "base_link", "caster_back_link", 0.0, 0.0, -0.1, 0.0, 0.0, 0.0
            ),
            self.create_static_transform(
                "base_link", "imu_link", 0.0, 0.0, 0.2, 0.0, 0.0, 3.14
            ),
            self.create_static_transform(
                "base_link", "base_scan", 0.21, 0.11, 0.3, 0.0, 0.0, 3.14
            ),
            self.create_static_transform(
                "base_link", "wheel_left_link", 0.1, 0.15, 0.0, 0.0, 0.0, 0.0
            ),
            self.create_static_transform(
                "base_link", "wheel_right_link", 0.1, -0.15, 0.0, 0.0, 0.0, 0.0
            ),
            self.create_static_transform(
                "base_footprint", "base_link", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            ),
        ]

        # Filter out invalid transforms
        static_transforms = [t for t in static_transforms if t is not None]

        if static_transforms:
            self.broadcaster.sendTransform(static_transforms)
            self.get_logger().info("Published static transforms")


def main():
    rclpy.init()
    node = StaticTFPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
