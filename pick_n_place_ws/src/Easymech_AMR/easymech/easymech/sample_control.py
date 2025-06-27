import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class Figure8Publisher(Node):
    def __init__(self):
        super().__init__('figure8_publisher')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.publish_velocity)
        self.start_time = time.time()
        self.period = 1  # Time to complete one half of the figure-8

    def publish_velocity(self):
        msg = Twist()
        elapsed = time.time() - self.start_time

        if (elapsed // self.period) % 2 == 0:
            # Left circular motion
            msg.linear.x = 0.01  # Move forward
            msg.angular.z = 0.0  # Turn left
        else:
            # Right circular motion
            msg.linear.x = 0.01   # Move forward
            msg.angular.z = 0.0 # Turn right
        
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: Linear={msg.linear.x}, Angular={msg.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    node = Figure8Publisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
