import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, termios, tty
 
class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info("Teleop Node Started. Use WASD keys to move the robot.")
        self.run()
 
    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key
 
    def run(self):
        self.get_logger().info(f"The key pressed is ")
        twist = Twist()
        self.get_logger().info(f"The key pressed is ")
        while True:
            key = self.get_key()
            if key == 'w':
                twist.linear.x += 0.01
                twist.angular.z = 0.0
            elif key == 'x':
                twist.linear.x += -0.01
                twist.angular.z =  0.0
            elif key == 'a':
                twist.linear.x = 0.01
                twist.angular.z = 0.01
            elif key == 'd':
                twist.linear.x = 0.01
                twist.angular.z = -0.01
            elif key == 'q':  # Quit
                break
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            self.get_logger().info(f"The key pressed is {key}")
            self.publisher_.publish(twist)
 
def main():
    rclpy.init()
    teleop_node = TeleopNode()
    teleop_node.destroy_node()
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()
 