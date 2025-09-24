import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CircleMover(Node):
    def __init__(self):
        super().__init__('circle_mover')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.move)

    def move(self):
        msg = Twist()
        msg.linear.x = 2.0    # forward speed
        msg.angular.z = 1.0   # turning speed
        self.publisher.publish(msg)
        self.get_logger().info("Moving turtle in a circle...")

def main(args=None):
    rclpy.init(args=args)
    node = CircleMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
