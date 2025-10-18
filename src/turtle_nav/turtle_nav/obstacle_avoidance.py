import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class TurtleAvoider(Node):
    def __init__(self):
        super().__init__('turtle_avoider')

        # Publisher (velocity) and Subscriber (pose)
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        # Obstacle positions (x, y)
        self.obstacles = [(5.5, 5.5), (2.0, 8.0), (8.0, 2.0)]
        self.threshold = 1.0   # Distance to consider "too close"

        self.cmd = Twist()
        self.pose = None

        self.timer = self.create_timer(0.1, self.move_turtle)
        self.get_logger().info("🐢 Turtle obstacle avoidance node started!")

    def pose_callback(self, msg):
        self.pose = msg

    def is_near_obstacle(self):
        if self.pose is None:
            return False
        for (ox, oy) in self.obstacles:
            dist = math.sqrt((self.pose.x - ox)**2 + (self.pose.y - oy)**2)
            if dist < self.threshold:
                self.get_logger().warn(f"🚧 Near obstacle at ({ox:.1f}, {oy:.1f})!")
                return True
        return False

    def move_turtle(self):
        if self.pose is None:
            return

        if self.is_near_obstacle():
            # Turn away
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 1.5
        else:
            # Move forward
            self.cmd.linear.x = 2.0
            self.cmd.angular.z = 0.0

        self.pub.publish(self.cmd)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleAvoider()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
