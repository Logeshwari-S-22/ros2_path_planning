import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import speech_recognition as sr

class VoiceTurtle(Node):
    def __init__(self):
        super().__init__('voice_turtle')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.get_logger().info("Voice Controlled Turtle Started 🎤🐢")

        # recognizer setup
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        self.listen_commands()

    def listen_commands(self):
        while rclpy.ok():
            try:
                with self.microphone as source:
                    self.get_logger().info("Say a command: forward, back, left, right, stop")
                    audio = self.recognizer.listen(source)

                command = self.recognizer.recognize_google(audio).lower()
                self.get_logger().info(f"Heard: {command}")
                self.move_turtle(command)

            except sr.UnknownValueError:
                self.get_logger().info("Sorry, I didn't catch that.")
            except sr.RequestError:
                self.get_logger().info("Speech service unavailable.")

    def move_turtle(self, command):
        msg = Twist()
        if "forward" in command:
            msg.linear.x = 2.0
        elif "back" in command:
            msg.linear.x = -2.0
        elif "left" in command:
            msg.angular.z = 2.0
        elif "right" in command:
            msg.angular.z = -2.0
        elif "stop" in command:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        else:
            self.get_logger().info("Unknown command")
            return

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VoiceTurtle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
