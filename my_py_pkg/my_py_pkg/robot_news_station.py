#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class RobotNewsStationNode(Node):
    def __init__(self):
        super().__init__("robot_news_station") # usually the same name for the file and node
        self.declare_parameter("robot_name", "Aqua")
        self.robot_name_ = self.get_parameter("robot_name").value
        #self.robot_name_ = "Aqua"
        # creating a publisher: data type, data content, buffer size
        self.publisher_ = self.create_publisher(String, "robot_news", 10)
        # creating a timer to call the function publish_news
        self.timer_ = self.create_timer(0.5, self.publish_news)
        self.get_logger().info("Robot News Station has been started!")

    # creating a function to publish
    def publish_news(self):
        msg = String()
        msg.data = "Hi, this is " + str(self.robot_name_)
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RobotNewsStationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

