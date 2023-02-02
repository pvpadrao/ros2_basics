#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64


class NumberPublisherNode(Node): 
    def __init__(self):
        super().__init__("number_publisher")
        # declaring parameter + default value
        #self.declare_parameter("test123", 4)
        self.declare_parameter("number_to_publish", 2)
        self.declare_parameter("frequency", 1.0)
        
        self.number_ = self.get_parameter("number_to_publish").value
        self.number_publisher = self.create_publisher(Int64, "number", 10)
        self.frequency_ = self.get_parameter("frequency").value
        self.number_timer_ = self.create_timer(1/self.frequency_, self.callback_publish_number)
        self.get_logger().info("Number publisher has started!")

    def callback_publish_number(self):
        msg = Int64()
        msg.data = self.number_
        self.number_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()