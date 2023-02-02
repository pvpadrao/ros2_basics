#!/usr/bin/env python3
import rclpy # library for ROS
from rclpy.node import Node

class MyNode(Node): # create a node that is inherited from rclpy.node Class
    def __init__(self): 
        super().__init__("first_node") # initializing first_node using the class Node
        self.counter = 0
        self.create_timer(1.0, self.timer_callback)
    
    def timer_callback(self):
        self.get_logger().info("Hi" + str(self.counter))
        self.counter += 1


def main(args=None):
    rclpy.init(args=args) # initialize ROS communication
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
