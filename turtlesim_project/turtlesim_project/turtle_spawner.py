#!/usr/bin/env python3
import random
import math
from functools import partial
import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from std_srvs.srv import Empty
from turtlesim.srv import Kill
from robot_interfaces.msg import Turtle
from robot_interfaces.msg import TurtleArray
from robot_interfaces.srv import CatchTurtle


random.seed(1)

class TurtleSpawnerNode(Node): 
    def __init__(self):
        super().__init__("turtle_spawner")
        self.turtle_first_name_ = "turtle"
        self.turtle_counter = 1
        self.alive_turtles_ = []
        self.caught_turtle_ = None
        self.alive_turtles_publisher_= self.create_publisher(TurtleArray,"alive_turtles", 10)
        self.declare_parameter("spawn_frequency", 0.7)
        self.spawn_frequency_ = self.get_parameter("spawn_frequency").value
        self.timer_ = self.create_timer(self.spawn_frequency_, self.spawn_new_turtle)
        # creating service /catch_turtle
        self.catch_turtle_service_ = self.create_service(CatchTurtle, "catch_turtle", self.callback_catch_turtle)



    # service callback
    def callback_catch_turtle(self, request, response):
        self.call_turtle_killer(request.turtle_name)
        self.call_erase_path()
        response.success = True
        return response
        

    
    def publish_alive_turtles(self):
        msg = TurtleArray()
        msg.turtles = self.alive_turtles_
        self.alive_turtles_publisher_.publish(msg)
        

    def spawn_new_turtle(self):
        self.turtle_counter +=1
        name = self.turtle_first_name_ + str(self.turtle_counter)
        x = random.uniform(2.0, 9.0)
        y = random.uniform(2.0, 9.0)
        theta = random.uniform(0.0, 2*math.pi)
        self.call_turtle_spawner(x, y, theta, name)
        
######---------------------------------------------------------------#####
    def call_turtle_spawner(self, x, y, theta, turtle_name):
        client = self.create_client(Spawn, "spawn")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server...")
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = turtle_name
        future_obj = client.call_async(request)
        future_obj.add_done_callback(partial(self.callback_call_turtle_spawner, x=x, y=y, 
                                                            theta=theta, turtle_name=turtle_name))

    def callback_call_turtle_spawner(self, future_obj, x, y, theta, turtle_name):
        try:
            response = future_obj.result()
            if response.name != "":
                self.get_logger().info("Turtle " + response.name + " has been spawned")
                new_turtle = Turtle()
                new_turtle.name = response.name
                new_turtle.x = x
                new_turtle.y = y
                new_turtle.theta = theta
                self.alive_turtles_.append(new_turtle)
                self.publish_alive_turtles()
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

#####---------------------------------------------------------------######
    def call_turtle_killer(self,turtle_name):
        client = self.create_client(Kill, "kill")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server...")
        request = Kill.Request()
        request.name = turtle_name
        future_obj = client.call_async(request)
        future_obj.add_done_callback(partial(self.callback_call_turtle_killer, turtle_name=turtle_name))

    def callback_call_turtle_killer(self, future_obj, turtle_name):
        try:
            future_obj.result()
            for (i, turtle) in enumerate(self.alive_turtles_):
                if turtle.name == turtle_name:
                    del self.alive_turtles_[i]
                    self.publish_alive_turtles()
                    break
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

#####---------------------------------------------------------------######
    def call_erase_path(self):
        client = self.create_client(Empty, "clear")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server...")
        request = Empty.Request()
        future_obj = client.call_async(request)
        future_obj.add_done_callback(partial(self.callback_call_erase_path))

    def callback_call_erase_path(self, future_obj):
        try:
            future_obj.result()
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))
    

def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawnerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()