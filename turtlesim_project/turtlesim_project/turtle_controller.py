#!/usr/bin/env python3
import math
import rclpy
from functools import partial
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from robot_interfaces.msg import Turtle
from std_srvs.srv import Empty
from robot_interfaces.msg import TurtleArray
from robot_interfaces.srv import CatchTurtle


class TurtleControllerNode(Node): 
    def __init__(self):
        super().__init__("turtle_controller")
        self.pose_ = None
        self.turtle_to_catch_ = None
        # creating a flag for controller behavior. True: catch closest turtle, False: catch first turtle
        self.declare_parameter("turtle_master_name", "turtle1")
        self.turtle_master_name_ = self.get_parameter("turtle_master_name").value
        self.declare_parameter("flag", True)
        self.flag_ = self.get_parameter("flag").value
        # creating subscriber = msg type, topic, callback function, queue size
        self.pose_subscriber_ = self.create_subscription(Pose,(self.turtle_master_name_+"/pose"),self.callback_turtle_pose,10)
        self.cmd_vel_publisher_ = self.create_publisher(Twist, (self.turtle_master_name_+"/cmd_vel"), 10)
        self.alive_turtles_subscriber = self.create_subscription(TurtleArray, "alive_turtles", 
                                                            self.callback_alive_turtles, 10)
        
        self.get_logger().info("Turtle Master has been connected")
        self.control_loop_timer_ = self.create_timer(0.01, self.control_loop)

    def callback_turtle_pose(self, msg):
        # msg type: Pose
        self.pose_ = msg
        #self.get_logger().info("(" + str(msg.x) + "," + str(msg.y) + ")")

    def callback_alive_turtles(self, msg):
        # msg type: TurtleArray
        if len(msg.turtles) > 0:
            if self.flag_ == False:
                self.turtle_to_catch_ = msg.turtles[0]
            else:
                closest_turtle = None
                closest_turtle_distance = None
                for turtle in msg.turtles:
                    dist_x = turtle.x - self.pose_.x
                    dist_y = turtle.y - self.pose_.y
                    distance = math.sqrt(dist_x * dist_x  + dist_y * dist_y)
                    if closest_turtle == None or distance < closest_turtle_distance:
                        closest_turtle = turtle
                        closest_turtle_distance = distance
                self.turtle_to_catch_ = closest_turtle



    def control_loop(self):
        if self.pose_ == None or self.turtle_to_catch_ == None:
            return
        
        dist_x = self.turtle_to_catch_.x - self.pose_.x
        dist_y = self.turtle_to_catch_.y  - self.pose_.y
        position_error = math.sqrt(dist_x * dist_x + dist_y * dist_y)

        msg = Twist()

        if position_error > 0.4:
            # simple proportional controller
            Kp = 2.0
            msg.linear.x = Kp * position_error
            # orientation
            Ktheta = 5.0
            theta_target = math.atan2(dist_y, dist_x)
            theta_error = theta_target - self.pose_.theta

            if theta_error > math.pi:
                theta_error = theta_error - 2*math.pi
            elif theta_error < -math.pi:
                theta_error = theta_error + 2*math.pi

            msg.angular.z = Ktheta*theta_error
        else:
            # turtle stops
            msg.linear.x = 0.0 
            msg.angular.z = 0.0
            self.call_catch_turtle_server(self.turtle_to_catch_.name)
            self.turtle_to_catch_ = None


        self.cmd_vel_publisher_.publish(msg)
    
    def call_catch_turtle_server(self,turtle_name):
        client = self.create_client(CatchTurtle, "catch_turtle")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server...")
        request = CatchTurtle.Request()
        request.turtle_name = turtle_name
        future_obj = client.call_async(request)
        future_obj.add_done_callback(partial(self.callback_call_catch_turtle_server, turtle_name=turtle_name))

    def callback_call_catch_turtle_server(self, future_obj, turtle_name):
        try:
            response = future_obj.result()
            if not response.success:
                self.get_logger().error("Turtle " + str(turtle_name) + " was not caught")
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))
        

def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()