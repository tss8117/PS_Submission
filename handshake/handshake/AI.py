#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class AI_node(Node):
    def __init__(self):
        super().__init__("AI")
        self.get_msg = ""
        self.publisher = self.create_publisher(String,"AI2VCU",10)
        self.subscriber = self.create_subscription(String, "VCU2AI", self.AI_subscribe_callback,10)
        self.timer = self.create_timer(1.0, self.AI_publish_callback)
    
    def AI_subscribe_callback(self, msg:String):
        self.get_logger().info("Recieved message from VCU: "+str(msg.data))


    def AI_publish_callback(self):
        msg = String()
        msg.data = "HI!"
        self.publisher.publish(msg)
        self.get_logger().info(str(msg.data))


def main(args=None):
    rclpy.init(args=args)
    node = AI_node()
    rclpy.spin(node)
    rclpy.shutdown()