#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class VCU_node(Node):
    def __init__(self):
        super().__init__("VCU")
        self.get_msg = ""
        self.subscriber = self.create_subscription(String, "AI2VCU", self.VCU_subscribe_callback,10)
        self.publisher = self.create_publisher(String,"VCU2AI",10)
        self.timer = self.create_timer(1.0, self.VCU_publish_callback)
    
    def VCU_subscribe_callback(self, msg:String):
        self.get_msg = msg.data


    def VCU_publish_callback(self):
        w_msg = String()
        w_msg.data = "Nice to know that AI is working!!"
        nw_msg = String()
        nw_msg.data = "Ohh No seems like AI is not working"
        if(self.get_msg != ""):
            self.get_logger().info("Received message from AI: " + str(self.get_msg))
            self.publisher.publish(w_msg)
            self.get_logger().info(str(w_msg.data))
            self.get_msg = ""
        else:
            self.publisher.publish(nw_msg)
            self.get_logger().info(str(nw_msg.data))




def main(args=None):
    rclpy.init(args=args)
    node = VCU_node()
    rclpy.spin(node)
    rclpy.shutdown()