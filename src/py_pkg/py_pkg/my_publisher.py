#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String


class PublisherNode(Node): 
    def __init__(self):
        super().__init__("my_publisher") 
        self.publishers_ = self.create_publisher(String,"robot_news",10)
        self.counter_ = 0
        self.timer_=self.create_timer(0.5,self.publish_news)
        self.get_logger().info("Robot News Station has been started.")

    def publish_news(self):
        msg = String()
        self.get_logger().info(f"Publishing: Bye {self.counter_}")
        msg.data = f"Bye {self.counter_}"
        self.publishers_.publish(msg)
        self.counter_+=1

def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
