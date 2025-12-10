#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
from catamaran_interfaces.msg import Velocity

class ControlVelocityNode(Node): 
    def __init__(self):
        super().__init__("control_velocity_py") 
        self.publishers_ = self.create_publisher(Velocity,"velocity",10)
        self.timer_=self.create_timer(0.5,self.publish_news)
        self.get_logger().info("Velocity control has been started.")

    def publish_news(self):
        msg = Velocity()
        command = input("Give the velocity (vL,vR): ")
        coma_pos = command.find(',')
        if(coma_pos!=-1):
            msg.v_left = int (command[:coma_pos])
            msg.v_right = int (command[coma_pos+1:])
            self.get_logger().info(f"Publishing: {command}")
            self.publishers_.publish(msg)
        else:
            self.get_logger().error(f"Wrong input please give a velocity (vL,vR)")

def main(args=None):
    rclpy.init(args=args)
    node = ControlVelocityNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
