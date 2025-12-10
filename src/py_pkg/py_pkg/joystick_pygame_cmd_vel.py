#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import pygame


class LogitechCmdVelDirect(Node):
    def __init__(self):
        super().__init__('joystick_cmd_vel')

        # ----- CONFIGURACIÓN -----
        # Ejes del Logitech Extreme 3D Pro
        # Suelen ser:
        #   eje 1 -> adelante/atrás (stick)
        #   eje 0 -> izquierda/derecha (stick)
        self.axis_linear_index = 1   # Y del stick
        self.axis_angular_index = 0  # X del stick

        self.max_linear = 2.0   # m/s
        self.max_angular = -1.5  # rad/s
        self.deadzone = 0.1     # zona muerta en [-1, 1]

        # Publisher a /cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, '/catamaran/cmd_vel', 10)

        # Init pygame y joystick
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            self.get_logger().error("No se ha detectado ningún joystick. Conéctalo y vuelve a lanzar el nodo.")
            raise SystemExit

        self.joy = pygame.joystick.Joystick(0)
        self.joy.init()

        self.get_logger().info(f"Joystick detectado: {self.joy.get_name()}")

        # Timer para leer el joystick y publicar (20 Hz)
        self.timer = self.create_timer(0.05, self.timer_cb)

    def apply_deadzone(self, value: float) -> float:
        """Aplica zona muerta simple en [-1,1]."""
        if abs(value) < self.deadzone:
            return 0.0
        return value

    def timer_cb(self):
        # Actualiza el estado interno de pygame
        pygame.event.pump()

        # Leemos los ejes directamente del hardware
        raw_lin = -self.joy.get_axis(self.axis_linear_index)  
        raw_ang =  self.joy.get_axis(self.axis_angular_index)

        raw_lin = self.apply_deadzone(raw_lin)
        raw_ang = self.apply_deadzone(raw_ang)

        # Escalamos a velocidades físicas
        linear_x  = raw_lin * self.max_linear
        angular_z = raw_ang * self.max_angular

        twist = Twist()
        twist.linear.x  = linear_x
        twist.angular.z = angular_z

        self.cmd_vel_pub.publish(twist)

        # Debug opcional
        # self.get_logger().info(f"joy: lin={raw_lin:.2f}, ang={raw_ang:.2f} -> "
        #                        f"cmd_vel: lin={linear_x:.2f}, ang={angular_z:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = LogitechCmdVelDirect()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        pygame.quit()


if __name__ == '__main__':
    main()
