#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class RotateRobot(Node):
    def __init__(self):
        super().__init__('rotate_robot')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.vel_target = 0.05  # m/s
        self.time_to_move = 20  # s
        self.timer_period = 0.1  # 10 Hz
        self.start_time = time.time()
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.done = False

    def timer_callback(self):
        current_time = time.time()
        vel = Twist()
        if not self.done and (current_time - self.start_time) < self.time_to_move:
            vel.linear.x = self.vel_target
            self.publisher_.publish(vel)
        else:
            if not self.done:
                vel.linear.x = 0.0
                self.publisher_.publish(vel)
                self.get_logger().info('El robot ha recorrido 1 metro')
                self.done = True
            # Puedes añadir: self.destroy_node() si quieres cerrar el nodo

def main(args=None):
    rclpy.init(args=args)
    node = RotateRobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
