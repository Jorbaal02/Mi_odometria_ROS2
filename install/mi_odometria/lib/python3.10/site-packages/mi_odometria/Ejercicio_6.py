#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf_transformations import euler_from_quaternion
import argparse

class TrianguloNode(Node):
    def __init__(self, lado):
        super().__init__('triangulo')
        self.L = lado
        self.kp = 0.5
        self.angulo_giro = 2 * math.pi / 3  # 120 grados
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.yaw = 0.0

        self.sub = self.create_subscription(Odometry, '/odom', self.get_pos, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.estado = 0
        self.x_inicial = 0.0
        self.y_inicial = 0.0
        self.yaw_inicial = 0.0
        self.objetivo_yaw = 0.0
        self.vel = Twist()
        self.lado_actual = 0

    def get_pos(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, self.yaw) = euler_from_quaternion(orientation_list)
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y

    def normalizar_angulo(self, angulo):
        while angulo > math.pi:
            angulo -= 2 * math.pi
        while angulo < -math.pi:
            angulo += 2 * math.pi
        return angulo

    def control_loop(self):
        if self.lado_actual >= 3:
            self.vel.linear.x = 0.0
            self.vel.angular.z = 0.0
            self.pub.publish(self.vel)
            return

        if self.estado == 0:
            self.x_inicial = self.pos_x
            self.y_inicial = self.pos_y
            self.estado = 1

        elif self.estado == 1:
            distancia = math.sqrt((self.pos_x - self.x_inicial) ** 2 + (self.pos_y - self.y_inicial) ** 2)
            if distancia >= (self.L - 0.01):
                self.vel.linear.x = 0.0
                self.pub.publish(self.vel)
                self.estado = 2
                self.get_logger().info(f'Lado {self.lado_actual + 1} completado')
                self.yaw_inicial = self.yaw
                self.objetivo_yaw = self.yaw_inicial + self.angulo_giro
            else:
                self.vel.linear.x = self.kp * (self.L - distancia)
                self.vel.angular.z = 0.0
                self.pub.publish(self.vel)

        elif self.estado == 2:
            error_ang = self.normalizar_angulo(self.objetivo_yaw - self.yaw)
            if abs(error_ang) < 0.01:
                self.vel.angular.z = 0.0
                self.pub.publish(self.vel)
                self.lado_actual += 1
                self.estado = 0
                self.get_logger().info(f'Giro completado')
            else:
                self.vel.linear.x = 0.0
                self.vel.angular.z = self.kp * error_ang
                self.pub.publish(self.vel)

def main(args=None):
    parser = argparse.ArgumentParser(description='Genera trayectoria triangular en ROS2')
    parser.add_argument('lado', type=float, help='Longitud de cada lado del triángulo en metros')
    parsed_args = parser.parse_args()

    rclpy.init(args=args)
    node = TrianguloNode(parsed_args.lado)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
