#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf_transformations import euler_from_quaternion

# Parámetros de control y seguridad
DISTANCIA_SEGURIDAD = 0.2
VELOCIDAD_LINEAL = 0.2

class StopIfObstacle(Node):
    def __init__(self):
        super().__init__('robot_stop_if_obstacle')

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.yaw = 0.0
        self.timer = self.create_timer(0.1, self.control_loop)
        self.obstacle_detected = False

    def odom_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y,
                            orientation_q.z, orientation_q.w]
        (_, _, self.yaw) = euler_from_quaternion(orientation_list)

    def filtrar_lecturas(self, valores):
        return [r if r > 0.14 else 400.0 for r in valores]

    def lidar_callback(self, scan_data):
        num_muestras = len(scan_data.ranges)
        indice_centro = num_muestras // 2
        rango = 10
        indices = range(indice_centro - rango, indice_centro + rango + 1)

        lecturas_frente = self.filtrar_lecturas([scan_data.ranges[i] for i in indices])
        distancia_minima = min(lecturas_frente)

        self.get_logger().info(f"Distancia mínima frontal (21°): {distancia_minima:.2f}")

        self.obstacle_detected = distancia_minima < DISTANCIA_SEGURIDAD

    def control_loop(self):
        vel_msg = Twist()
        if self.obstacle_detected:
            self.get_logger().info("Obstáculo detectado. Deteniendo robot.")
            vel_msg.linear.x = 0.0
        else:
            vel_msg.linear.x = VELOCIDAD_LINEAL
        vel_msg.angular.z = 0.0
        self.publisher_.publish(vel_msg)

def main(args=None):
    rclpy.init(args=args)
    nodo = StopIfObstacle()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

