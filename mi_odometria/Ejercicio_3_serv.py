import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from servicios.srv import ejercicio3  

class RobotMover(Node):
    def __init__(self):
        super().__init__('robot_mover')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.srv = self.create_service(ejercicio3, 'mover_robot', self.callback_servicio)

        self.kp = 1.0
        self.avanzando = False
        self.x_actual = None
        self.x_inicial = None
        self.x_target = None

        self.get_logger().info('Nodo listo para mover el robot una distancia.')

    def callback_servicio(self, request, response):
        if self.x_actual is None:
            self.get_logger().warn("Esperando odometría antes de mover...")
            response.success = False
            return response

        # Definir punto inicial y objetivo
        self.x_inicial = self.x_actual
        self.x_target = self.x_inicial + request.distancia
        self.avanzando = True

        self.get_logger().info(f'Distancia objetivo: {request.distancia:.2f} m')
        response.success = True
        return response

    def odom_callback(self, msg):
        self.x_actual = msg.pose.pose.position.x

        if self.avanzando:
            error = self.x_target - self.x_actual
            vel = Twist()

            if abs(error) < 0.01:
                vel.linear.x = 0.0
                self.avanzando = False
                self.get_logger().info("Objetivo alcanzado.")
            else:
                vel.linear.x = self.kp * error

            self.cmd_pub.publish(vel)

def main(args=None):
    rclpy.init(args=args)
    nodo = RobotMover()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
