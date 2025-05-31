import sys
import math
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf_transformations import euler_from_quaternion

class NodoPoligono(Node):
    def __init__(self, N, L):
        super().__init__('poligono_node')
        self.N = N
        self.L = L
        self.kp = 0.5
        self.yaw = 0.0
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.yaw_objetivo = 0.0

        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.get_pos,
            10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info('Esperando mensaje de /odom...')
        self.odom_recibido = False
        while rclpy.ok() and not self.odom_recibido:
            rclpy.spin_once(self)

        self.ejecutar_poligono()

    def get_pos(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ]
        (_, _, self.yaw) = euler_from_quaternion(orientation_list)
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y
        self.odom_recibido = True

    def normalizar_angulo(self, angulo):
        while angulo > math.pi:
            angulo -= 2 * math.pi
        while angulo < -math.pi:
            angulo += 2 * math.pi
        return angulo

    def ejecutar_poligono(self):
        angulo_giro = (2 * math.pi) / self.N
        self.yaw_objetivo = self.yaw
        vel = Twist()
        rate = self.create_rate(10)

        for _ in range(self.N):
            # Movimiento lineal
            x_inicial, y_inicial = self.pos_x, self.pos_y

            while rclpy.ok():
                distancia = math.sqrt((self.pos_x - x_inicial) ** 2 + (self.pos_y - y_inicial) ** 2)
                if distancia >= (self.L - 0.01):
                    break
                vel.linear.x = self.kp * (self.L - distancia)
                vel.angular.z = 0.0
                self.publisher.publish(vel)
                rclpy.spin_once(self)
                rate.sleep()

            vel.linear.x = 0.0
            self.publisher.publish(vel)
            time.sleep(1)

            # Giro
            self.yaw_objetivo += angulo_giro
            self.yaw_objetivo = self.normalizar_angulo(self.yaw_objetivo)

            while rclpy.ok():
                error_ang = self.normalizar_angulo(self.yaw_objetivo - self.yaw)
                if abs(error_ang) < 0.01:
                    break
                vel.linear.x = 0.0
                vel.angular.z = self.kp * error_ang
                self.publisher.publish(vel)
                rclpy.spin_once(self)
                rate.sleep()

            vel.angular.z = 0.0
            self.publisher.publish(vel)
            time.sleep(1)

        self.get_logger().info('✅ Polígono completo')

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 3:
        print("Uso: ros2 run <paquete> <nodo> <nº_lados> <longitud_lado>")
        return

    N = int(sys.argv[1])
    L = float(sys.argv[2])

    nodo = NodoPoligono(N, L)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
