import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from servicios.srv import ejercicio4
import tf_transformations as tft

class RobotAvanzado(Node):
    def __init__(self):
        super().__init__('robot_avanzado')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.srv = self.create_service(ejercicio4, 'mover_robot', self.callback_servicio)

        self.kp = 1.0
        self.modo = None  # "lineal" o "rotacion"
        self.target = None
        self.inicial = None
        self.actual = None
        self.ejecutando = False

        self.get_logger().info('Nodo preparado para movimiento lineal o rotación.')

    def callback_servicio(self, request, response):
        if self.actual is None:
            self.get_logger().warn("Esperando datos de odometría...")
            response.success = False
            return response

        if request.tipo not in ["lineal", "rotacion"]:
            self.get_logger().error("Tipo inválido. Usa 'lineal' o 'rotacion'.")
            response.success = False
            return response

        self.modo = request.tipo
        self.inicial = self.actual
        self.target = self.inicial + request.valor
        self.ejecutando = True

        self.get_logger().info(f'Inicio de movimiento {self.modo} hacia objetivo: {self.target:.2f}')
        response.success = True
        return response

    def odom_callback(self, msg):
        # Obtener posición o yaw según el modo
        if self.modo == "lineal":
            self.actual = msg.pose.pose.position.x
        elif self.modo == "rotacion":
            orientation_q = msg.pose.pose.orientation
            q = (orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
            _, _, yaw = tft.euler_from_quaternion(q)
            self.actual = self.wrap_angle(yaw)

        if self.ejecutando:
            error = self.wrap_angle(self.target - self.actual) if self.modo == "rotacion" else self.target - self.actual

            vel = Twist()
            if abs(error) < 0.01:
                self.ejecutando = False
                self.get_logger().info(f'Movimiento {self.modo} completado.')
                # Detener movimiento
                self.cmd_pub.publish(Twist())
                return

            if self.modo == "lineal":
                vel.linear.x = self.kp * error
            elif self.modo == "rotacion":
                vel.angular.z = self.kp * error

            self.cmd_pub.publish(vel)

    def wrap_angle(self, angle):
        # Asegura que esté en [-pi, pi]
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    nodo = RobotAvanzado()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
