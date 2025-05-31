import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import sqrt
from interfaces_pkg.srv import Examen2024

class Servidor(Node):
    def __init__(self):
        super().__init__('servidor_movimiento')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.srv = self.create_service(Examen2024, '/mover_robot', self.callback_servicio)

        self.inicio = None
        self.posicion_actual = None
        self.valor = 0.0
        self.acel = 0.0
        self.vel_max = 0.0
        self.decel = 0.0
        self.mover = False

        self.get_logger().info('Nodo listo. Esperando llamadas al servicio...')

    def callback_servicio(self, request, response):
        if self.mover:
            response.success = False
            response.message = 'El robot ya está en movimiento'
        else:
            self.inicio = None
            self.mover = True
            self.valor = request.valor
            self.acel = request.acel
            self.vel_max = request.vel_max
            self.decel = request.decel

            response.success = True
            response.message = 'Movimiento iniciado'
            self.get_logger().info('Servicio recibido. Comenzando movimiento...')

        return response

    def odom_callback(self, msg):
        if not self.mover:
            return

        self.posicion_actual = msg.pose.pose.position
        if self.inicio is None:
            self.inicio = self.posicion_actual
            return

        dx = self.posicion_actual.x - self.inicio.x
        dy = self.posicion_actual.y - self.inicio.y
        distancia = sqrt(dx**2 + dy**2)
        restante = self.valor - distancia

        if distancia >= self.valor:
            self.publicar_velocidad(0.0)
            self.mover = False
            self.get_logger().info('Movimiento completado')
            return

        # Fase de aceleración y deceleración según la distancia
        vel_deseada = min(self.acel * distancia, self.vel_max)

        if restante < (self.vel_max ** 2) / (2 * self.decel):
            vel_deseada = self.decel * restante

        # 🚀 Añadido: velocidad mínima para arrancar
        if vel_deseada < 0.05:
            vel_deseada = 0.05

        self.get_logger().info(f'Distancia: {distancia:.2f}, Velocidad: {vel_deseada:.2f}')
        self.publicar_velocidad(vel_deseada)

    def publicar_velocidad(self, lineal=0.0):
        msg = Twist()
        msg.linear.x = lineal
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    nodo = Servidor()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

			
