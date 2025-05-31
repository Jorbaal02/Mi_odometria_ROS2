import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class MoverProporcional(Node):
    def __init__(self):
        super().__init__('mover_proporcional')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.kp = 0.4
        self.x_actual = None
        self.x_inicial = None
        self.x_target = None
        self.referencia_recibida = False

    def odom_callback(self, msg):
        self.x_actual = msg.pose.pose.position.x
        if not self.referencia_recibida:
            self.x_inicial = self.x_actual
            self.x_target = self.x_inicial + 1.0
            self.referencia_recibida = True
            self.get_logger().info(f"Referencia inicial recibida: x = {self.x_inicial:.2f}")

    def control_loop(self):
        if not self.referencia_recibida or self.x_actual is None:
            return

        error = self.x_target - self.x_actual
        vel = Twist()
        vel.linear.x = self.kp * error

        # Detener el robot si ya llegó
        if abs(error) < 0.01:
            vel.linear.x = 0.0
            self.get_logger().info("Objetivo alcanzado.")
            self.destroy_timer(self.timer)

        self.publisher_.publish(vel)
        self.get_logger().info(f'Target: {self.x_target:.2f}, Actual: {self.x_actual:.2f}, Vel: {vel.linear.x:.2f}')

def main(args=None):
    rclpy.init(args=args)
    nodo = MoverProporcional()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
