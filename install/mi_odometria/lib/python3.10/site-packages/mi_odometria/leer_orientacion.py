import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

class LeerOrientacion(Node):
    def __init__(self):
        super().__init__('LeerOrientacion')

        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        # Obtenemos el cuaternión
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )

        # Convierte a ángulos de Euler (roll, pitch, yaw)
        roll, pitch, yaw = euler_from_quaternion(quaternion)

        # Muestra los ángulos
        self.get_logger().info(f'Roll: {roll:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f}')


def main(args=None):
    rclpy.init(args=args)
    leer_orientacion = LeerOrientacion()
    rclpy.spin(leer_orientacion)
    leer_orientacion.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
