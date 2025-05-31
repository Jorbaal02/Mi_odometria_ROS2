import rclpy
from rclpy.node import Node
from interfaces_pkg.srv import Examen2024

class Cliente(Node):
    def __init__(self):
        super().__init__('cliente_movimiento')
        self.cli = self.create_client(Examen2024, '/mover_robot')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando al servicio /mover_robot...')

        self.solicitar_datos()

    def solicitar_datos(self):
        try:
            valor = float(input("Introduce la distancia a recorrer (m): "))
            acel = float(input("Introduce la aceleración (m/s²): "))
            vel_max = float(input("Introduce la velocidad máxima (m/s): "))
            decel = float(input("Introduce la constante de deceleración: "))

            request = Examen2024.Request()
            request.valor = valor
            request.acel = acel
            request.vel_max = vel_max
            request.decel = decel

            future = self.cli.call_async(request)
            future.add_done_callback(self.respuesta_servicio)

        except Exception as e:
            self.get_logger().error(f'Error al ingresar datos: {e}')

    def respuesta_servicio(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Respuesta: {response.message}')
            else:
                self.get_logger().warn(f'Servicio rechazado: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Error al llamar al servicio: {e}')

def main(args=None):
    rclpy.init(args=args)
    nodo = Cliente()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

