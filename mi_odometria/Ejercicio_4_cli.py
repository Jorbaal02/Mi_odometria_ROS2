import rclpy
from rclpy.node import Node
from servicios.srv import ejercicio4 

class ClienteEjercicio3(Node):
    def __init__(self):
        super().__init__('cliente_ejercicio3')
        self.cli = self.create_client(ejercicio4, '/mover_robot')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando a que el servicio /mover_robot esté disponible...')

        self.solicitar_datos()

    def solicitar_datos(self):
        # Pedir datos al usuario
        tipo = "lineal" # o "rotacion"
        valor = 1 # A gusto del consumidor

        if tipo not in ["lineal", "rotacion"]:
            self.get_logger().error("Tipo no válido. Usa 'lineal' o 'rotacion'.")
            return

        request = ejercicio4.Request()
        request.tipo = tipo
        request.valor = valor

        future = self.cli.call_async(request)
        future.add_done_callback(self.respuesta_servicio)

    def respuesta_servicio(self, future):
        response = future.result()
        if response.success:
            self.get_logger().info('Movimiento ejecutado con éxito.')
        else:
            self.get_logger().warn('El movimiento no se pudo ejecutar.')

def main(args=None):
    rclpy.init(args=args)
    nodo = ClienteEjercicio3()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
