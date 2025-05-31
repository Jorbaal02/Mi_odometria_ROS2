import rclpy
from rclpy.node import Node
from servicios.srv import ejercicio3

class Cliente_ejercicio3(Node):
    def __init__(self):
        super().__init__('Cliente_examen')
        self.cli = self.create_client(ejercicio3, '/mover_robot')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('El robot se encuentra en la linea de salida')

        self.solicitar_datos()

    def solicitar_datos(self):        
        x = 1

        request = ejercicio3.Request()
        request.x = x

        resp = self.cli.call_async(request)
        resp.add_done_callback(self.respuesta_servicio)

    def respuesta_servicio(self, resp):
        if resp.success:
            self.get_logger().info('Datos enviados con exito')

def main(args=None):
    rclpy.init(args=args)
    nodo = Cliente_ejercicio3()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()