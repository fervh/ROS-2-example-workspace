## Nodo encargado de ofrecer el servicio de multiplicar por 2 la edad recibida del usuario

import rclpy
from rclpy.node import Node
from interaccion_interfaces.srv import Multiplicador # Importamos el servicio de Multiplicador

class MatematicoNodo(Node):

    # Constructor del nodo
    def __init__(self):
        # Inicializamos el nodo con el nombre de matematico_nodo
        super().__init__('matematico_nodo')
        # Crear el servicio de Multiplicador, con el nombre de servicio_multiplicador, y la funcion multiplicador_callback
        self.multiplicador_service = self.create_service(Multiplicador, 'servicio_multiplicador', self.multiplicador_callback)
        
        # Imprimir mensaje de inicio
        self.get_logger().info("Matematico nodo iniciado")
    
    # Funcion que se ejecuta cuando se recibe una peticion de servicio
    def multiplicador_callback(self, request, response):
        response.resultado = request.entrada * 2 # Multiplicar por 2 la edad recibida
        # Imprimir mensaje con la edad recibida y el resultado
        self.get_logger().info(f"Multiplicador servicio recibido: {request.entrada}, resultado: {response.resultado}")
        return response

# Funcion main
def main(args=None):
    # Inicializar el nodo y crear instancia de la clase
    rclpy.init(args=args)
    matematico_nodo = MatematicoNodo()

    try:
        rclpy.spin(matematico_nodo) # Ejecución secuencial de callbacks
    except KeyboardInterrupt:
        pass

    # Destruir el nodo y finalizar la ejecucion
    matematico_nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
