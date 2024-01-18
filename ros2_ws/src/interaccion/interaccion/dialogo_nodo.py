## Nodo suscriptor encargado de mostrar por terminar cada mensaje que recibe del nodo publicador

import rclpy
from rclpy.node import Node
from interaccion_interfaces.srv import Multiplicador # Importar el servicio Multiplicador
from std_msgs.msg import String # Importar el mensaje String
from interaccion_interfaces.msg import Usuario # Importar el mensaje Usuario

class DialogoNodo(Node):

    # Constructor del nodo
    def __init__(self):
        # Inicializar el nodo con el nombre "dialogo_nodo"
        super().__init__('dialogo_nodo')
        # Crear una suscripción al tópico "user_topic" con el mensaje Usuario y el método user_callback
        self.user_subscription = self.create_subscription(Usuario, 'user_topic', self.user_callback, 10)
        # Crear un cliente para el servicio Multiplicador
        self.multiplicador_client = self.create_client(Multiplicador, 'servicio_multiplicador')

        # Imprimir mensaje de inicio
        self.get_logger().info("Dialogo nodo iniciado")

    # Método que se ejecuta cada vez que se recibe un mensaje en el tópico "user_topic"
    def user_callback(self, msg):
        # Esperar el resultado del servicio
        self.get_logger().info("Información personal: {} , Emoción: {} , Posición: {}".format(msg.inf_personal, msg.emocion, msg.posicion))

        # Crear una solicitud para el servicio Multiplicador
        request = Multiplicador.Request()
        request.entrada = msg.inf_personal.edad

        # Enviar la solicitud al servicio Multiplicador
        future = self.multiplicador_client.call_async(request)

        # Manejar la respuesta del servicio
        future.add_done_callback(self.multiplicador_callback)

    # Método que se ejecuta cuando se recibe una respuesta del servicio Multiplicador
    def multiplicador_callback(self, future):
        try:
            # Obtener la respuesta del servicio
            response = future.result()
            self.get_logger().info("Edad multiplicada: {}".format(response.resultado))
        except Exception as e:
            # Mostrar un mensaje de error si el servicio no responde
            self.get_logger().info("Error al llamar al servicio Multiplicador: {}".format(e))

# Método principal
def main(args=None):
    # Inicializar el nodo y crear instancia de la clase
    rclpy.init(args=args)
    dialogo_nodo = DialogoNodo()

    try:
        rclpy.spin(dialogo_nodo) # Ejecución secuencial de callbacks
    except KeyboardInterrupt:
        pass

    # Destruir el nodo y finalizar la ejecucion
    dialogo_nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
