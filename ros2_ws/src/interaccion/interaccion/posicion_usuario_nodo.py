## Nodo publicador encargado de pedir información por teclado sobre la posición del usuario

import rclpy
from rclpy.node import Node
from interaccion_interfaces.msg import PosUsuario # Importar el mensaje que definirá el publicador
import threading
import time

class PosicionUsuarioNodo(Node):
    # Constructor de la clase PosicionUsuarioNodo
    def __init__(self):
        # Inicializar el nodo
        super().__init__('posicion_usuario_nodo')

        # Establecer coordenadas predeterminadas
        self.coordenada_x = self.declare_parameter('coordenada_x', 0).value
        self.coordenada_y = self.declare_parameter('coordenada_y', 0).value
        self.coordenada_z = self.declare_parameter('coordenada_z', 0).value

        # Crear publicador
        self.pos_usuario_publisher = self.create_publisher(PosUsuario, 'pos_usuario_topic', 10)

        # Imprimir mensaje de inicio
        self.get_logger().info("Posición usuario nodo iniciado con coordenadas por defecto: x={}, y={}, z={}".format(self.coordenada_x, self.coordenada_y, self.coordenada_z))

        # Crear mensaje y publicar en el tópico con coordenadas predeterminadas
        pos_usuario_msg = PosUsuario()
        pos_usuario_msg.x = self.coordenada_x
        pos_usuario_msg.y = self.coordenada_y
        pos_usuario_msg.z = self.coordenada_z

        self.pos_usuario_publisher.publish(pos_usuario_msg)

        # Iniciar hilo para la entrada continua desde el teclado
        self.thread = threading.Thread(target=self.continuous_input)
        self.thread.daemon = True
        self.thread.start()

    # Solicitar continuamente al usuario una entrada y actualizar valores
    def continuous_input(self):
        while True:
            try:
                # Solicitar nuevas coordenadas al usuario
                x = int(input('Ingrese la nueva coordenada x (presione Enter para mantener la actual): ') or self.coordenada_x)
                y = int(input('Ingrese la nueva coordenada y (presione Enter para mantener la actual): ') or self.coordenada_y)
                z = int(input('Ingrese la nueva coordenada z (presione Enter para mantener la actual): ') or self.coordenada_z)

                # Actualizar las coordenadas
                self.coordenada_x = x
                self.coordenada_y = y
                self.coordenada_z = z

            except ValueError as e:
                self.get_logger().warn("Error al procesar la entrada: {}".format(str(e)))

    # Publicar continuamente en el mensaje los valores actuales
    def publish_values(self):
        while True:
            pos_usuario_msg = PosUsuario()
            pos_usuario_msg.x = self.coordenada_x
            pos_usuario_msg.y = self.coordenada_y
            pos_usuario_msg.z = self.coordenada_z

            self.pos_usuario_publisher.publish(pos_usuario_msg)
            time.sleep(0.1)  # Frecuencia de publicación de 10 Hz

# Punto inicial del programa
def main(args=None):
    # Inicializar el nodo y crear instancia de la clase
    rclpy.init(args=args)
    posicion_usuario_nodo = PosicionUsuarioNodo()

    try:
        # Iniciar hilo para la publicación continua
        publish_thread = threading.Thread(target=posicion_usuario_nodo.publish_values)
        publish_thread.daemon = True
        publish_thread.start()

        rclpy.spin(posicion_usuario_nodo) # Ejecución secuencial de callbacks

    except KeyboardInterrupt:
        pass

    # Destruir nodo y finalizar la ejecucion
    posicion_usuario_nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
