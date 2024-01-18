## Nodo publicador encargado de pedir información por teclado sobre la emoción del usuario

import rclpy
from rclpy.node import Node
from std_msgs.msg import String # Importar la estructura que definirá el publicador
import threading
import time

class EmocionUsuarioNodo(Node):
    
    # Constructor de la clase EmocionUsuarioNodo
    def __init__(self):
        # Inicializar el nodo
        super().__init__('emocion_usuario_nodo')

        # Establecer emoción predeterminada
        self.emocion = self.declare_parameter('emocion', "feliz").value

        # Crear publicador
        self.emocion_publisher = self.create_publisher(String, 'emocion_topic', 10)

        # Imprimir mensaje de inicio
        self.get_logger().info("Emoción por defecto: {}".format(self.emocion))

        # Crear mensaje y publicar en el tópico con emoción predeterminada
        emocion_msg = String()
        emocion_msg.data = self.emocion

        self.emocion_publisher.publish(emocion_msg)

        # Iniciar hilo para la entrada continua desde el teclado
        self.thread = threading.Thread(target=self.continuous_input)
        self.thread.daemon = True
        self.thread.start()

    # Solicitar continuamente al usuario una entrada y actualizar valores
    def continuous_input(self):
        while True:
            try:
                # Solicitar nueva emoción al usuario
                emocion = input('Ingrese la nueva emoción (presione Enter para mantener la actual): ') or self.emocion
                
                # Actualizar la emocion actual
                self.emocion = emocion

            except ValueError as e:
                self.get_logger().warn("Error al procesar la entrada: {}".format(str(e)))

    # Publicar continuamente en el mensaje los valores actuales
    def publish_values(self):
        while True:
            emocion_msg = String()
            emocion_msg.data = self.emocion

            self.emocion_publisher.publish(emocion_msg)
            time.sleep(0.1)  # Frecuencia de publicación de 10 Hz

# Punto inicial del programa
def main(args=None):
    # Inicializar el nodo y crear instancia de la clase
    rclpy.init(args=args)
    emocion_usuario_nodo = EmocionUsuarioNodo()

    try:
        # Iniciar hilo para la publicación continua si no hay interrupción de teclado
        publish_thread = threading.Thread(target=emocion_usuario_nodo.publish_values)
        publish_thread.daemon = True
        publish_thread.start()

        rclpy.spin(emocion_usuario_nodo) # Ejecución secuencial de callbacks

    except KeyboardInterrupt:
        pass

    # Destruir nodo y finalizar la ejecucion
    emocion_usuario_nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
