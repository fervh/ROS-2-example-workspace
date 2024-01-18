## Nodo publicador encargado de pedir información por teclado sobre el nombre, la edad y los idiomas del usuario

import rclpy
from rclpy.node import Node
import threading
import time
from interaccion_interfaces.msg import InfPersonalUsuario # Importar el mensaje que definirá el publicador

class InformacionPersonalNodo(Node):

    # Constructor de la clase InformacionPersonalNodo
    def __init__(self):
    	# Inicializar el nodo
        super().__init__('informacion_personal_nodo')

        # Establecer valores predeterminados de nombre, edad e idiomas
        self.nombre = self.declare_parameter('nombre', 'Pepe').value
        self.edad = self.declare_parameter('edad', 22).value
        self.idiomas = self.declare_parameter('idiomas', 'aleman,español'.split(',')).value

        # Crear publicador
        self.inf_pers_publisher = self.create_publisher(InfPersonalUsuario, 'inf_pers_topic', 10)

        # Imprimir mensaje de inicio
        self.get_logger().info("Información personal nodo iniciado con nombre: {}, edad: {} e idiomas: {}".format(self.nombre, self.edad, self.idiomas))

        # Crear mensaje y publicarlo
        inf_pers_msg = InfPersonalUsuario()
        inf_pers_msg.nombre = self.nombre
        inf_pers_msg.edad = self.edad
        inf_pers_msg.idiomas = self.idiomas

        self.inf_pers_publisher.publish(inf_pers_msg)

        # Iniciar hilo para la entrada continua desde el teclado
        self.thread = threading.Thread(target=self.continuous_input)
        self.thread.daemon = True
        self.thread.start()
	
    # Solicitar continuamente al usuario una entrada y actualizar valores
    def continuous_input(self):
        while True:
            try:
                # Solicitar nuevos valores
                nombre = input('Introduzca su nombre(presione Enter para mantener la actual): ') or self.nombre
                edad = int(input('Introduzca su edad(presione Enter para mantener la actual): ') or self.edad)
                idiomas = input('Introduzca sus idiomas separados por comas(presione Enter para mantener los actuales): ').split(',') or self.idiomas

                # Actualizar los valores
                self.nombre = nombre
                self.edad = edad
                self.idiomas = idiomas

            except ValueError as e:
                self.get_logger().warn("Error al procesar la entrada: {}".format(str(e)))
    
    # Publicar continuamente en el mensaje los valores actuales
    def publish_values(self):
        while True:
            inf_pers_msg = InfPersonalUsuario()
            inf_pers_msg.nombre = self.nombre
            inf_pers_msg.edad = self.edad
            inf_pers_msg.idiomas = self.idiomas

            self.inf_pers_publisher.publish(inf_pers_msg)
            time.sleep(0.1)  # Frecuencia de publicación de 10 Hz

# Punto inicial del programa
def main(args=None):
    # Inicializar el nodo y crear instancia de la clase
    rclpy.init(args=args)
    informacion_personal_nodo = InformacionPersonalNodo()

    try:
        # Iniciar hilo para la publicación continua si no hay interrupción de teclado
        publish_thread = threading.Thread(target=informacion_personal_nodo.publish_values)
        publish_thread.daemon = True
        publish_thread.start()

        rclpy.spin(informacion_personal_nodo) # Ejecución secuencial de callbacks

    except KeyboardInterrupt:
        pass
    
    # Destruir nodo y finalizar la ejecucion
    informacion_personal_nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
