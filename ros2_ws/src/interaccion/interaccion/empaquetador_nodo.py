## Nodo con 3 suscripciones y 1 publicación encargado de unificar la información sobre el usuario y enviarla en 1 único mensaje

import rclpy
from rclpy.node import Node
from interaccion_interfaces.msg import Usuario, InfPersonalUsuario, PosUsuario # Importar los mensajes necesarios
from std_msgs.msg import String

class EmpaquetadorNodo(Node):

    # Constructor de la clase EmpaquetadorNodo
    def __init__(self):
        # Inicializar el nodo
        super().__init__('empaquetador_nodo')
        # Crear el publicador del mensaje de Usuario
        self.user_publisher = self.create_publisher(Usuario, 'user_topic', 10)
        
        # Imprimir mensaje de inicio
        self.get_logger().info("Empaquetador nodo iniciado")

        # Mensajes recibidos
        self.inf_personal_msg = InfPersonalUsuario()
        self.emocion_msg = String()
        self.pos_usuario_msg = PosUsuario()

        # Flags para indicar si se han recibido los mensajes
        self.inf_personal_received = False
        self.emocion_received = False
        self.pos_usuario_received = False

        # Suscripciones a los tópicos correspondientes
        self.create_subscription(InfPersonalUsuario, 'inf_pers_topic', self.inf_personal_callback, 10)
        self.create_subscription(String, 'emocion_topic', self.emocion_callback, 10)
        self.create_subscription(PosUsuario, 'pos_usuario_topic', self.pos_usuario_callback, 10)

    # Callback de la suscripción a inf_personal_topic
    def inf_personal_callback(self, msg):
        # Actualizar el mensaje y la bandera
        self.inf_personal_msg = msg
        self.inf_personal_received = True
        # Llamar a la función para publicar el mensaje de Usuario
        self.publish_user_message()

    # Callback de la suscripción a emocion_topic
    def emocion_callback(self, msg):
        # Actualizar el mensaje y la bandera
        self.emocion_msg = msg
        self.emocion_received = True
        # Llamar a la función para publicar el mensaje de Usuario
        self.publish_user_message()

    # Callback de la suscripción a pos_usuario_topic
    def pos_usuario_callback(self, msg):
        # Actualizar el mensaje y la bandera
        self.pos_usuario_msg = msg
        self.pos_usuario_received = True
        # Llamar a la función para publicar el mensaje de Usuario
        self.publish_user_message()
    
    # Función para publicar el mensaje de Usuario
    def publish_user_message(self):
        # Verificar si se han recibido todos los mensajes
        if self.inf_personal_received and self.emocion_received and self.pos_usuario_received:
            # Crear el mensaje de Usuario
            user_msg = Usuario()
            user_msg.inf_personal = self.inf_personal_msg
            user_msg.emocion = self.emocion_msg.data
            user_msg.posicion = self.pos_usuario_msg

            # Publicar el mensaje en el tópico
            self.user_publisher.publish(user_msg)

            # Reiniciar los flags y los mensajes
            self.inf_personal_received = False
            self.emocion_received = False
            self.pos_usuario_received = False

# Punto inicial del programa
def main(args=None):
    # Inicializar el nodo y crear instancia de la clase
    rclpy.init(args=args)
    empaquetador_nodo = EmpaquetadorNodo()

    try:
        rclpy.spin(empaquetador_nodo) # Ejecución secuencial de callbacks
    except KeyboardInterrupt:
        pass

    # Destruir nodo y finalizar la ejecucion
    empaquetador_nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
