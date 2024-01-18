# ROS-2 Example Workspace

Este repositorio contiene un ejemplo de un sistema ROS 2 desarrollado como trabajo final para la asignatura de Sistemas Operativos de Robots en la Universidad Carlos III de Madrid (UC3M). El proyecto está estructurado en el workspace `ros2_ws` y la carpeta `rosbag_files`. A continuación, se detallan los nodos y la funcionalidad del sistema, así como las instrucciones para ejecutarlo.

## Estructura del Repositorio

- `ros2_ws`: Contiene el espacio de trabajo ROS 2 con los paquetes necesarios.
- `rosbag_files`: Carpeta para almacenar archivos de registro ROSBag.

## Estructura del Workspace

- **interaccion:**
  Contiene el paquete principal con todos los nodos y launchers necesarios.

- **interaccion_interfaces:**
  Contiene los mensajes y servicios personalizados utilizados en el sistema.


## Instrucciones de Ejecución

1. Clona este repositorio en tu sistema:

```bash
git clone https://github.com/fervh/ROS-2-example-workspace.git
```

2. Navega al directorio del workspace:

```bash
cd ROS-2-example-workspace/ros2_ws
```

3. Compila el workspace usando colcon desde la carpeta del workspace:

```bash
colcon build
```

4. Recarga las variables de entorno:

```bash
source install/setup.bash
```

5. Ejecuta los nodos con el launcher de tu elección:
      a. launcher.py (Ventana única):
  
      ```bash
      ros2 launch interaccion launcher.py
      ```
      
      b. launcher_all.py (Ventanas separadas)
   
      **Nota:** Se ha utilizado la aplicación Konsole en el launcher. Asegúrate de tenerla instalada en tu sistema.

      ```bash
      ros2 launch interaccion launcher_all.py
      ```

## Nodos del sistema

1. **informacion_personal_nodo:**
   - Solicita por teclado el nombre, la edad y los posibles idiomas que habla un usuario inventado. Puede introducirse información de varios usuarios.

2. **emocion_usuario_nodo:**
   - Similar al anterior, solicita indefinidamente por teclado y en bucle la emoción expresada por el usuario.

3. **posicion_usuario_nodo:**
   - Pide por teclado las coordenadas del usuario en el espacio. Se pueden introducir tantas veces como se desee.

4. **empaquetador_nodo:**
   - Combina la información recibida por los tres nodos anteriores en un único mensaje de ROS. No enviará el mensaje hasta que se reciban todos los campos necesarios. Si se reciben varios mensajes de un tipo antes de recibir todos los campos, se utilizará el último mensaje recibido.

5. **dialogo_nodo:**
   - Muestra por la terminal cada mensaje recibido del nodo empaquetador.
