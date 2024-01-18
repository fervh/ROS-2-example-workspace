## Script para lanzar todos los nodos del paquete

import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
    	# Lanzar el nodo de posición del usuario en ventana nueva
        ExecuteProcess(
            cmd=['konsole', '--new-tab', '-e', 'ros2', 'run', 'interaccion', 'posicion_usuario_nodo'],
            output='screen',
        ),
        # Lanzar el nodo de emoción del usuario en ventana nueva
        
        ExecuteProcess(
            cmd=['konsole', '--new-tab', '-e', 'ros2', 'run', 'interaccion', 'emocion_usuario_nodo'],
            output='screen',
        ),
	# Lanzar el nodo de información personal del usuario en ventana nueva
	
        ExecuteProcess(
            cmd=['konsole', '--new-tab', '-e', 'ros2', 'run', 'interaccion', 'informacion_personal_nodo'],
            output='screen',
        ),
        
        # Lanzar los demas nodos en ventana principal

        Node(
            package='interaccion',
            executable='empaquetador_nodo',
            name='empaquetador_nodo',
            output='screen',
        ),

        Node(
            package='interaccion',
            executable='dialogo_nodo',
            name='dialogo_nodo',
            output='screen',
        ),

        Node(
            package='interaccion',
            executable='matematico_nodo',
            name='matematico_nodo',
            output='screen'
        ),
    ])
