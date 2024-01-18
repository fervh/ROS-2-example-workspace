## Script para lanzar todos los nodos del paquete

import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Lanzar el nodo de posición del usuario
        Node(
            package='interaccion', # Nombre del paquete
            executable='posicion_usuario_nodo', # Nombre del ejecutable
            name='posicion_usuario_nodo', # Nombre del nodo
            output='screen', # Mostrar salida en pantalla
            parameters=[ # Parámetros del nodo
                {'coordenada_x': 12},  # Valor predeterminado para la coordenada x
                {'coordenada_y': 23},  # Valor predeterminado para la coordenada y
                {'coordenada_z': 12},  # Valor predeterminado para la coordenada z
            ],
        ),
        # Lanzar el nodo de emoción del usuario
        Node(
            package='interaccion',
            executable='emocion_usuario_nodo',
            name='emocion_usuario_nodo',
            output='screen',
            parameters=[
                {'emocion': 'feliz'},  # Valor predeterminado para la emoción
            ],
        ),
        # Lanzar el nodo de información personal del usuario
        Node(
            package='interaccion',
            executable='informacion_personal_nodo',
            name='informacion_personal_nodo',
            output='screen',
            parameters=[
                {'nombre': 'Pepe'},    # Valor predeterminado para el nombre
                {'edad': 22},           # Valor predeterminado para la edad
                {'idiomas': 'aleman,español'.split(',')},  # Valor predeterminado para los idiomas
            ],
        ),
        # Lanzar el nodo de empaquetador
        Node(
            package='interaccion',
            executable='empaquetador_nodo',
            name='empaquetador_nodo',
            output='screen',
        ),
        # Lanzar el nodo de matemático
        Node(
            package='interaccion',
            executable='matematico_nodo',
            name='matematico_nodo',
            output='screen'
        ),
    ])
