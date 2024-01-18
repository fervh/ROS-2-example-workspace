from setuptools import setup

package_name = 'interaccion'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/launcher.py']),
        ('share/' + package_name, ['launch/launcher_withoutdialogo.py']),
        ('share/' + package_name, ['launch/launcher_all.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fvela',
    maintainer_email='fvela@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Puntos de encuentro para identificar los nodos empleados
            'informacion_personal_nodo = interaccion.informacion_personal_nodo:main',
            'emocion_usuario_nodo = interaccion.emocion_usuario_nodo:main',
            'posicion_usuario_nodo = interaccion.posicion_usuario_nodo:main',
            'empaquetador_nodo = interaccion.empaquetador_nodo:main',
            'dialogo_nodo = interaccion.dialogo_nodo:main',
            'matematico_nodo = interaccion.matematico_nodo:main',
        ],
    },
)
