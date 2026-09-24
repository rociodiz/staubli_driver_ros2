"""Launch file de la interfaz gráfica del Staubli TX2-60L.

Lanza la GUI y, con ella, el nodo servidor del control manual
(staubli_joystick_servo) reutilizando su propio launch file: así TODO el
control manual (FRAME, cambio de controladores y trayectoria) queda
gestionado desde la interfaz sin comandos adicionales en la terminal.

Uso:
    ros2 launch staubli_gui gui.launch.py
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    joystick_servo_launch = PathJoinSubstitution(
        [
            FindPackageShare("staubli_joystick_servo"),
            "launch",
            "joystick_servo.launch.py",
        ]
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(joystick_servo_launch),
                launch_arguments={"use_sim_time": "true"}.items(),
            ),
            Node(
                package="staubli_gui",
                executable="gui_node",
                name="staubli_gui",
                output="screen",
            ),
        ]
    )
