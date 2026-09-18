"""Launch file de la interfaz gráfica del Staubli TX2-60L.

Uso:
    ros2 launch staubli_gui gui.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="staubli_gui",
                executable="gui_node",
                name="staubli_gui",
                output="screen",
            ),
        ]
    )
