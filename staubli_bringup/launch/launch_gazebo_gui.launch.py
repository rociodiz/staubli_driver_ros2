# Copyright 2025 ICube Laboratory
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Arranque COMPLETO de la celda: Gazebo Sim (robot Staubli + inspector de
# piezas) + ventana de Gazebo + GUI de control (que a su vez lanza el servo
# del joystick).
#
#   ros2 launch staubli_bringup launch_gazebo_gui.launch.py robot_model:=tx2_60l
#
# Con un solo comando la simulación queda lista para trabajar desde la GUI:
# Gazebo (servidor + ventana), los controladores con joint_trajectory_controller
# activo y forward_velocity_controller INACTIVO (modo TRAYECTORIA), el servo del
# joystick y la interfaz. La GUI cambia de controlador automáticamente al
# activar el joystick y al enviar trayectorias: no hace falta tocar la terminal.
# Con gui:=false se lanza solo el servidor de Gazebo (sin ventana).

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Launch Gazebo GUI (gz sim -g) if true.",
            choices=["true", "false"],
        ),
    ]

    gazebo_launch = PathJoinSubstitution(
        [FindPackageShare("staubli_bringup"), "launch", "launch_gazebo.launch.py"]
    )
    gui_launch = PathJoinSubstitution(
        [FindPackageShare("staubli_gui"), "launch", "gui.launch.py"]
    )

    return LaunchDescription(
        declared_arguments
        + [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gazebo_launch),
                launch_arguments={
                    "robot_model": "tx2_60l",
                    "gui": LaunchConfiguration("gui"),
                }.items(),
            ),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(gui_launch)),
        ]
    )