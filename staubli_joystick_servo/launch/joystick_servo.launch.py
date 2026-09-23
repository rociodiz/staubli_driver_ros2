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

# Arranque del control cartesiano por velocidad (TX2-60L, sin MoveIt).
#
#   ros2 launch staubli_joystick_servo joystick_servo.launch.py
#
# Requiere: simulación de Gazebo con forward_velocity_controller activo.
# Entradas:  /cmd_vel (base_link)  |  /joint_states
# Salida:    /forward_velocity_controller/commands

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation (Gazebo) clock if true.",
            choices=["true", "false"],
        )
    )

    params_file = PathJoinSubstitution(
        [FindPackageShare("staubli_joystick_servo"), "config", "servo_params.yaml"]
    )

    servo_node = Node(
        package="staubli_joystick_servo",
        executable="servo_node",
        parameters=[
            params_file,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        output="screen",
    )

    return LaunchDescription(declared_arguments + [servo_node])