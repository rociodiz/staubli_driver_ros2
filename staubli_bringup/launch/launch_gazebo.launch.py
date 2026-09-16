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

# Gazebo Sim launch for Staubli TX2-60L with gz_ros2_control.
#
# Architecture:
#   1. robot_state_publisher publishes /robot_description (URDF with <ros2_control> + <gazebo> plugin)
#   2. GzServer starts empty world (ground plane + sun) — default empty.sdf
#   3. ros_gz_sim create reads /robot_description topic, spawns model in gz-sim
#      → URDF→SDF conversion preserves <gazebo><plugin> block → gz_ros2_control plugin loads
#   4. Plugin creates controller_manager internally at /controller_manager
#   5. Clock bridge provides /clock (gz→ROS) for use_sim_time
#   6. Spawners connect to /controller_manager to load controllers
#
# Usage:
#   ros2 launch staubli_bringup launch_gazebo.launch.py robot_model:=tx2_60l
#
# To visualize:
#   gz sim -g
#
# To verify:
#   ros2 control list_controllers
#   ros2 topic echo /joint_states

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ros_gz_sim.actions import GzServer


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_model",
            default_value="tx2_60l",
            description="Model of the robot (e.g, 'tx2_60l').",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation (Gazebo) clock if true.",
            choices=["true", "false"],
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="false",
            description="Launch Gazebo GUI (gz sim -g) if true.",
            choices=["true", "false"],
        )
    )

    # --- Robot description ---

    description_file = PathJoinSubstitution(
        [FindPackageShare("staubli_robot_description"), "urdf", "staubli.urdf.xacro"]
    )

    controllers_yaml = PathJoinSubstitution(
        [FindPackageShare("staubli_bringup"), "config", "controllers.yaml"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            description_file,
            " robot_model:=",
            LaunchConfiguration("robot_model"),
            " use_mock_hardware:=false",
            " use_gazebo_hardware:=true",
            " gazebo_controllers_file:=",
            controllers_yaml,
        ]
    )

    robot_description = {
        "robot_description": ParameterValue(value=robot_description_content, value_type=str),
    }

    # --- Robot state publisher ---

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            robot_description,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        output="both",
    )

    # --- Gazebo Sim server (inspection cell world) ---
    # GzServer action (ros_gz_sim) launches the gzserver node.
    # The gzserver node (ros_gz_sim::GzServer) requires a non-empty
    # world_sdf_file or world_sdf_string: with both empty it aborts with
    # "Must specify either 'world_sdf_file' or 'world_sdf_string'".
    # inspection_cell.sdf contains the ground plane, sun and the static
    # fixtures of the scene (inspection table + three workpieces). The
    # robot is spawned separately from /robot_description below.
    # GzServer also sets GZ_SIM_RESOURCE_PATH and GZ_SIM_SYSTEM_PLUGIN_PATH.

    inspection_cell_sdf = PathJoinSubstitution(
        [FindPackageShare("staubli_bringup"), "worlds", "inspection_cell.sdf"]
    )

    gz_server = GzServer(
        world_sdf_file=inspection_cell_sdf,
        create_own_container="False",
        use_composition="False",
    )

    # --- Spawn robot from /robot_description topic ---
    # The create node subscribes to /robot_description (transient_local),
    # receives the URDF string, and calls gz-sim's /world/<name>/create
    # service. gz-sim converts URDF→SDF via sdformat internally. The
    # <gazebo><plugin> block survives the conversion as a model-level
    # <plugin>, so gz_ros2_control loads and creates the controller_manager.
    # Spawned at X=0, Y=0, Z=0.30 to sit on top of the robot_platform box
    # (0.30 m tall) in inspection_cell.sdf.

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "/robot_description",
            "-name", "staubli",
            "-z", "0.30",
            "-allow_renaming", "true",
        ],
        output="screen",
    )

    # --- Clock bridge (/clock gz→ROS) ---
    # gz-sim publishes /clock on the gz transport (via SceneBroadcaster).
    # parameter_bridge forwards it to the ROS /clock topic. This is required
    # for every ROS node running with use_sim_time:=true (robot_state_publisher,
    # spawners, controller_manager) to be synchronized with the simulation.

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    # --- Controller spawners ---
    # The gz_ros2_control plugin creates the controller_manager node at
    # /controller_manager (default controller_manager_name and namespace "/").
    # The controllers YAML is loaded by the controller_manager itself via the
    # <parameters> tag in the SDF plugin block, so it already knows the
    # controller definitions. --controller-manager-timeout waits until the CM
    # service is available (the plugin needs robot_description to initialize
    # the ResourceManager before the CM becomes responsive).

    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "-c", "/controller_manager",
            "--controller-manager-timeout", "30",
        ],
        output="screen",
    )

    load_joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "-c", "/controller_manager",
            "--controller-manager-timeout", "30",
            "--param-file", controllers_yaml,
        ],
        output="screen",
    )

    # --- Gazebo GUI (optional) ---

    gz_gui = ExecuteProcess(
        cmd=["gz", "sim", "-g"],
        output="screen",
        condition=IfCondition(LaunchConfiguration("gui")),
    )

    # --- Log info ---

    log_info = LogInfo(
        msg=[
            "\n\033[32mGazebo Sim launch started.\033[0m",
            "\n  To visualize: gz sim -g (or use gui:=true)",
            "\n  To check controllers: ros2 control list_controllers",
            "\n  To echo joint states: ros2 topic echo /joint_states",
            "\n  To send a trajectory goal: ros2 action send_goal "
            "/joint_trajectory_controller/follow_joint_trajectory "
            "control_msgs/action/FollowJointTrajectory '{...}'",
            "\n",
        ]
    )

    return LaunchDescription(
        declared_arguments
        + [
            log_info,
            robot_state_publisher_node,
            gz_server,
            clock_bridge,
            spawn_robot,
            load_joint_state_broadcaster,
            load_joint_trajectory_controller,
            gz_gui,
        ]
    )
