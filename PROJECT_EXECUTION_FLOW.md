# PROJECT_EXECUTION_FLOW.md

Flujos de ejecución paso a paso del repositorio
`~/staubli_ws/src/staubli_driver_ros2`, rama `feature/joystick-control`
(commit `4d195ec`).

Todos los diagramas son ASCII. Las referencias `archivo:línea` remiten a ese
commit.

---

## Índice

1. [Preparación del entorno](#1-preparación-del-entorno)
2. [Flujo A — Simulación Gazebo + GUI (camino principal de la rama)](#2-flujo-a--simulación-gazebo--gui-camino-principal-de-la-rama)
3. [Flujo B — Control manual FRAME (velocidad)](#3-flujo-b--control-manual-frame-velocidad)
4. [Flujo C — Control de trayectoria (posición)](#4-flujo-c--control-de-trayectoria-posición)
5. [Flujo D — MoveIt (camino upstream)](#5-flujo-d--moveit-camino-upstream)
6. [Flujo E — Robot real (UDP + VAL3)](#6-flujo-e--robot-real-udp--val3)
7. [Máquina de estados de los controladores](#7-máquina-de-estados-de-los-controladores)
8. [Catálogo completo de interfaces](#8-catálogo-completo-de-interfaces)
9. [Recetas de comandos](#9-recetas-de-comandos)
10. [Verificación y diagnóstico](#10-verificación-y-diagnóstico)
11. [Modos de fallo conocidos](#11-modos-de-fallo-conocidos)

---

## 1. Preparación del entorno

```bash
# 1. Distribución ROS: en ESTA máquina es lyrical, NO jazzy (README/CI usan jazzy)
source /opt/ros/lyrical/setup.bash

# 2. Workspace con los dos repositorios
cd ~/staubli_ws
ls src/          # staubli_driver_ros2  y  staubli_experimental
                 # (staubli_experimental queda fuera del alcance de esta doc)

# 3. Compilar
colcon build --symlink-install
source install/setup.bash

# 4. (Solo para el robot real) Desplegar la aplicación VAL3
ros2 run staubli_robot_driver upload_val3_server.py
```

### 1.1 Parche de `gz_ros2_control` (prerrequisito de la simulación)

`patches/gz_ros2_control/fix_command_mode_switch.patch` **no** se aplica
automáticamente. Sin él, `gz_ros2_control` limpia todos los bits de control al
parar una interfaz (bug `&` en lugar de `|` en `gz_system.cpp:~784`).

```bash
# Aplicación manual sobre el código fuente de gz_ros2_control 3.0.8
cd ~/staubli_ws/ros-lyrical-gz-ros2-control-3.0.8   # checkout presente en el workspace
git apply /home/rocio/staubli_ws/src/staubli_driver_ros2/patches/gz_ros2_control/fix_command_mode_switch.patch
# Reconstruir y reinstalar el paquete
```

> El workspace local contiene ya los artefactos de esa compilación:
> `ros-lyrical-gz-ros2-control_3.0.8-1resolute.deb`, `.dsc`, `.tar.xz` y el
> directorio `ros-lyrical-gz-ros2-control-3.0.8`.

---

## 2. Flujo A — Simulación Gazebo + GUI (camino principal de la rama)

### 2.1 Comando

```bash
source ~/staubli_ws/install/setup.bash

ros2 launch staubli_bringup launch_gazebo_gui.launch.py robot_model:=tx2_60l
# equivalente, sin ventana de Gazebo:
ros2 launch staubli_bringup launch_gazebo_gui.launch.py robot_model:=tx2_60l gui:=false
```

> `launch_gazebo_gui.launch.py:58` **fija** `robot_model: "tx2_60l"` en el
> `include` hacia `launch_gazebo.launch.py`; el argumento `robot_model` de este
> nivel no se propaga realmente.

### 2.2 Árbol de inclusiones

```
launch_gazebo_gui.launch.py
│
├── IncludeLaunchDescription(launch_gazebo.launch.py)      gui:=<arg>
│
│   ├── Node        robot_state_publisher                  (robot_state_publisher)
│   │                xacro ... use_gazebo_hardware:=true
│   │                use_mock_hardware:=false
│   │                gazebo_controllers_file:=controllers.yaml
│   │                    │
│   │                    └── publica /robot_description  (TRANSIENT_LOCAL)
│   │
│   ├── GzServer     mundo = worlds/inspection_cell.sdf
│   │                create_own_container=False
│   │                use_composition=False
│   │
│   ├── Node        camera_image_bridge   (ros_gz_bridge/parameter_bridge)
│   │                /camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image
│   │
│   ├── Node        clock_bridge           (ros_gz_bridge/parameter_bridge)
│   │                /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock
│   │
│   ├── Node        spawn_robot            (ros_gz_sim/create)
│   │                -topic /robot_description -name staubli -z 0.30
│   │
│   ├── Node        spawner joint_state_broadcaster          (ACTIVO)
│   ├── Node        spawner joint_trajectory_controller      (ACTIVO)
│   ├── Node        spawner forward_velocity_controller      (--inactive)
│   │
│   └── ExecuteProcess  gz sim -g                          (si gui:=true)
│
└── IncludeLaunchDescription(staubli_gui/gui.launch.py)     sin argumentos
    │
    ├── IncludeLaunchDescription(joystick_servo.launch.py)  use_sim_time:=true
    │   └── Node  servo_node  (staubli_joystick_servo)
    │              + config/servo_params.yaml
    │
    └── Node  gui_node  (staubli_gui, nombre "staubli_gui")
```

### 2.3 Secuencia de arranque

```
 t0  launch arranca
      │
 t1  xacro se ejecuta (Command substitution) y se genera el URDF
      │   URDF contiene:
      │     <ros2_control type="system">
      │        <hardware><plugin>gz_ros2_control/GazeboSimSystem</plugin>
      │     + <gazebo><plugin filename="libgz_ros2_control-system.so">
      │           <parameters>$CTRL_YAML</parameters>
      │     + <sensor type="camera"> en camera_link  →  /camera/image_raw (gz)
      │     + visuales del indicador láser en camera_link
      │
 t2  robot_state_publisher publica /robot_description  (QoS TRANSIENT_LOCAL)
      │
 t3  GzServer carga inspection_cell.sdf:
      │     ground_plane, sun, inspection_table, robot_platform, part_1
      │     sistemas: Physics, UserCommands, SceneBroadcaster, Contact, Sensors(ogre2)
      │     El robot NO está en el fichero
      │
 t4  ros_gz_sim/create recibe /robot_description y llama al servicio
      │   /world/inspection_cell/create  con  name=staubli, z=0.30
      │   gz-sim convierte URDF→SDF; el bloque <gazebo><plugin> sobrevive
      │   como <plugin> a nivel de <model>
      │
 t5  El plugin GazeboSimROS2ControlPlugin carga controllers.yaml y crea
      │   internamente el nodo controller_manager en /controller_manager
      │
 t6  Los 3 spawners esperan (--controller-manager-timeout 30) a que el CM
      │   responda, y cargan/activan:
      │       joint_state_broadcaster      → active
      │       joint_trajectory_controller  → active
      │       forward_velocity_controller  → inactive   ◄── MODO TRAYECTORIA
      │
 t7  El reloj ya está bridged, así que /clock fluye a ROS y todos los nodos
      │   con use_sim_time:=true quedan sincronizados
      │
 t8  servo_node y gui_node arrancan
      │
 t9  Listo. Modo actual = "trayectoria"
```

### 2.4 Diagrama de la cadena de datos (simulación)

```
 ┌──────────────────────────── URDF (única fuente) ───────────────────────────┐
 │ xacro: staubli.urdf.xacro                                                    │
 │   ├── staubli_macro.xacro      → base_link, link_1..link_6, tool0            │
 │   ├── staubli_camera.xacro     → camera_link + sensor gazebo + láser        │
 │   └── staubli.r2c.xacro        → <ros2_control> + plugin GazeboSimSystem     │
 └──────────────────────────────────────┬──────────────────────────────────────┘
                                        │ /robot_description (String, latched)
                ┌───────────────────────┼────────────────────────┐
                ▼                       ▼                        ▼
     robot_state_publisher        ros_gz_sim/create        (otros suscriptores)
                │                       │
       publica /tf, /tf_static        │  servicio /world/inspection_cell/create
       (árbol TF con camera_link)     ▼
                                 ┌─────────┐
                                 │  gz-sim │  GzServer
                                 └────┬────┘
                                      │
      ┌───────────────────────────────┼──────────────────────────────┐
      │                               │                              │
      ▼                               ▼                              ▼
/clock (gz)                  /camera/image_raw (gz)        plugins de sistema
      │                               │                    Physics/Sensors/...
      │ parameter_bridge              │ parameter_bridge
      ▼                               ▼
   /clock  (ros)              /camera/image_raw (ros, sensor_msgs/Image)
      │                               │
      │  use_sim_time de             │  QoS sensor_data (BEST_EFFORT)
      │  robot_state_publisher,       │
      │  servo_node, GUI              ▼
      │                        staubli_gui  CameraView.set_frame(QImage)
      ▼
  (relooj compartido)

 ┌──────────────┐   /joint_states   ┌────────────────┐
 │ GazeboSim    │ ─────────────────▶ │  GUI            │
 │ System       │                    │  servo_node     │
 │ (ros2_control│ ◀─ q̇ / q_cmd ──── │  MoveIt         │
 │  hardware)   │                    └────────────────┘
 └──────┬───────┘
        │ carga controllers.yaml
        ▼
 ┌──────────────────────────────────────────────────────────────┐
 │ controller_manager  (update_rate = 250 Hz)                    │
 │   ├── joint_state_broadcaster      → /joint_states           │
 │   ├── joint_trajectory_controller  → /follow_joint_trajectory│
 │   └── forward_velocity_controller  → /commands (velocity)    │
 └──────────────────────────────────────────────────────────────┘
```

### 2.5 El indicador láser: no es un marcador ROS

```
 camera_link (en el árbol TF del robot)
   │
   ├── visual: laser_triangle_visual
   │      mesh  laser_triangle.obj
   │      scale = (laser_distance, laser_width, 1.0) = (0.50, 0.30, 1.0)
   │      color  = rgba(0.10, 0.70, 1.00, 0.25)   ← cian, 25% opaco
   │
   └── visual: laser_line_visual
          box   size = (0.008, 0.30, 0.008)
          origin xyz = (0.50, 0, 0)
          color  = rgba(1.00, 0.00, 0.00, 0.95)   ← rojo, 95% opaco

 Mesh (laser_triangle.obj):
   v 0.0  0.0  0.0
   v 1.0 -0.5  0.0
   v 1.0  0.5  0.0
   f 1 2 3                     (una sola cara triangular)
```

Consecuencia práctica: el triángulo y la línea **se mueven con el robot** porque
son links de URDF. No hay nodo, ni publicador, ni topic. Se ven en Gazebo
(`Sensors` + ogre2) y en RViz. Grep de `visualization_msgs` /
`Marker` / `MarkerArray` en el repositorio: **0 coincidencias**.

Ajustable en caliente vía argumentos Xacro:

```bash
ros2 launch staubli_bringup launch_gazebo.launch.py \
    robot_model:=tx2_60l laser_distance:=0.35 laser_width:=0.20 laser_line_width:=0.006
```
> `launch_gazebo.launch.py:90-102` **no** propaga estos argumentos al `Command`
> de xacro, así que esta vía no funciona tal cual. Solo se pueden cambiar
> editando el Xacro o pasando un `Command` propio.

---

## 3. Flujo B — Control manual FRAME (velocidad)

### 3.1 Objetivo

Convertir un comando cartesiano en `base_link` (`/cmd_vel`) en velocidades
articulares, **sin MoveIt**, y publicarlas en el topic del
`forward_velocity_controller`.

### 3.2 Activación desde la GUI

```
 Usuario pulsa "ACTIVAR MANUAL"
        │
        ▼
 MainWindow._on_manual_toggle(active=True)          gui.py:1053
        │
        ▼
 MainWindow._switch_to_mode("manual", ...)          gui.py:984
        │
        ▼
 ControllerManager.switch(                          controller_manager.py:79
     activate=("forward_velocity_controller",
               "joint_state_broadcaster"),
     deactivate=("joint_trajectory_controller",))
        │
        ├─(1) refresh() → ListControllers  (estado real)
        │
        ├─(2) filtrado (STRICT no es idempotente):
        │      activar   = los que NO están "active"
        │      desactivar= los que SÍ  están "active"
        │
        ├─(3) SwitchController(strictness=STRICT, activate_asap=False, timeout=5s)
        │        → o se cambia todo, o no se cambia nada
        │
        └─(4) refresh() de verificación → confirma el estado final
        │
        ▼
 _on_mode_switch_done("manual", ok, states, message)  gui.py:1030
        │
        ├─ si !ok  → NO se publica /cmd_vel; se registra el error
        └─ si ok   → self._manual_active = True
                    log "CONTROL MANUAL ACTIVADO: publicando /cmd_vel"
                    _update_mode_display("manual")
        │
        ▼
 El usuario pulsa los botones ± del panel FRAME
        │
        ▼
 MainWindow._publish_manual_twist()                  gui.py:842
        │  (solo si self._manual_active)
        ▼
 publica geometry_msgs/Twist en /cmd_vel  (depth 10)
```

### 3.3 Cadena del servo

```
 /cmd_vel  (geometry_msgs/Twist, base_link)
     │  GUI, solo con control manual activo
     ▼
 ┌──────────────────────────────────────────────────────────────────┐
 │ staubli_joystick_servo/servo_node.py :: JoystickServoNode        │
 │                                                                  │
 │  timer a 250 Hz                                                 │
 │                                                                  │
 │  (1) ¿/joint_states fresco?  ── no ──▶ limiter.reset()           │
 │        │ sí                                    + publicar [0]*6  │
 │        ▼                                                        │
 │  (2) ¿/cmd_vel fresco?       ── no ──▶ limiter.force_zero()      │
 │        │ sí                                    + publicar       │
 │        ▼                                                        │
 │  (3) apply_deadzone(lin, ang, 0.01, 0.01)                        │
 │        │                                                        │
 │        ├── Twist ≈ 0 ──▶ target = zeros(6)                       │
 │        │                    (frenado progresivo vía limiter)     │
 │        ▼                                                        │
 │  (4) clamp_vector(lin, 0.15 m/s)                                │
 │      clamp_vector(ang, 0.50 rad/s)                               │
 │        ▼                                                        │
 │  (5) twist_vec = [lin.x, lin.y, lin.z, ang.x, ang.y, ang.z]      │
 │        │                                                        │
 │        ▼                                                        │
 │  (6) jac = Tx260lKinematics.jacobian(q)        6x6, tool0/base  │
 │        │         (geometría compartida con staubli_trajectory)  │
 │        ▼                                                        │
 │  (7) q̇, σmin, U, S = solve_qdot_dls(                            │
 │           J, twist_vec, λ=0.05, σ_thr=0.02)                      │
 │                                                                 │
 │         q̇ = Jᵀ (J Jᵀ + λ² I)⁻¹ v     ← Damped Least Squares    │
 │         Si σmin < σ_thr → mayor amortiguamiento efectivo         │
 │                                                                 │
 │        ▼                                                        │
 │  (8) scale_to_joint_velocity_limits(q̇, JOINT_VELOCITY_LIMITS_RAD)│
 │        │                                                        │
 │        ▼                                                        │
 │  (9) out = JointVelocityLimiter.update(target)                   │
 │           rampa de aceleración/desaceleración (1.5 rad/s²)      │
 │        ▼                                                        │
 └────────┬─────────────────────────────────────────────────────────┘
          ▼
 /forward_velocity_controller/commands  (std_msgs/Float64MultiArray, depth 10)
          │   6 valores en rad/s: [joint_1 ... joint_6]
          ▼
 forward_velocity_controller → GazeboSimSystem → q̇ en Gazebo
```

### 3.4 Watchdogs y paradas de emergencia

| Disparador | Tiempo | Acción | Código |
|---|---|---|---|
| Sin `/joint_states` al arrancar | inmediato | `limiter.reset()` + ceros | `servo_node.py:140-144` |
| `/joint_states` obsoleto | > `joint_state_timeout` (0.2 s) | `limiter.reset()` + ceros | `servo_node.py:141-144` |
| Sin `/cmd_vel` | > `command_timeout` (0.2 s) | `limiter.force_zero()` | `servo_node.py:147-150` |
| Twist dentro de deadzone | inmediato | frenado progresivo a cero | `servo_node.py:162-163` |
| `Ctrl+C` en el nodo | en el `finally` | `node.stop()` → ceros | `servo_node.py:195` |
| Botón STOP de la GUI | inmediato | `_send_zero_twist()` + cambia a modo trayectoria | `gui.py:1114`, `gui.py:1998` |
| Salir del control manual | inmediato | `Twist = 0` + restaura JTC | `gui.py:45-49` |

---

## 4. Flujo C — Control de trayectoria (posición)

### 4.1 Dos caminos de entrada

```
 CAMINO 1 — nodo autónomo (staubli_trajectory)
   ros2 launch staubli_trajectory trajectory.launch.py [send:=true]

 CAMINO 2 — desde la GUI
   botón "RUN" (gui.py:1964) / "IR A P1|P2|P3" / puntos de usuario
```

### 4.2 Camino 1: nodo autónomo

```
 config/points.yaml
     │  (única zona a editar; ver cabecera del fichero)
     ▼
 PointsConfig(config_path)                    points_config.py:38
     · joint_names, point_type, planning_frame
     · orientation_mode, part_center_base_link
     · lista de puntos, tiempos, tolerancias
     · _validate()                            points_config.py:100
     ▼
 TrajectoryBuilder(config)                    trajectory_builder.py:58
     │
     ├── si point_type == "cartesian":        trajectory_builder.py:94
     │     1. orientation_frame_looking_at(part_center_base_link)
     │        (o quaternion_xyzw explícito si el punto lo fija)
     │     2. frame_from_position_quaternion(pose)
     │     3. kinematics.inverse_kinematics(frame)   tx2_60l_ik.py:224
     │        multi-semilla: random_seed=42, n_random_seeds=30
     │     4. _check_solution → FK + _IK_POS_EPS(1e-4) / _IK_ROT_EPS(1e-3)
     │     5. check_joint_limits → ok, flags_por_articulación
     │     6. WaypointRecord(q[6], goal, ik_ok, flags, tiempo)
     │
     ├── si point_type == "joint":            trajectory_builder.py:154
     │     diagnóstico local: posiciones en rad, sin IK
     │
     └── _to_joint_trajectory(records)        trajectory_builder.py:177
           → trajectory_msgs/JointTrajectory con timestamps
     ▼
 TrajectoryPlan
     │
     ├── SIN --send  (por defecto)
     │     └─▶ imprime el plan y termina. NO mueve el robot.
     │
     └── CON --send  (send:=true)
           ▼
 JointTrajectoryClient(node)                  joint_trajectory_client.py:36
     │  wait_for_server(timeout=10 s)          joint_trajectory_client.py:54
     │  _build_goal(trajectory, config)       joint_trajectory_client.py:117
     │     · goal.trajectory
     │     · JointTolerance por articulación
     │       (goal_position_tolerance,
     │        goal_velocity_tolerance,
     │        goal_acceleration_tolerance)
     │     · goal_time_tolerance
     │  send(trajectory, config)              joint_trajectory_client.py:66
     │     _on_feedback → imprime índice de waypoint y posiciones reales
     ▼
 /joint_trajectory_controller/follow_joint_trajectory
     (control_msgs/action/FollowJointTrajectory)
     ▼
 joint_trajectory_controller (ya activo) → GazeboSimSystem → robot
```

### 4.3 Camino 2: desde la GUI

```
 Usuario: selecciona P1/P2/P3, o un punto de usuario, o una lista ordenada
     │
     ├── _on_punto(name)                       gui.py:1161
     │     └─▶ _load_point_plan()              gui.py:1909
     │           from staubli_trajectory.points_config import PointsConfig   (:1919)
     │           from staubli_trajectory.trajectory_builder import TrajectoryBuilder (:1920)
     │           config = PointsConfig(str(config_path))                      (:1924)
     │           plan  = TrajectoryBuilder(config).build()                    (:1925)
     │
     ├── _build_single_goal()                  gui.py:1710
     ├── _build_trajectory_goal()              gui.py:1727
     └── _build_user_trajectory_goal()         gui.py:1734
           (usa las posiciones articulares ya resueltas por TrajectoryBuilder,
            sin IK propia — gui.py:203)
     ▼
 _fill_goal_tolerances(goal)                   gui.py:1757
     ▼
 _run_goal(...)  /  _on_run()                   gui.py:1633 / :1964
     │
     ├─ Precondición: ¿el modo actual es "trayectoria"?
     │     NO → _switch_to_mode("trayectoria") y esperar al callback
     │           _tras_switch(ok, message)      gui.py:1670
     │
     ├─ _pause_status_poll()                    gui.py:1897
     │     (deja de consultar list_controllers durante el goal)
     │
     └─ ActionClient.send_goal_async(goal)
             ▼
     _on_motion_goal_response()                gui.py:1770
     _on_motion_feedback()                     gui.py:1798
         └─▶ RESALTA el waypoint completado     (commit 6cd1471)
     _on_motion_result()                       gui.py:1834
         ├─ SUCCESSFUL → _finish_motion(texto, color)   gui.py:1884
         └─ otro      → _finish_motion(error,   rojo)
     _resume_status_poll()                     gui.py:1902
```

### 4.4 Configuración de `points.yaml` relevante para la celda

```yaml
joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]
point_type: cartesian
planning_frame: base_link
orientation_mode: look_at_part_center
part_center_base_link: [0.0, 0.66, 0.24]
```

Trazabilidad del valor:

```
 inspection_cell.sdf
   part_1 en world ≈ (0.0, 0.66, 0.54)
        │
        │  el robot se spawnea con -z 0.30  (launch_gazebo.launch.py:155)
        ▼
 base_link = world − (0, 0, 0.30)
   part_center_base_link = (0.0, 0.66, 0.54 − 0.30) = (0.0, 0.66, 0.24)
```

---

## 5. Flujo D — MoveIt (camino upstream)

### 5.1 Comando

```bash
ros2 launch staubli_bringup launch_demo.launch.py robot_model:=tx2_60l
# o, por partes:
ros2 launch staubli_bringup launch_robot_control.launch.py \
    robot_model:=tx2_60l use_mock_hardware:=true gui:=false \
    start_controller:=joint_trajectory_controller
ros2 launch staubli_bringup launch_moveit.launch.py robot_model:=tx2_60l
```

### 5.2 Secuencia de `launch_demo`

```
 launch_demo.launch.py
      │
      ├── Include(launch_robot_control.launch.py)
      │      use_sim_time: "false"
      │      start_controller: joint_trajectory_controller
      │      gui: "false"                    (RViz desactivado aquí)
      │        ├── robot_state_publisher  (xacro, use_mock_hardware:=true)
      │        ├── controller_manager/ros2_control_node
      │        │     + controllers.yaml   (update_rate 250 Hz)
      │        │     ⚠ use_sim_time=False HARDCODEADO (línea 144)
      │        ├── spawner joint_state_broadcaster     → active
      │        ├── spawner joint_trajectory_controller → inactive
      │        │     OnProcessExit → set_controller_state active
      │        └── rviz2  (desactivado por gui:=false)
      │
      └── Include(launch_moveit.launch.py)
             use_sim_time: "false", gui: "true", launch_servo: "false"
               │
               ├── Node  wait_for_robot_ready  (staubli_bringup)
               │      · sub /robot_description  QoS TRANSIENT_LOCAL
               │      · sub /joint_states       QoS VOLATILE
               │      · log cada 5 s con ✓/✗
               │
               └── OnProcessExit ──▶ Include(staubli_moveit_config/staubli_moveit.launch.py)
                      use_sim_time=false
                      warehouse_sqlite_path=~/.ros/warehouse_ros.sqlite
                      launch_servo=false
                      launch_rviz=true
                        │
                        ├── Node  move_group   (moveit_ros_move_group)
                        │      · MoveItConfigsBuilder(robot_name="staubli")
                        │      · robot_description_semantic desde srdf/staubli.srdf.xacro
                        │      · planning_scene_monitor
                        │      · warehouse_ros_sqlite::DatabaseConnection
                        │
                        ├── Node  servo_node   (CONDICIONAL launch_servo:=true)
                        │      ⚠ §11: command_out_topic inexistente
                        │
                        └── Node  rviz2_moveit  con config/moveit.rviz
```

### 5.3 Interfaz de MoveIt

```
 move_group
   ├─ planificador: OMPL (por defecto) y/o Pilz
   │     · pilz_cartesian_limits.yaml:
   │         max_trans_vel 1.0, max_trans_acc 2.25,
   │         max_trans_dec −5.0, max_rot_vel 1.57
   ├─ cinemática: KDLKinematicsPlugin
   │     · kinematics_solver_search_resolution 0.005
   │     · kinematics_solver_timeout 0.5
   ├─ límites: default_velocity_scaling_factor 0.1
   │           default_acceleration_scaling_factor 0.1
   │           joint_1..5 max_velocity 1.5 rad/s, max_acceleration 5.0
   │           joint_6   max_velocity 2.0 rad/s, max_acceleration 10.0
   ├─ warehouse: SQLite en ~/.ros/warehouse_ros.sqlite
   └─ ejecución: moveit_simple_controller_manager
         · joint_trajectory_controller
         · action_ns follow_joint_trajectory
         · type FollowJointTrajectory
         · default: false                (no se toma por defecto)
         · joints joint_1..joint_6
             ▼
         /joint_trajectory_controller/follow_joint_trajectory
```

> `allow_partial_joints_goal: false` en `controllers.yaml:34` implica que el
> JTC rechazará goals con subconjunto de articulaciones. MoveIt con
> `default: false` debe resolver el conflicto por su cuenta.

---

## 6. Flujo E — Robot real (UDP + VAL3)

### 6.1 Comando

```bash
# 1. Instalar la aplicación en el robot (una vez)
ros2 run staubli_robot_driver upload_val3_server.py

# 2. Lanzar el control
ros2 launch staubli_bringup launch_robot_control.launch.py \
    robot_model:=tx2_60l \
    robot_ip:=192.168.0.254 \
    use_mock_hardware:=false \
    gui:=true
```

### 6.2 Selección de backend

```
 xacro staubli.urdf.xacro
   use_mock_hardware:=false
   use_gazebo_hardware:=false
        │
        ▼ (r2c/staubli.r2c.xacro:39-58)
   <hardware>
     <plugin>staubli_robot_driver/StaubliHardwareInterface</plugin>
     <param name="robot_ip">192.168.0.254</param>
     <param name="control_port">11000</param>
     <param name="diagnostic_port">11001</param>
     <param name="local_control_port">11000</param>
     <param name="local_diagnostic_port">11001</param>
     <param name="num_digital_inputs">16</param>
     <param name="num_digital_outputs">16</param>
     <param name="num_analog_inputs">2</param>
     <param name="num_analog_outputs">2</param>
   </hardware>
   + interfaz de acceleration (solo hardware real)
   + gpio "supervision" (10 state interfaces)
   + gpio E/S digital y analogico
```

### 6.3 Ciclo del `controller_manager` (250 Hz)

```
 controller_manager (update_rate = 250 Hz, periodo = 4 ms)
        │
        ├── update() ──▶ StaubliHardwareInterface::write()
        │                    │
        │                    ├─ ¿conectado?          no ─▶ ERROR
        │                    ├─ ¿robot en error?      sí ─▶ send_stop_all + ERROR
        │                    ├─ construir RobotCommandMessage
        │                    │     header.sequence_number = get_current_message_sequence()
        │                    │     controller_period    = 4 (ms)
        │                    │     command_type         = STOP | JOINT_POSITION
        │                    │     command_reference[]  = rad2deg(q_cmd)
        │                    │     digital_outputs[]    = (cmd > 0.5)
        │                    │     analog_outputs[]     = cmd
        │                    ├─ ¿NaN en reference?    sí ─▶ send_stop_all + ERROR
        │                    └─ send_robot_command(cmd)
        │
        ├── read()  ──▶ StaubliHardwareInterface::read()
        │                    │
        │                    ├─ ¿conectado?          no ─▶ ERROR
        │                    ├─ read_robot_state(state_msg_)
        │                    │     ├─ is_ready()
        │                    │     ├─ read_message(current_robot_state, status)
        │                    │     ├─ ¿message_type == ROBOT_STATE?
        │                    │     ├─ ¿status.time_since_received < 100 ms?  no ─▶ ERROR
        │                    │     └─ state = current_robot_state
        │                    │        last_valid_sequence_id_ = state.header.sequence_number
        │                    │
        │                    └─ update_state(state_msg)
        │                          deg2rad() en posiciones y velocidades
        │                          pares tal cual
        │                          E/S + 10 campos de supervision
        │                          ¿faltan posiciones o velocidades? ─▶ ERROR
        │
        └── prepare/perform_command_mode_switch()
                 │
                 ├─ JOINT_POSITION → OK
                 │     perform: ¿error_state?      ─▶ ERROR (y forzar STOP)
                 │              ¿!motion_possible? ─▶ ERROR (frenos puestos)
                 │              operation_mode_status != 0 ─▶ WARN (HOLD)
                 │              copiar q_medido → q_cmd   (no el último comando)
                 │              q̇_cmd = 0, q̈_cmd = 0
                 │
                 ├─ JOINT_VELOCITY ─▶ ERROR  "Velocity control is not yet supported"
                 ├─ JOINT_TORQUE   ─▶ ERROR  "Torque/effort control is not yet supported"
                 └─ STOP / solo accel → OK (modo = STOP)
```

### 6.4 Protocolo UDP

```
 ROBOT CS9  ────────────── UDP ──────────────  HOST (staubli_robot_driver)
   │                                            │
   │  control  : 11000 ────────────────────────▶ │  RobotStateMessage   (0x01)
   │                                            │    periódicamente
   │                                            │
   │  ◀──────────────────────────────────────── │  RobotCommandMessage (0x02)
   │  command  : 11000                          │    a 250 Hz
   │                                            │
   │  diagnostic : 11001 ──────────────────────▶ │  DiagnosticDataMessage(0x03)
   │                                            │    ⚠ NO LEÍDO (ver §11.6)

 Formato de trama (8 bytes de cabecera):
  ┌─────────┬───────┬───────┬───────┬──────────┬─────────────┐
  │ Magic   │ Proto │ Type  │ Seq   │ Payload  │   Payload   │
  │ 2B u16  │ 1B u8 │ 1B u8 │ 2B u16│ 2B u16   │  variable   │
  │ 0xABCD  │   1   │       │       │  ⚠BUG    │             │
  └─────────┴───────┴───────┴───────┴──────────┴─────────────┘
                       ▲                ▲
                       │                └── encodeHeader.pgx escribe el
                       │                    SEQUENCE NUMBER en vez del
                       │                    payload size  (§11.3)
                       └── sequence_number
```

### 6.5 Asignación de puertos (⚠ dos fuentes en conflicto)

| Origen | Control remoto | Control local | Diagnóstico remoto | Diagnóstico local |
|---|---|---|---|---|
| `communication/protocol.hpp:45,50` (defaults C++) | 8080 | 8080 | 8081 | 8081 |
| `r2c/staubli.r2c.xacro:50-53` (**usado en producción**) | 11000 | 11000 | 11001 | 11001 |
| `test_receive_robot_state.cpp` (usa los defaults) | 8080 | 8080 | 8081 | 8081 |

El flujo de producción pasa por el Xacro, así que manda 11000/11001. El
ejecutable de diagnóstico manual usa 8080/8081.

---

## 7. Máquina de estados de los controladores

### 7.1 Diagrama

```
                     ┌──────────────────────┐
                     │       seguro         │
                     │  JSB active          │
                     │  JTC inactive        │
                     │  FVC inactive        │
                     └──────────┬───────────┘
                                │
                ┌───────────────┴───────────────┐
                │  switch(activate={JTC,JSB},   │
                │          deactivate={FVC})      │
                ▼                               ▲
     ┌──────────────────────┐                  │
     │     trayectoria      │──────────────────┘
     │  JTC active (posición)│   STRICT, activate_asap=False,
     │  FVC inactive        │   timeout 5 s, verificación posterior
     │  JSB active          │
     └──────────┬───────────┘
                │
                │  "ACTIVAR MANUAL"
                │  switch(activate={FVC,JSB},
                │         deactivate={JTC})
                ▼
     ┌──────────────────────┐
     │       manual         │
     │  FVC active (velocidad)│
     │  JTC inactive        │
     │  JSB active          │   GUI publica /cmd_vel
     └──────────┬───────────┘
                │
                │  "VOLVER A TRAYECTORIA"
                │  switch(activate={JTC,JSB},
                │         deactivate={FVC})
                └──────────▶ trayectoria
```

### 7.2 Transiciones y reglas

| Desde | Hacia | Acción | Publica `/cmd_vel` |
|---|---|---|---|
| `trayectoria` | `manual` | `activate={FVC,JSB}`, `deactivate={JTC}` | sí, tras confirmar |
| `manual` | `trayectoria` | `activate={JTC,JSB}`, `deactivate={FVC}`; antes envía `Twist=0` | no |
| cualquiera | `seguro` | `activate={JSB}`, `deactivate={JTC,FVC}` | no |
| `conflicto` | — | **estado prohibido** (JTC y FVC activos a la vez); la GUI lo detecta y lo reporta | — |
| `desconocido` | — | sin información fiable (CM no responde) | no |

### 7.3 Por qué 3 pasos y no 1 llamada

`controller_manager.py:12-14` y `:100-123`:

```
STRICT no es idempotente: pedir activar un controlador YA activo hace fallar
la llamada completa (y entonces NADA se cambia).

  1. list_controllers      → conocer el estado real
  2. filtrar              → activar   solo lo que NO está active
                            desactivar solo lo que SÍ  está active
  3. switch_controller    → UNA llamada con STRICT
                            (o se aplica todo, o no se aplica nada)
  4. list_controllers      → verificar el resultado
                            y confirmar que se alcanzó el estado deseado
```

Si no hay nada que cambiar, se responde `ok=True` con
*"los controladores ya estaban en el estado deseado"*.

---

## 8. Catálogo completo de interfaces

### 8.1 Topics

| Topic | Tipo | QoS | Publicador | Suscriptor | Productor |
|---|---|---|---|---|---|
| `/robot_description` | `std_msgs/String` | depth 1, **TRANSIENT_LOCAL** | `robot_state_publisher` | `ros_gz_sim/create`, MoveIt, `wait_for_robot_ready` | El URDF completo (Xacro) |
| `/joint_states` | `sensor_msgs/JointState` | depth 1, **VOLATILE** | `joint_state_broadcaster` | GUI, `servo_node`, MoveIt, `wait_for_robot_ready` | `controller_manager` a 250 Hz |
| `/clock` | `rosgraph_msgs/Clock` | — | gz-sim (`SceneBroadcaster`) | bridge → todos los nodos `use_sim_time` | Gazebo |
| `/camera/image_raw` (ROS) | `sensor_msgs/Image` (`rgb8`) | **sensor_data** (BEST_EFFORT, VOLATILE) | `camera_image_bridge` | GUI `CameraView` | `launch_gazebo.launch.py:183` |
| `/camera/image_raw` (gz) | `gz.msgs.Image` | gz | sensor Gazebo en `camera_link` | bridge | `staubli_camera.xacro` |
| `/cmd_vel` | `geometry_msgs/Twist` | depth 10 | GUI (solo si `_manual_active`) | `servo_node` | `gui.py:807` |
| `/forward_velocity_controller/commands` | `std_msgs/Float64MultiArray` | depth 10 | `servo_node` | `forward_velocity_controller` | `servo_node.py:96` |
| `/tf`, `/tf_static` | `tf2_msgs/*` | — | `robot_state_publisher` | RViz, MoveIt, `robot_state_publisher` | árbol URDF (incluye `camera_link`) |

### 8.2 Servicios

| Servicio | Tipo | Dirección | Notas |
|---|---|---|---|
| `/controller_manager/list_controllers` | `controller_manager_msgs/srv/ListControllers` | GUI → CM | async, ~1 Hz, solo para el indicador |
| `/controller_manager/switch_controller` | `controller_manager_msgs/srv/SwitchController` | GUI → CM | `STRICT`, `activate_asap=False`, `timeout=5 s` |
| `/world/inspection_cell/create` | `gz.msgs.EntityFactory` | `ros_gz_sim/create` → gz-sim | requiere que el CM exista para JTC |
| `action_msgs/srv/CancelGoal` | `action_msgs/srv/CancelGoal` | GUI → JTC | botón STOP (`gui.py:56`, `:2030`) |

### 8.3 Acciones

| Acción | Tipo | Cliente | Servidor | Goal |
|---|---|---|---|---|
| `/joint_trajectory_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | GUI, `JointTrajectoryClient` | `joint_trajectory_controller` | `trajectory` + `goal_tolerance[]` + `goal_time_tolerance` |

### 8.4 Nodos y procesos

| Nodo | Paquete | Ejecutable | Backend de hardware | `use_sim_time` |
|---|---|---|---|---|
| `robot_state_publisher` | `robot_state_publisher` | `robot_state_publisher` | — | arg del launch |
| `controller_manager` | `controller_manager` | `ros2_control_node` | `mock_components/GenericSystem` | **False hardcodeado** en `launch_robot_control` |
| `controller_manager` (en Gazebo) | `gz_ros2_control` | (creado por el plugin) | `GazeboSimSystem` | `true` (del reloj bridged) |
| `StaubliHardwareInterface` | `staubli_robot_driver` | (plugin) | UDP → CS9 | `false` |
| `servo_node` | `staubli_joystick_servo` | `servo_node` | — | `true` |
| `gui_node` | `staubli_gui` | `gui_node` | — | (heredado) |
| `trajectory_node` | `staubli_trajectory` | `trajectory_node` | — | (heredado) |
| `move_group` | `moveit_ros_move_group` | `move_group` | — | arg |
| `servo_node` (MoveIt) | `moveit_servo` | `servo_node` | — | ⚠ roto, ver §11.2 |
| `rviz2` / `rviz2_moveit` | `rviz2` | `rviz2` | — | arg |
| `wait_for_robot_ready` | `staubli_bringup` | `wait_for_robot_ready.py` | — | (heredado) |
| `spawner` ×3 | `controller_manager` | `spawner` | — | (heredado) |
| `camera_image_bridge`, `clock_bridge` | `ros_gz_bridge` | `parameter_bridge` | — | — |
| `ros_gz_sim` create | `ros_gz_sim` | `create` | — | — |
| `gzserver` / `gz sim -g` | Gazebo Sim | — | — | — |

---

## 9. Recetas de comandos

### 9.1 Simulación completa (lo que hace la rama)

```bash
source ~/staubli_ws/install/setup.bash

# Servidor Gazebo + puente de reloj + puente de cámara + 3 spawners, sin ventana
ros2 launch staubli_bringup launch_gazebo.launch.py robot_model:=tx2_60l gui:=false

# Lo anterior + ventana de Gazebo + GUI + servo del joystick
ros2 launch staubli_bringup launch_gazebo_gui.launch.py robot_model:=tx2_60l

# Solo la GUI y el servo (asumiendo Gazebo ya corriendo)
ros2 launch staubli_gui gui.launch.py

# Solo el servo
ros2 launch staubli_joystick_servo joystick_servo.launch.py
```

### 9.2 Trayectorias

```bash
# Solo calcular e imprimir el plan (no mueve nada): comportamiento por defecto
ros2 launch staubli_trajectory trajectory.launch.py

# Calcular Y enviar (requiere Gazebo + JTC activo)
ros2 launch staubli_trajectory trajectory.launch.py send:=true

# Con un YAML propio
ros2 launch staubli_trajectory trajectory.launch.py config:=/abs/path/mis_puntos.yaml
```

### 9.3 MoveIt (upstream)

```bash
ros2 launch staubli_bringup launch_demo.launch.py robot_model:=tx2_60l

# Con MoveIt Servo — ⚠ NO FUNCIONA en esta rama (§11.2)
ros2 launch staubli_bringup launch_moveit.launch.py robot_model:=tx2_60l launch_servo:=true
```

### 9.4 Robot real

```bash
# Desplegar la app VAL3 (una vez)
ros2 run staubli_robot_driver upload_val3_server.py

# Control real
ros2 launch staubli_bringup launch_robot_control.launch.py \
    robot_model:=tx2_60l robot_ip:=192.168.0.254 use_mock_hardware:=false

# Diagnóstico manual de la comunicación UDP (usa 8080/8081, no 11000/11001)
ros2 run staubli_robot_driver test_receive_robot_state

# Logs del robot
ros2 run staubli_robot_driver download_logs.py
ros2 run staubli_robot_driver show_logs.py

# Desinstalar la app
ros2 run staubli_robot_driver remove_val3_server.py
```

### 9.5 Solo ver el modelo

```bash
ros2 launch staubli_robot_description view_robot.launch.py
```

### 9.6 Docker (upstream)

```bash
cd .docker
# .env: STAUBLI_ROBOT_IP=192.168.0.254, STAUBLI_USE_MOCK_HARDWARE=true
docker compose up
# → ros2 launch staubli_bringup launch_demo.launch.py
```

---

## 10. Verificación y diagnóstico

### 10.1 Tras `launch_gazebo_gui.launch.py`

```bash
# ¿Los 3 controladores están donde deben?
ros2 control list_controllers
#   joint_state_broadcaster      active
#   joint_trajectory_controller  active
#   forward_velocity_controller  inactive      ◄── modo "trayectoria"

# ¿Llega el estado de las articulaciones?
ros2 topic echo /joint_states --once

# ¿Está el reloj bridged?
ros2 topic hz /clock

# ¿Llega la imagen de la cámara?
ros2 topic hz /camera/image_raw
#   esperado ≈ 30 Hz (update_rate del sensor)

# ¿Existe el árbol TF con la cámara?
ros2 run tf2_ros tf2_echo camera_link link_6
ros2 run tf2_ros tf2_echo tool0 camera_link

# ¿El controller_manager existe?
ros2 service list | grep controller_manager
```

### 10.2 Durante el control manual

```bash
# ¿La GUI publica Twist?
ros2 topic hz /cmd_vel

# ¿El servo responde con velocidades?
ros2 topic echo /forward_velocity_controller/commands

# ¿El FVC está activo?
ros2 control list_controllers | grep forward_velocity
```

### 10.3 Durante una trayectoria

```bash
ros2 topic echo /joint_states
#   seguir joint_1..joint_6 aproximándose al objetivo

ros2 action info /joint_trajectory_controller/follow_joint_trajectory
ros2 action list
```

### 10.4 En el robot real

```bash
# La app VAL3 está desplegada y escuchando?
ros2 run staubli_robot_driver test_receive_robot_state
#   ⚠ este binario usa 8080/8081; el Xacro configura 11000/11001

# Estado del CM y del hardware
ros2 control list_hardware_interfaces
ros2 control list_controllers
```

### 10.5 Probar los paquetes

```bash
cd ~/staubli_ws

# Driver C++ (7 gtests)
colcon test --packages-select staubli_robot_driver && colcon test-result --verbose

# Descripción y MoveIt
colcon test --packages-select staubli_robot_description staubli_moveit_config

# Python
pytest src/staubli_driver_ros2/staubli_gui/test
pytest src/staubli_driver_ros2/staubli_joystick_servo/test

# staubli_trajectory y staubli_bringup NO tienen pruebas
```

---

## 11. Modos de fallo conocidos

Estos fallos están **verificados en el código** de este commit. Ninguno ha sido
corregido.

| # | Modo de fallo | Dónde | Síntoma | Ámbito |
|---|---|---|---|---|
| 1 | **Distribución divergente** | `README.md:7`, `ci.yml:21`, `ci-format.yml:15`, `docker.yml:35` vs entorno local | La CI construye `jazzy`; el código está adaptado a `lyrical` | Riesgo alto: ningún cambio del driver C++ está validado por CI |
| 2 | **`MoveIt Servo` a topic inexistente** | `staubli_servo.yaml:70` | `servo_node` publica en `/forward_position_controller/commands`; no hay ningún `forward_position_controller` | `launch_servo:=true` no mueve el robot |
| 3 | **`payload_size` = `sequence_number`** | `val3/.../encodeHeader.pgx` (offset 6) | El host rechaza la trama al leer `payload_size`; conexión con el robot real rota | **Solo robot real**; la simulación no usa UDP |
| 4 | **Velocidad/par rechazados** | `staubli_hardware_interface.cpp:551-557` | `prepare_command_mode_switch` devuelve `ERROR` para `JOINT_VELOCITY`/`JOINT_TORQUE` | **El control manual FRAME no funciona en el robot real**, solo en Gazebo/mock |
| 5 | **Diagnósticos nunca leídos** | `robot_driver.cpp:182` | `diag_interface_ready = true` forzado; `read_diagnostic_data` sin llamar | La versión de firmware nunca se valida |
| 6 | **Puertos inconsistentes** | `communication/protocol.hpp:45,50` vs `r2c/staubli.r2c.xacro:50-53` | `test_receive_robot_state` escucha en 8080/8081 mientras el Xacro configura 11000/11001 | Diagnóstico manual |
| 7 | **TCP no implementado** | `src/communication/socket_factory.cpp:31`; `src/communication/tcp_socket.cpp:18` | `throw runtime_error("TCP protocol not implemented yet")`; include de un `.hpp` inexistente | Ninguno (nadie usa TCP) |
| 8 | **`move_group_name` incorrecto** | `staubli_servo.yaml:45` | `staubli_arm` vs nodo real `move_group` | Con `launch_servo:=true` |
| 9 | **`use_sim_time` ignorado** | `launch_robot_control.launch.py:144` | `{"use_sim_time": False}` hardcodeado | Solo en el camino upstream, no en Gazebo |
| 10 | **Enlace roto en README** | `README.md:52` | Apunta a `staubli_robot_driver/README.md` (no existe) | Menor |
| 11 | **`staubli_trajectory` sin tests** | No existe `test/` | La IK y el builder no tienen cobertura | Riesgo de regresión |
| 12 | **Parche de `gz_ros2_control` manual** | `patches/gz_ros2_control/` | Sin el parche, parar una interfaz borra todos los bits de control | Solo simulación |
| 13 | **Código muerto** | `tcp_socket.cpp`, `plotjuggler/*.xml`, lecturas de salidas comentadas | Confunde al lector | — |
| 14 | **Textos obsoletos** | `staubli_gui/package.xml:6-8` | Dice "Fase 1: solo presentación visual (sin conexión ROS)" | Menor |
| 15 | **Sin `ROS_GZ` deps declaradas** | `staubli_bringup/package.xml` | Usa `ros_gz_sim`, `ros_gz_bridge`, `gz_ros2_control` sin declararlos | `rosdep` no los instalará |
