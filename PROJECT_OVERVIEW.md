# PROJECT_OVERVIEW.md

Visión general del proyecto `staubli_driver_ros2`, basada en la lectura directa del
código del repositorio en la rama `feature/joystick-control`.

---

## 1. Metadatos del repositorio analizado

| Dato | Valor |
|---|---|
| Ruta del workspace | `/home/rocio/staubli_ws` |
| Ruta del repositorio | `/home/rocio/staubli_ws/src/staubli_driver_ros2` |
| Rama activa | `feature/joystick-control` |
| Commit activo | `4d195ec` — *Add camera laser marker visualization* |
| Otras ramas locales | `main`, `adaptacion-ubuntu-26-ros2-lyrical` |
| Licencia declarada | Apache License 2.0 (`staubli_*` originales), Apache-2.0 (paquetes nuevos) |
| Distribución ROS del README/CI | `jazzy` (Ubuntu 24.04 LTS) |
| Distribución ROS instalada localmente | `lyrical` (Ubuntu 26.04) — `ROS_DISTRO=lyrical` |
| Workspace ya construido | Sí: `~/staubli_ws/install` contiene los 8 paquetes de este repo |

### 1.1 Diferencia entre `main` y la rama analizada

La rama añade 18 commits sobre `main` (71 archivos, +7731 / −118 líneas). Los
commits, en orden cronológico, son:

```
4cac784 Adapt driver for ROS 2 Lyrical and Gazebo simulation
7c07db2 Add TX2-60 robot model and simulation support
b5603c9 Add inspection cell Gazebo world
b4ec161 Añade trayectoria de inspección y retorno a HOME
7a0b044 Añade GUI de control del robot
e54dd7a Conecta ENABLE DISABLE y HOME en la GUI
ee8271c Integra puntos P1 P2 P3 en la GUI
b1b91d2 Integra trayectoria y STOP en la GUI
64f7a2a Add user points and multi-point trajectories
7facb16 Add eye-in-hand camera to Gazebo
e35efbf Fix camera view refresh in GUI
e92efc0 feat: add joystick servo control and redesign GUI
74c745d checkpoint: manual frame control stable
d46eac3 checkpoint: JTC reactivation keeps real pose after manual control
6cd1471 feat: visual feedback of completed trajectory waypoints
d7c298f checkpoint: finalize GUI layout
4d195ec Add camera laser marker visualization
```

Es decir: la rama analizada **no es la versión upstream**. Sobre el driver C++
original añade tres paquetes Python nuevos (`staubli_gui`,
`staubli_joystick_servo`, `staubli_trajectory`), la simulación Gazebo, un mundo
de inspección, una cámara eye-in-hand y un parche a `gz_ros2_control`.

### 1.2 Fuera de alcance

En `/home/rocio/staubli_ws/src` existe un segundo repositorio,
`staubli_experimental`, con 7 paquetes (`staubli_experimental`,
`staubli_rx160_gazebo`, `staubli_tx2_60_support`, `staubli_tx2_90_support`,
`staubli_tx60_gazebo`, `staubli_tx60_support`, `staubli_tx90_gazebo`). **No forma
parte de este análisis**: no se ha leído su código ni se ha trazado ninguna
dependencia hacia o desde él, salvo la observación de que `install/` contiene
`staubli_rx160_gazebo` (procede de ese segundo repositorio, no de éste).

---

## 2. Propósito del proyecto

`staubli_driver_ros2` es un stack de ROS 2 que permite controlar robots Staubli a
través de `ros2_control`. El proyecto original de ICube Laboratory se centra en
el robot real con controlador CS9 mediante un protocolo UDP propio implementado
en parte en C++ (host) y en parte en VAL3 (robot).

La rama analizada amplía ese alcance con un **lote de inspección completo en
simulación**: Gazebo Sim, mundo con mesa y piezas, cámara eye-in-hand con
indicador láser, una GUI PySide6 y dos paquetes de lógica de trayectoria y
control manual cartesiano que no dependen de MoveIt.

### 2.1 Objetivos declarados en el código

- `README.md:11` — "ROS2 stack to control Staubli robots using `ros2_control`".
- `README.md:7` — desarrollador basado en `jazzy` / Ubuntu 24.04 LTS.
- `README.md:13` — controladores **CS9**; versiones antiguas no soportadas.
- `README.md:13` — "To date, only position control is supported".
- `README.md:15-19` — driver en desarrollo / prototipo, no industrial.
- `staubli_gui/package.xml:6-8` — "Fase 1: solo presentación visual (sin conexión ROS)".
  **Este texto está desactualizado**: la GUI actual sí se conecta a ROS 2
  (`gui.py` crea suscripciones, publicadores, clientes de servicio y de acción).
- `staubli_joystick_servo/package.xml:7-10` — control cartesiano por velocidad
  del TX2-60L sin MoveIt.
- `staubli_trajectory/package.xml:6-9` — trayectorias cartesianas configurables.

---

## 3. Estructura del repositorio

```
staubli_driver_ros2/
├── PROJECT_OVERVIEW.md          (este documento)
├── PROJECT_DEEP_DIVE.md         (detalle por paquete y archivo)
├── PROJECT_EXECUTION_FLOW.md    (flujos de ejecución paso a paso)
├── PROJECT_GLOSSARY.md          (glosario de términos)
├── README.md
├── CHANGELOG.md
├── LICENSE
├── .pre-commit-config.yaml
├── .clang-format.yaml
├── .github/
│   ├── dependabot.yml
│   └── workflows/{ci.yml, ci-format.yml, docker.yml, documentation.yml}
├── .docker/{Dockerfile, compose.yaml, .env, README.md}
├── patches/gz_ros2_control/fix_command_mode_switch.patch
├── logo-conectus-couleur.png
│
├── staubli_driver_ros2/          METAPAQUETE (ament_cmake)  + docs Sphinx/Doxygen
├── staubli_robot_description/    MODELO URDF/Xacro + ros2_control (ament_cmake)
├── staubli_robot_driver/         DRIVER C++ + aplicación VAL3 (ament_cmake, C++17)
├── staubli_bringup/              LAUNCH + controladores + mundo Gazebo (ament_cmake)
├── staubli_moveit_config/        CONFIGURACIÓN MoveIt (ament_cmake)
├── staubli_gui/                  GUI PySide6 (ament_python)
├── staubli_joystick_servo/       SERVO cartesiano por velocidad (ament_python)
└── staubli_trajectory/           TRAZECTORIAS + IK PyKDL (ament_python)
```

---

## 4. Los ocho paquetes

| Paquete | Tipo de build | Versión | Rol | Nº archivos |
|---|---|---|---|---|
| `staubli_driver_ros2` | `ament_cmake` (`NONE`) | 0.1.4 | Metapaquete y documentación | 3 + `sphinx/` + `doxygen/` |
| `staubli_robot_description` | `ament_cmake` | 0.1.4 | URDF/Xacro del robot, cámara y `ros2_control` | ~70 |
| `staubli_robot_driver` | `ament_cmake` / C++17 | 0.1.4 | Plugin de hardware por UDP + app VAL3 | ~90 |
| `staubli_bringup` | `ament_cmake` | 0.1.4 | Launch files, controladores, mundo Gazebo, RViz | 12 |
| `staubli_moveit_config` | `ament_cmake` | 0.1.4 | SRDF, cinemática, límites, MoveIt launch/RViz | 11 |
| `staubli_gui` | `ament_python` | 0.0.1 | Interfaz gráfica PySide6 de control | 11 |
| `staubli_joystick_servo` | `ament_python` | 0.0.1 | `/cmd_vel` → velocidades articulares (DLS) | 8 |
| `staubli_trajectory` | `ament_python` | 0.0.1 | Puntos YAML → IK → `FollowJointTrajectory` | 10 |

### 4.1 Dependencias entre paquetes (declaradas en `package.xml`)

```
staubli_driver_ros2 (metapaquete)
  ├── staubli_bringup
  ├── staubli_moveit_config
  ├── staubli_robot_description
  └── staubli_robot_driver

staubli_bringup
  ├── staubli_robot_description
  ├── staubli_gui            (solo exec_depend; usado por launch_gazebo_gui)
  └── [launch] staubli_moveit_config, staubli_joystick_servo, staubli_gui

staubli_moveit_config
  └── staubli_robot_description

staubli_gui
  ├── staubli_trajectory
  └── staubli_joystick_servo

staubli_joystick_servo
  └── staubli_trajectory     (importa tx2_60l_ik)

staubli_trajectory
  └── (ninguna interna)

staubli_robot_description
  └── (ninguna interna)

staubli_robot_driver
  └── (ninguna interna)
```

**Observación**: el metapaquete `staubli_driver_ros2` declara `exec_depend` de
cuatro paquetes, pero **no** de `staubli_gui`, `staubli_joystick_servo` ni
`staubli_trajectory`, aunque los tres son parte del producto en esta rama. Se
resuelven de forma transitiva (`staubli_bringup` → `staubli_gui` → los otros
dos), por lo que `rosdep` no fallará, pero la lista del metapaquete ya no
refleja el contenido real del repositorio.

---

## 5. Arquitectura por capas

```
┌──────────────────────────────────────────────────────────────────────┐
│  CAPA 5 — INTERFACES DE USUARIO                                     │
│  RViz (rviz2, moveit.rviz, default.rviz)                             │
│  Gazebo GUI (gz sim -g)                                             │
│  staubli_gui (PySide6): cámara, FRAME, trayectorias, ENABLE/HOME    │
└──────────────────────────────────────────────────────────────────────┘
┌──────────────────────────────────────────────────────────────────────┐
│  CAPA 4 — PLANIFICACIÓN Y CINEMÁTICA                                 │
│  staubli_moveit_config: move_group, Pilz, servo_node, KDL IK        │
│  staubli_trajectory: PointsConfig → TrajectoryBuilder → ActionClient │
│  staubli_joystick_servo: DLS (PyKDL) /cmd_vel → q̇                   │
└──────────────────────────────────────────────────────────────────────┘
┌──────────────────────────────────────────────────────────────────────┐
│  CAPA 3 — ros2_control                                               │
│  controller_manager (nodo)                                          │
│    ├── joint_state_broadcaster  → /joint_states                     │
│    ├── joint_trajectory_controller → posición (FollowJointTrajectory)│
│    └── forward_velocity_controller → velocidad (Float64MultiArray)   │
└──────────────────────────────────────────────────────────────────────┘
┌──────────────────────────────────────────────────────────────────────┐
│  CAPA 2 — DESCRIPCIÓN DEL ROBOT                                     │
│  staubli_robot_description                                          │
│    urdf/staubli.urdf.xacro → /robot_description                      │
│    r2c/staubli.r2c.xacro → <ros2_control> + selección de plugin      │
│    urdf/include/staubli_camera.xacro → cámara + indicador láser      │
└──────────────────────────────────────────────────────────────────────┘
┌──────────────────────────────────────────────────────────────────────┐
│  CAPA 1 — BACKENDS DE HARDWARE (mutuamente excluyentes)             │
│  (A) staubli_robot_driver/StaubliHardwareInterface  → UDP → CS9 real │
│  (B) mock_components/GenericSystem                   → sin hardware  │
│  (C) gz_ros2_control/GazeboSimSystem                 → Gazebo Sim     │
└──────────────────────────────────────────────────────────────────────┘
```

### 5.1 Selección de backend de hardware

La selección ocurre **dentro del Xacro**, no en el launch. La macro
`staubli_ros2_control` de `staubli_robot_description/r2c/staubli.r2c.xacro:8-120`
elige el plugin en este orden (`r2c/staubli.r2c.xacro:36-59`):

| `use_gazebo_hardware` | `use_mock_hardware` | Plugin seleccionado |
|---|---|---|
| `true` | (ignorado) | `gz_ros2_control/GazeboSimSystem` |
| `false` | `true` | `mock_components/GenericSystem` |
| `false` | `false` | `staubli_robot_driver/StaubliHardwareInterface` |

Además, la misma macro decide dinámicamente:

- **Interfaz de comando `acceleration`**: solo se exporta con hardware real
  (`r2c/staubli.r2c.xacro:64-79`). En Gazebo y mock se omite porque
  `GazeboSimSystem` no expone esa interfaz; en hardware real sirve únicamente
  para que el limitador de `ros2_control` pueda aplicar límites de aceleración.
- **Bloques `<gpio>`**: se excluyen cuando `use_gazebo_hardware=true`
  (`r2c/staubli.r2c.xacro:90-118`), porque `GazeboSimSystem` no soporta GPIO
  virtuales de supervisión ni E/S.

---

## 6. Inventario de interfaces ROS 2

### 6.1 Topics

| Topic | Tipo | Publicador | Suscriptor | Notas |
|---|---|---|---|---|
| `/robot_description` | `std_msgs/String` | `robot_state_publisher` | `ros_gz_sim create`, MoveIt, `wait_for_robot_ready` | QoS `TRANSIENT_LOCAL` (latched) |
| `/joint_states` | `sensor_msgs/JointState` | `joint_state_broadcaster` | `staubli_gui`, `staubli_joystick_servo`, MoveIt, `wait_for_robot_ready` | QoS `VOLATILE` |
| `/camera/image_raw` (ROS) | `sensor_msgs/Image` | `ros_gz_bridge parameter_bridge` | `staubli_gui` | `rgb8`, QoS `sensor_data` |
| `/camera/image_raw` (gz) | `gz.msgs.Image` | sensor Gazebo en `camera_link` | bridge | 640×480, 30 Hz, `R8G8B8` |
| `/clock` | `rosgraph_msgs/Clock` | Gazebo (`SceneBroadcaster`) | `ros_gz_bridge` | necesario para `use_sim_time:=true` |
| `/cmd_vel` | `geometry_msgs/Twist` | `staubli_gui` (solo con control manual activo) | `staubli_joystick_servo` | frame `base_link` |
| `/forward_velocity_controller/commands` | `std_msgs/Float64MultiArray` | `staubli_joystick_servo` | `forward_velocity_controller` | 6 valores, `joint_1..6` en rad/s |
| `/controller_manager/list_controllers` | topic interno del CM | `controller_manager` | — | La GUI lo usa como **servicio** |

### 6.2 Servicios

| Servicio | Tipo | Cliente | Productor |
|---|---|---|---|
| `/controller_manager/list_controllers` | `controller_manager_msgs/srv/ListControllers` | `staubli_gui` (async, ~1 Hz) | `controller_manager` |
| `/controller_manager/switch_controller` | `controller_manager_msgs/srv/SwitchController` | `staubli_gui` (`STRICT`, `activate_asap=False`, timeout 5 s) | `controller_manager` |
| `/world/inspection_cell/create` | `gz.msgs.EntityFactory` | `ros_gz_sim create` | `gz-sim` |

### 6.3 Acciones

| Acción | Tipo | Cliente | Servidor |
|---|---|---|---|
| `/joint_trajectory_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | `staubli_gui`, `staubli_trajectory` | `joint_trajectory_controller` |

### 6.4 Parámetros ROS 2 utilizados

| Nodo / componente | Parámetro | Valor por defecto |
|---|---|---|
| `controller_manager` | `update_rate` | `250` Hz |
| `joint_trajectory_controller` | `action_monitor_rate` | `20.0` |
| `joint_trajectory_controller` | `interpolate_from_desired_state` | `true` |
| `joint_trajectory_controller` | `set_last_command_interface_value_as_state_on_activation` | `false` |
| `joint_trajectory_controller` | `allow_nonzero_velocity_at_trajectory_end` | `true` |
| `joint_trajectory_controller` | `constraints.goal_time` | `0.0` |
| `staubli_joystick_servo` | `control_rate` | `250.0` |
| `staubli_joystick_servo` | `command_timeout` / `joint_state_timeout` | `0.2` s |
| `staubli_joystick_servo` | `damping_lambda` | `0.05` |
| `staubli_joystick_servo` | `singularity_threshold` | `0.02` |
| `staubli_joystick_servo` | `max_linear_speed` | `0.15` m/s |
| `staubli_joystick_servo` | `max_angular_speed` | `0.5` rad/s |
| `staubli_joystick_servo` | `acceleration_limit` | `1.5` rad/s² |
| `staubli_joystick_servo` | `deadzone_linear` / `deadzone_angular` | `0.01` |
| `StaubliHardwareInterface` | `robot_ip` | `""` (obligatorio) |
| `StaubliHardwareInterface` | `control_port` / `diagnostic_port` | `11000` / `11001` (vía Xacro) |
| `StaubliHardwareInterface` | `local_control_port` / `local_diagnostic_port` | `11000` / `11001` (vía Xacro) |
| MoveIt Servo | `command_out_topic` | `/forward_position_controller/commands` (ver §9.2) |
| MoveIt | `default_velocity_scaling_factor` | `0.1` |
| MoveIt | `default_acceleration_scaling_factor` | `0.1` |

---

## 7. Puntos de entrada (launch files)

| Comando | Descripción | Include |
|---|---|---|
| `ros2 launch staubli_bringup launch_gazebo.launch.py robot_model:=tx2_60l` | Simulación Gazebo + puente de reloj + puente de cámara + spawn + 3 spawners | — |
| `ros2 launch staubli_bringup launch_gazebo_gui.launch.py` | Lo anterior + ventana Gazebo + GUI + servo del joystick | `launch_gazebo` + `staubli_gui/gui.launch.py` |
| `ros2 launch staubli_gui gui.launch.py` | GUI + `joystick_servo` con `use_sim_time:=true` | `joystick_servo.launch.py` |
| `ros2 launch staubli_joystick_servo joystick_servo.launch.py` | Solo el nodo de servo | — |
| `ros2 launch staubli_trajectory trajectory.launch.py [send:=true]` | Nodo de trayectoria (solo cálculo por defecto) | — |
| `ros2 launch staubli_bringup launch_robot_control.launch.py` | Stack clásico mock/real + RViz (sin Gazebo) | — |
| `ros2 launch staubli_bringup launch_moveit.launch.py` | `wait_for_robot_ready` + MoveIt | `staubli_moveit_config/staubli_moveit.launch.py` |
| `ros2 launch staubli_bringup launch_demo.launch.py` | `launch_robot_control` + `launch_moveit` (upstream) | los dos anteriores |
| `ros2 launch staubli_robot_description view_robot.launch.py` | Solo visualisation del URDF | — |

Argumentos comunes: `robot_model` (`tx2_60l` por defecto en Gazebo; **sin valor
por defecto** en `launch_robot_control`, `launch_moveit`, `staubli_moveit`),
`gui`, `use_sim_time`, `use_mock_hardware`, `robot_ip`.

Detalle completo de cada launch en `PROJECT_EXECUTION_FLOW.md`.

---

## 8. Construcción y ejecución rápida

### 8.1 Entorno local detectado

```bash
$ echo $ROS_DISTRO
lyrical
$ ls /opt/ros/
lyrical
```

El workspace ya contiene `build/`, `install/` y `log/`, además de artefactos de
una compilación local de `gz_ros2_control` 3.0.1/3.0.8 (`.deb`, `.dsc`,
`.tar.xz` y un checkout `ros-lyrical-gz-ros2-control-3.0.8`) en
`~/staubli_ws/`. Esto indica que el usuario **parcheó localmente**
`gz_ros2_control` (ver `patches/gz_ros2_control/fix_command_mode_switch.patch`),
pero el parche **no está aplicado automáticamente** por ningún CMakeLists ni
script del repositorio: hay que aplicarlo a mano sobre el código fuente de
`gz_ros2_control`.

### 8.2 Compilar

```bash
source /opt/ros/lyrical/setup.bash        # NO jazzy: ver §9.1
cd ~/staubli_ws
colcon build --symlink-install
source install/setup.bash
```

### 8.3 Lanzar la celda de inspección completa

```bash
source ~/staubli_ws/install/setup.bash
ros2 launch staubli_bringup launch_gazebo_gui.launch.py robot_model:=tx2_60l
```

### 8.4 Ejecutar las pruebas

| Paquete | Comando |
|---|---|
| `staubli_robot_driver` | `colcon test --packages-select staubli_robot_driver` (7 gtests) |
| `staubli_robot_description` | `colcon test --packages-select staubli_robot_description` (pytest) |
| `staubli_moveit_config` | `colcon test --packages-select staubli_moveit_config` (pytest) |
| `staubli_gui` | `pytest staubli_gui/test` (5 archivos, ~1064 líneas) |
| `staubli_joystick_servo` | `pytest staubli_joystick_servo/test` (1 archivo, 248 líneas) |
| `staubli_trajectory` | **No existe directorio `test/`**: sin pruebas |

---

## 9. Situación de versiones, coherencia y riesgos

Esta sección resume los hallazgos; el detalle con evidencia `archivo:línea` está
en `PROJECT_DEEP_DIVE.md` §11.

### 9.1 Distribuciones ROS divergentes (riesgo alto)

- `README.md:7`, `.github/workflows/ci.yml:21`, `ci-format.yml:15,24` y
  `docker.yml:35` declaran
  `jazzy` / Ubuntu 24.04.
- El entorno instalado es `lyrical` / Ubuntu 26.04, y el commit `4cac784` se
  titula "Adapt driver for ROS 2 **Lyrical**".
- Consecuencia: **la CI no valida el código que se está ejecutando**. La rama
  contiene adaptations a Lyrical (`real_time_socket_interface.hpp`,
  `hardware_interface::HardwareComponentInterfaceParams`, `std::copy`/`std::fill`
  en lugar de `resize`, `socket_factory.cpp`, `udp_socket_impl.hpp`) que no
  existen en el estado que construye la CI de `jazzy`.

### 9.2 `MoveIt Servo` apunta a un controlador inexistente

`staubli_moveit_config/config/staubli_servo.yaml:47` fija
`command_out_topic: /forward_position_controller/commands`, pero
`staubli_bringup/config/controllers.yaml` solo define
`joint_state_broadcaster`, `joint_trajectory_controller` y
`forward_velocity_controller`. No hay ningún `forward_position_controller` en
todo el repositorio. Además `command_out_type: std_msgs/Float64MultiArray` con
`publish_joint_positions: true` es exactamente la firma de un
`forward_position_controller`. Conclusión: **`launch_servo:=true` no puede
funcionar en esta rama**. El paquete `staubli_joystick_servo` cubre el control
cartesiano por velocidad, que es un camino distinto y sí coherente.

### 9.3 Bug confirmado en la serialización VAL3

`val3/userapp/ros2_server/Serialization/encodeHeader.pgx:56-58` escribe el
número de secuencia **dos veces**:

```
toBinary(x_header.nMsgSequenceNumber,2,"2l",x_nData[l_nIndexOffset])  # offset 4
l_nIndexOffset = l_nIndexOffset + 2
toBinary(x_header.nMsgSequenceNumber,2,"2l",x_nData[l_nIndexOffset])  # offset 6  <-- debería ser nPayloadSize
```

El campo del offset 6 es el tamaño del payload según
`include/staubli_robot_driver/communication/protocol.hpp:70`
(`FrameHeader::payload_size`). `decodeHeader.pgx:50-51` sí decodifica
`nPayloadSize` correctamente ("We don't really need this one, but let's decode it
anyway"). El C++ **sí** valida `payload_size`, por lo que este bug rompe la
comunicación real con el robot en cuanto el número de secuencia deje de coincidir
con el tamaño del mensaje. **No afecta a la simulación** (que no usa UDP).

### 9.4 Discrepancia de puertos UDP

- `staubli_robot_driver/include/staubli_robot_driver/communication/protocol.hpp:45,50` → `DEFAULT_CONTROL_PORT = 8080`,
  `DEFAULT_DIAGNOSTICS_PORT = 8081`.
- `r2c/staubli.r2c.xacro:50-53` → `control_port`, `diagnostic_port`,
  `local_control_port`, `local_diagnostic_port` = `11000` / `11001`.

Vía Xacro (que es el camino real de producción) mandan los valores del Xacro, así
que los defaults C++ son código muerto en ese flujo; solo afectan a las
sobrecargas `RobotDriver::connect(robot_ip, timeout_ms)` y a
`test_receive_robot_state.cpp`. Documentado como inconsistencia, no como bug
confirmado en producción.

### 9.5 `tcp_socket.cpp` no compila y el sistema TCP está inacabado

- `src/communication/tcp_socket.cpp:18` incluye `"tcp_socket_impl.hpp"`, archivo
  **que no existe** en el repositorio.
- `staubli_robot_driver/CMakeLists.txt:64-77` (`LIBRARY_SOURCES`) **no incluye** `tcp_socket.cpp`, así
  que el error no se manifiesta en el build.
- `SocketFactory` (`src/communication/socket_factory.cpp`) puede crear sockets
  TCP, pero no hay implementación funcional: el driver real usa UDP.

### 9.6 Diagnósticos implementados a medias

- `robot_driver.cpp:182` — `diag_interface_ready = true;` con el
  `read_diagnostic_data` comentado y un `TODO(tpoignonec)`.
- `robot_driver.cpp:220-232` — la comprobación de tipo de controlador CS9 está
  comentada.
- `robot_driver.cpp:240` — `TODO`: validar versiones de VAL3 soportadas.
- `staubli_hardware_interface.cpp:219-224` — `on_shutdown` envía STOP
  automáticamente, pero `send_stop_all_command()` no consulta
  `is_connected()` internamente (lo hace el llamador, OK) y `on_error` sí.
- El control por **velocidad y par** está explícitamente rechazado
  (`staubli_hardware_interface.cpp:551-557`): `prepare_command_mode_switch`
  devuelve `ERROR` para `JOINT_VELOCITY` y `JOINT_TORQUE`. Esto significa que
  **`forward_velocity_controller` no puede activarse contra el robot real**; solo
  funciona en Gazebo (vía `GazeboSimSystem`) o mock. El branch
  `feature/joystick-control` depende de ese controlador, luego el control manual
  FRAME es **solo simulación**.

### 9.7 Enlace roto en el README

`README.md:52` enlaza a `staubli_robot_driver/README.md`, que **no existe**. El
README real de VAL3 está en `staubli_robot_driver/val3/README.md`.

### 9.8 Textos desactualizados

- `staubli_gui/package.xml:6-8` — "Fase 1: solo presentación visual (sin conexión ROS)".
- `README.md:13` — "only position control is supported" (correcto para el robot
  real, pero la rama añade control por velocidad **solo en simulación**).
- `staubli_moveit_config/config/joint_limits.yaml` — los límites están en rad/s
  y rad/s², pero `staubli_robot_description/config/tx2_60l/joint_limits.yaml`
  declara los límites del URDF en **grados** (`!degrees 300.0` para
  `max_velocity`). Las unidades son coherentes por separado, pero el valor
  `max_velocity: 1.5 rad/s` de MoveIt es mucho más restrictivo que
  `300 deg/s ≈ 5.24 rad/s` del URDF.

### 9.9 Detalles menores

- `staubli_robot_driver/val3/COLCON_IGNORE` — correctamente excluye el árbol
  VAL3 de `colcon`.
- `.pre-commit-config.yaml` excluye `staubli_robot_driver/val3/` de todos los
  hooks.
- Hay artefactos `__pycache__/` y `.pytest_cache/` versionados o presentes en el
  árbol de trabajo (`.pyc` de Python 3.14).
- `staubli_robot_description/config/` incluye tres modelos: `tx2_60`,
  `tx2_60l`, `tx2_60l_med`. Solo `tx2_60l` tiene `dynamics.yaml` +
  `robot_model.yaml` completos en la rama analizada; `tx2_60` y `tx2_60l_med`
  también los tienen, pero `tx2_60` **no** tiene los meshes STL de colisión
  binarios de `link_2`/`link_4` genéricos (solo `_L`).

---

## 10. Conclusión

`feature/joystick-control` es un **prototipo de celda de inspección**, no un
driver de producción. La parte de simulación (Gazebo + GUI + servo +
trayectorias) está cohesionada y bien documentada en comentarios; la parte del
robot real hereda el estado de prototipo del upstream (solo control de posición,
diagnósticos sin implementar, un bug de serialización en VAL3 y soporte de
velocidad/par explícitamente rechazado).

Los tres documentos siguientes profundizan:

- `PROJECT_DEEP_DIVE.md` — detalle por paquete y por archivo, con evidencia.
- `PROJECT_EXECUTION_FLOW.md` — diagramas ASCII y pasos de ejecución.
- `PROJECT_GLOSSARY.md` — definiciones de todos los términos técnicos usados.
