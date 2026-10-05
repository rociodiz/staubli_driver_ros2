# PROJECT_GLOSSARY.md

Glosario de términos del repositorio
`~/staubli_ws/src/staubli_driver_ros2`, rama `feature/joystick-control`
(commit `4d195ec`).

Criterios:

- Cada entrada indica **dónde** aparece el término (`archivo:línea`) o se marca
  como `DESCONOCIDO` si el repositorio no permite determinarlo.
- Los valores numéricos proceden de enums, constantes o literales reales; no hay
  valores inventados.
- Las entradas marcadas **⚠** son inconsistencias o defectos documentados en
  `PROJECT_DEEP_DIVE.md` §11.
- Solo se incluyen términos que aparecen realmente en el repositorio. Los
  términos de ROS 2 o cinemática que se usan sin aparecer literalmente se
  indican como "concepto usado en…" para no fingir una cita inexistente.

---

## Índice

[A](#a) · [B](#b) · [C](#c) · [D](#d) · [E](#e) · [F](#f) · [G](#g) ·
[H](#h) · [I](#i) · [J](#j) · [K](#k) · [L](#l) · [M](#m) · [N](#n) ·
[O](#o) · [P](#p) · [Q](#q) · [R](#r) · [S](#s) · [T](#t) · [U](#u) ·
[V](#v) · [W](#w) · [X](#x) · [Z](#z)
· [Apéndice 1: rutas y ficheros clave](#apéndice-1-rutas-y-ficheros-clave)
· [Apéndice 2: códigos del protocolo](#apéndice-2-códigos-del-protocolo)

---

## A

| Término | Definición |
|---|---|
| **action** (acción ROS 2) | Interfaz de petición con streaming de feedback y cancelación. El repo usa una: `/joint_trajectory_controller/follow_joint_trajectory` (`control_msgs/action/FollowJointTrajectory`). |
| **actuador** | Cada uno de los 6 motores del TX2-60L. En el URDF son las articulaciones `joint_1`…`joint_6`; los comandos viajan como `command_reference[6]` dentro del `RobotCommand`. |
| **`ament_cmake`** | Sistema de build de ROS 2 para paquetes C++. `staubli_robot_driver/CMakeLists.txt`. `ament_target_dependencies()` fue eliminado en versiones modernas, por eso el paquete usa targets directos (`staubli_robot_driver/CMakeLists.txt:90-92`). |
| **`ament_lint_auto`** | Linter de ROS 2. En `staubli_robot_driver` se desactivan `ament_copyright` y `ament_cpplint` porque los ficheros ya llevan cabecera propia (`staubli_robot_driver/CMakeLists.txt:177-184`). |
| **autoConnectMove(true)** | Instrucción VAL3: encadena tareas automáticamente sin `wait` explícito. `val3/userapp/ros2_server/start.pgx:41`. |

## B

| Término | Definición |
|---|---|
| **base_link** | Marco de la base del robot. Es el `planning_frame` de `staubli_trajectory/config/points.yaml` y el marco en el que la GUI publica el `Twist` del control manual. |
| **`bInError`** | Flag de error del `RobotCommand` de VAL3. Si se activa, el host envía `STOP` y devuelve `ERROR` desde `perform_command_mode_switch()`. `staubli_robot_driver/src/staubli_hardware_interface.cpp`. |
| **`bIsOverrun`** | Flag de overrun de la tarea periódica de control. `start.pgx:44-45`: si `controlTask` no termina en `l_nCtrlPeriod`, el robot lo detecta. |
| **`bIsValid`** | Flag de validez del comando en VAL3 (`RobotCommand.bIsValid`). Si es falso, `updateJointPositionControl.pgx:49-55` aborta y llama a `stopJointPositionControl`. |
| **`bMotionPossible`** | Flag "movimiento posible" del `RobotState` en VAL3. Si es falso, el control aborta: `updateJointPositionControl.pgx:38`, `updateJointVelocityControl.pgx:22`. |
| **BOM (boxcar filter)** | Filtro de media móvil de CS9. `start.pgx:19-21` lee `$getBoxcarDelay()`, registra el valor `"0.1"` y avisa si `getMonitorSpeed() < 100`. La llamada `//$setBoxcarFreq(60.0)` está comentada. |
| **Boost** | Librería C++ del socket de tiempo real. En Ubuntu 26.04 / Boost 1.90 `Boost.System` es header-only, así que se usa el target `Boost::headers` (`staubli_robot_driver/CMakeLists.txt:53-61`). |
| **boxcar delay** | Periodo del filtro BOM en segundos. `l_nBoxcarDelay` en `start.pgx:20`. |

## C

| Término | Definición |
|---|---|
| **camera_link** | Link del URDF que porta el sensor de cámara y los visuales del indicador láser. Se define en `staubli_robot_description/urdf/include/staubli_camera.xacro` como hijo de `tool0`, por eso el triángulo láser se mueve con el robot. |
| **ciclo de control** | Periodo unificado del bucle de control: **4 ms / 250 Hz**. Aparece como `update_rate: 250` (`staubli_bringup/config/controllers.yaml`), `l_nCtrlPeriod = 0.004` (`start.pgx:38`) y `controller_period = 4` ms (C++, `robot_driver_helpers.hpp`). |
| **cinemática** | `Tx260lKinematics` en `staubli_trajectory/staubli_trajectory/tx2_60l_ik.py`: FK, IK y Jacobiano con las constantes del TX2-60L. |
| **`clearBuffer(siSocketControl)`** | Instrucción VAL3 que vacía el búfer del socket de control al arrancar. `start.pgx:34`. |
| **comando `STOP`** | `CommandType::STOP = 0x01` (`types.hpp:37`). Provoca `send_stop_all()`: detiene el movimiento y pone a cero las salidas. |
| **`controller_manager`** | Nodo de `ros2_control` que arbitra los controladores a 250 Hz. En Gazebo lo crea el plugin `GazeboSimROS2ControlPlugin`; en mock/real lo crea `ros2_control_node` explícitamente. |
| **controlador de trayectoria** | `joint_trajectory_controller` (JTC): controla **posición** articular. Leaf `/joint_trajectory_controller`. |
| **controlador de velocidad** | `forward_velocity_controller` (FVC): controla **velocidad** articular. Leaf `/forward_velocity_controller`. Solo funcional en simulación/mock. |
| **`ControllersManager` (GUI)** | Clase propia de `staubli_gui/staubli_gui/controller_manager.py` que encapsula `list_controllers` + `switch_controller` para la máquina de estados de la GUI. No es el nodo `controller_manager`. |
| **CS9** | Controlador de robot Staubli. Ejecuta VAL3 (`.pgx`) internamente y habla UDP hacia el host. |
| **CSP / modo de motor** | El driver solo manipula un `Mode` local (`NONE` / `POSITION` / `ACCELERATION`) dentro de `StaubliHardwareInterface`. El modo real del CS9 no se consulta. **DESCONOCIDO**: la correspondencia entre ambos. |

## D

| Término | Definición |
|---|---|
| **deadzone** (zona muerta) | Umbral por eje del `Twist` por debajo del cual el eje se considera cero. Evita que el ruido del joystick produzca movimiento. `staubli_joystick_servo/staubli_joystick_servo/servo_node.py:154-160`. |
| **DECELERATION** | `DECELERATION_JOINT_POSITION = 0x0A` y `DECELERATION_JOINT_VELOCITY = 0x0B` (`types.hpp:42-43`). Declarados; el host nunca los genera. |
| **`deg2rad` / `rad2deg`** | Conversión de unidades. El host almacena en radianes (ROS) y envía grados (CS9); e inversamente al leer. `staubli_hardware_interface.cpp`, `robot_driver_helpers.hpp`. |
| **`digital_outputs[]`** | Array de 16 salidas binarias del `RobotCommand`. El host las escribe con el criterio `cmd > 0.5`. |
| **DLS (Damped Least Squares)** | Solución de mínimos cuadrados amortiguada: `q̇ = Jᵀ(JJᵀ + λ²I)⁻¹ v`, con `λ = 0.05` y umbral de singularidad `σ_thr = 0.02`. `servo_node.py:170-176`. |
| **`dq` / `q̇`** | Vector de 6 velocidades articulares en rad/s que el servo publica. |

## E

| Término | Definición |
|---|---|
| **estado de supervisión** | `SupervisionState` con 10 elementos en el driver (estado, velocidad, etc.). Su semántica exacta por índice está marcada **DESCONOCIDO** en el código. `staubli_robot_driver/include/staubli_robot_driver/hardware_interface.hpp`. |
| **E/S digital y analógica** | 16 entradas, 16 salidas, 2 entradas y 2 salidas analógicas, declaradas como interfaces `gpio` de `ros2_control` en `r2c/staubli.r2c.xacro:54-57`. La lectura del lado C++ está parcialmente comentada. |
| **`ee_pos` / `ee_rot`** | Pose del frame de herramienta `tool0` respecto a `base_link`. Salida de la cinemática directa. **DESCONOCIDO**: estos identificadores literales no aparecen en el código; es notación usada en los comentarios de IK. |

## F

| Término | Definición |
|---|---|
| **FK (Forward Kinematics)** | Cinemática directa, de `q` a pose de `tool0`. `Tx260lKinematics.forward_kinematics()`. |
| **`Float64MultiArray`** | Tipo de `std_msgs` del topic de comandos de velocidad: 6 valores en rad/s. |
| **freshness / watchdog** | Vigencia de un mensaje. El servo frena si `/joint_states` o `/cmd_vel` llevan más de 0.2 s sin actualizarse. `servo_node.py:139-150`. |
| **FVC** | Abreviatura de `forward_velocity_controller`. |

## G

| Término | Definición |
|---|---|
| **generador de trayectorias** | `TrajectoryBuilder` (`staubli_trajectory/staubli_trajectory/trajectory_builder.py:58`): convierte los puntos de `points.yaml` en un `trajectory_msgs/JointTrajectory` con timestamps. |
| **`getMonitorSpeed()`** | VAL3: porcentaje de velocidad del monitor. `start.pgx:26-28` avisa si es < 100 % porque los algoritmos de control pueden fallar. |
| **`getVersion("System")`** | VAL3: versión del CS9. `start.pgx:13`; `l_sMajor = mid(l_sVal3,1,1)` extrae el número mayor. |
| **GZ (Gazebo)** | Simulador. `gz_ros2_control` conecta el URDF con `ros2_control`; `ros_gz_bridge` conecta topics entre ROS y Gazebo. |
| **GUI** | Interfaz PySide6 del paquete `staubli_gui`. Módulos: `gui.py`, `controller_manager.py`, `frame_control.py`, `user_points.py`, `points_table.py`, `theme.py`. |

## H

| Término | Definición |
|---|---|
| **hardware interface** | Implementación de `hardware_interface::SystemInterface` que `ros2_control` carga como plugin. Hay tres en el repo: `GazeboSimSystem` (simulación), `GenericSystem` (mock) y `StaubliHardwareInterface` (robot real). |
| **`hardware_interface::SystemInterface`** | Interfaz base de `ros2_control`: un `read()` y un `write()` por ciclo, a diferencia del diseño de `realtime_tools` de dos hilos. |
| **`HardwareComponent`** | Clase de `gz_ros2_control` que envuelve la `HardwareInterface` y la expone en el mundo de Gazebo. |
| **hook de `hardware_interface`** | **DESCONOCIDO**: la cadena concreta de `create_component` / `HardwareComponent` dentro de `gz_ros2_control` 3.0.8 no está en este repositorio. |

## I

| Término | Definición |
|---|---|
| **IK (Inverse Kinematics)** | Cinemática inversa, de pose de `tool0` a `q`. `Tx260lKinematics.inverse_kinematics(frame)` con multi-semilla (`tx2_60l_ik.py:224`): `random_seed=42`, `n_random_seeds=30`. |
| **tolerancia de IK** | `IK_POS_EPS = 1e-4` m y `IK_ROT_EPS = 1e-3` rad. Criterio de aceptación de una solución de IK en `tx2_60l_ik.py`. |
| **`is_ready()`** | `robot_driver` del host: comprueba que el socket está conectado y operativo antes de leer o escribir. |

## J

| Término | Definición |
|---|---|
| **Jacobiano** | Matriz 6×6 `∂v_tool/∂q̇`. `Tx260lKinematics.jacobian(q)`, recalculado en cada ciclo de 250 Hz del servo. |
| **`joint_1`…`joint_6`** | Las 6 articulaciones del TX2-60L. Nombres canónicos en todo el repo: URDF, `points.yaml`, `controllers.yaml`, SRDF. |
| **`joint_state_broadcaster` (JSB)** | Controlador que publica `/joint_states`. Leaf `/joint_state_broadcaster`. |
| **`/joint_states`** | Topic `sensor_msgs/JointState` con posiciones, velocidades y efforts. QoS depth 1, VOLATILE. |
| **`JointTolerance`** | Tolerancias por articulación en un goal de `FollowJointTrajectory` (posición, velocidad, aceleración). `_build_goal()` las toma de `PointsConfig` (`joint_trajectory_client.py:117`). |
| **`joint_trajectory_controller` (JTC)** | Controlador de trayectoria. Leaf `/joint_trajectory_controller`. Activo en el flujo Gazebo; se activa y desactiva en la máquina de estados de la GUI. |
| **jog / control manual** | Movimiento manual continuo. En la GUI se llama "control manual" o "carril FRAME". |

## K

| Término | Definición |
|---|---|
| **KDL** | Kinematics and Dynamics Library. Resolvedor de IK y Jacobiano. En `staubli_trajectory` se usa directamente vía PyKDL; en MoveIt como `KDLKinematicsPlugin`. |
| **`kdl_parser`** | Convierte el URDF en el `Tree` de KDL. |
| **`kinematics.yaml`** | Configuración de `moveit_kinematics`: `KDLKinematicsPlugin`, `kinematics_solver_search_resolution: 0.005`, `kinematics_solver_timeout: 0.5`. |
| **`Kinematics.yaml` (paquete MoveIt)** | `staubli_moveit_config/config/kinematics.yaml`. |
| **kApp** | La aplicación VAL3 desplegada en el CS9 con `upload_val3_server.py`; el nombre del paquete de destino es `ros2_server`. |

## L

| Término | Definición |
|---|---|
| **leaf** | Sufijo del topic de un controlador, p. ej. `/forward_velocity_controller/commands`. |
| **latched / TRANSIENT_LOCAL** | QoS "durable": el topic retiene el último mensaje para suscriptores tardíos. `/robot_description` lo necesita porque `ros_gz_sim/create` arranca después del publicador. |
| **`l_nBoxcarDelay`** | Retardo del filtro BOM leído con `$getBoxcarDelay()`. `start.pgx:19`. |
| **`l_nCtrlPeriod`** | Periodo de la tarea de control en VAL3: `0.004` s = 4 ms. `start.pgx:38`. |
| **lifecycle** | Máquina de estados de `rclcpp_lifecycle`. `staubli_robot_driver` la usa en su nodo de diagnóstico (`DiagnosticDataLcmHandlerNode`). |
| **LSP** | Abreviatura de "lifecycle" usada en los comentarios del driver. |

## M

| Término | Definición |
|---|---|
| **Magic number** | `0xABCD`: los 2 primeros bytes de cada trama UDP. `communication/protocol.hpp`. |
| **manual (modo)** | Estado de la GUI donde el control es por velocidad: FVC activo, JTC inactivo, `/cmd_vel` publicado. |
| **mock / `GenericSystem`** | Hardware simulado de `ros2_control`: devuelve el estado actual como comando, no mueve nada. Se activa con `use_mock_hardware:=true` y produce `<plugin>mock_components/GenericSystem</plugin>` (`r2c/staubli.r2c.xacro:41-43`). |
| **`MessageHeader`** | Cabecera VAL3 de 8 bytes: `nMsgType`, `nPayloadSize`, `nMsgSequenceNumber`, `nProtocolVersion`, `nChecksum`. `val3/userapp/ros2_server/Types/MessageHeader`. |
| **`mMdesc.blend = off`** | Desactiva el blending del descriptor de movimiento en CS9. `start.pgx:37`. |
| **move_group** | Nodo de `moveit_ros_move_group`: planifica, verifica y ejecuta. ⚠ `move_group_name` en `staubli_servo.yaml:45` declara `staubli_arm`, no `move_group`. |
| **MoveIt** | Framework de planificación y ejecución. `staubli_moveit_config` + `move_group` + `rviz2_moveit`. |
| **`moveit_simple_controller_manager`** | Backend de ejecución de MoveIt que traduce un plan a llamadas `FollowJointTrajectory`. `staubli_moveit_config/config/moveit_controllers.yaml`. |
| **MoveIt Servo** | Streaming de comandos cartesianos. ⚠ Roto: `command_out_topic` apunta a `/forward_position_controller/commands` (`staubli_servo.yaml:70`), controlador que no existe. |

## N

| Término | Definición |
|---|---|
| **`nMsgType`** | Tipo de trama: `0x01` RobotState, `0x02` RobotCommand, `0x03` DiagnosticData. `protocol.hpp:98-100`; confirmado por `initRobotStateMsg.pgx:33` y `initRobotCommandMsg.pgx:32`. |
| **`nMotionCmdType`** | Tipo de comando de movimiento: `0x01` STOP, `0x02` JOINT_POSITION, `0x03` JOINT_VELOCITY. `types.hpp:37-40`; el lado VAL3 compara contra `2` (`updateJointPositionControl.pgx:59`) y `3` (`updateJointVelocityControl.pgx:40`). |
| **`nPayloadSize`** | Longitud del payload. `initRobotStateMsg.pgx:34` = 124; `initRobotCommandMsg.pgx:35` = 47. ⚠ `encodeHeader.pgx` escribe el número de secuencia en este campo. |
| **`nMsgSequenceNumber`** | Contador incremental de trama, generado en el host por `get_current_message_sequence()`. |
| **`nProtocolVersion`** | `1`. `protocol.hpp:40`. |
| **Nav2** | Framework de navegación. No se usa directamente; `nav2_util::LifecycleNode` sí aparece en la cadena de `ros_gz_sim` y en `staubli_robot_driver`. |

## O

| Término | Definición |
|---|---|
| **Ogre2** | Motor de render de Gazebo. Configurado en el plugin `Sensors` del mundo (`render_engine: ogre2`), `staubli_robot_description/worlds/inspection_cell.sdf`. |
| **`operation_mode_status`** | Campo de `RobotState`. Si es distinto de 0 tras un cambio a POSITION, el host emite WARN y pasa a HOLD. `staubli_hardware_interface.cpp`. |
| **output (salida)** | Interfaz de `ros2_control` de tipo `gpio` para salidas digitales y analógicas, declarada en `staubli.r2c.xacro:55,57`. |

## P

| Término | Definición |
|---|---|
| **`parameter_bridge`** | Nodo de `ros_gz_bridge` que republica un topic con dos tipos. Se usa para `/clock` y `/camera/image_raw` (`launch_gazebo.launch.py:183`). |
| **planificación / planner** | Algoritmo que genera un camino en MoveIt. `ompl` está activo; `pilz_industrial_motion_planner` está declarado sin `default: true`. |
| **planning_frame** | Marco en el que se expresan los objetivos: `base_link` en `staubli_trajectory/config/points.yaml`. |
| **pluginlib** | Registro de plugins de ROS 2. `StaubliHardwareInterface` se exporta con `pluginlib_export_plugin_description` en `staubli_robot_driver/CMakeLists.txt`. |
| **P1 / P2 / P3** | Los tres puntos taught de la trayectoria en `points.yaml`. |
| **`point_type`** | `cartesian` (pasa por IK, `trajectory_builder.py:94`) o `joint` (posición articular directa, `:154`). |
| **`PointsConfig`** | Lector y validador de `points.yaml`. `staubli_trajectory/staubli_trajectory/points_config.py:38`; validación en `:100`. |
| **PROTOCOL_VERSION** | `1`. `communication/protocol.hpp:40`. |
| **PyKDL** | Binding de Python de KDL. Base de `Tx260lKinematics`. |
| **PySide6** | Toolkit Qt para Python. Base de la GUI. |

## Q

| Término | Definición |
|---|---|
| **QoS** | Quality of Service: fiabilidad, durabilidad, historia e histéresis de un topic. Ejemplos del repo: `/joint_states` depth 1 + VOLATILE; `/robot_description` depth 1 + TRANSIENT_LOCAL; imagen con perfil `sensor_data` (BEST_EFFORT). |
| **`q`** | Vector de posiciones articulares, 6 elementos, en radianes. |
| **`q̇` / `q̈`** | Velocidades y aceleraciones articulares. En `perform_command_mode_switch()` se ponen `q̇_cmd = 0` y `q̈_cmd = 0`. |
| **`quaternion_xyzw`** | Formato de cuaternión `[x, y, z, w]`. `rpy_degrees_to_quaternion_xyzw()` en `staubli_gui/staubli_gui/user_points.py`. |
| **Quick Start** | Sección del README con los comandos de arranque. `README.md:28-70`. |

## R

| Término | Definición |
|---|---|
| **r2c (URDF-to-ros2_control)** | Xacro que decide el hardware: `StaubliHardwareInterface` (real), `GenericSystem` (mock) o `GazeboSimSystem` (simulación). `staubli_robot_description/r2c/staubli.r2c.xacro:39-58`. |
| **`random_seed` / `n_random_seeds`** | Semilla y número de intentos de la IK multi-semilla: `42` y `30`. `tx2_60l_ik.py`. |
| **`read()`** | Método del `SystemInterface` que lee el estado. En `StaubliHardwareInterface` decodifica el `RobotState` y rellena las interfaces de `ros2_control`. |
| **`readDiagnosticData()`** | ⚠ Nunca invocado: `robot_driver.cpp:182` fija `diag_interface_ready = true;` con la llamada comentada. |
| **`read_message()`** | Decodifica la trama recibida y comprueba la frescura (`status.time_since_received < 100 ms`). |
| **real-time / non-real-time** | Clasificación del tráfico en `robot_driver`: un hilo de mayor prioridad sirve el socket de control. |
| **`resetMotion()`** | VAL3: cancela el movimiento en curso. `start.pgx:40`. |
| **ROS_DOMAIN_ID** | Variable de entorno que aísla redes ROS. |
| **`ros2_control_node`** | Ejecutable que crea el `controller_manager` y carga las interfaces de hardware. |
| **RPY (Roll-Pitch-Yaw)** | Ángulos de rotación del sensor de cámara: `rpy="2.9391 -1.3969 -2.9714"` rad en `staubli_camera.xacro`. |
| **RViz** | Visualizador 3D. `rviz2_moveit` con `staubli_moveit_config/config/moveit.rviz`. |

## S

| Término | Definición |
|---|---|
| **SAFE (modo de la GUI)** | Estado con solo JSB activo; JTC y FVC inactivos. |
| **scaling de trayectoria** | `default_velocity_scaling_factor` y `default_acceleration_scaling_factor`, ambos `0.1` en `moveit_controllers.yaml`. |
| **search_resolution** | `0.005` en `kinematics.yaml`. Resolución de búsqueda de la IK. |
| **`send_goal_async`** | Cliente ROS 2 de acciones: envía un goal sin bloquear. `staubli_gui/staubli_gui/gui.py`. |
| **sensors (plugin de Gazebo)** | Sistema que produce los topics de los sensores del mundo, incluida la cámara. `inspection_cell.sdf`. |
| **serialización** | Conversión entre estructuras C++ y bytes de la trama. `staubli_robot_driver/src/communication/serialization/`. |
| **service (servicio ROS 2)** | Interfaz petición/respuesta. `/controller_manager/list_controllers` y `/controller_manager/switch_controller`. |
| **servo (del joystick)** | Nodo `staubli_joystick_servo/staubli_joystick_servo/servo_node.py`: convierte `/cmd_vel` en velocidades articulares. |
| **`sio.cfx`** | Configuración de E/S del robot. `staubli_robot_driver/val3/configs/sio.cfx`. |
| **spawner** | Nodo de `controller_manager` que carga y activa un controlador. Tres en el flujo Gazebo; dos en el de robot. |
| **spawning (creación del robot)** | Servicio `/world/inspection_cell/create` con `name = staubli` y `-z 0.30` (`launch_gazebo.launch.py:155`). |
| **SRDF** | Semantic Robot Description Format: grupos, EOS y límites. `staubli_robot_description/srdf/staubli.srdf.xacro`. |
| **STRICT (strictness)** | Modo de `switch_controller` en el que se aplica todo o nada. `controller_manager.py:12-14` documenta que no es idempotente. |
| **supervision (10 campos)** | `SupervisionState` con 10 elementos, publicados como interfaces de estado. |
| **switch_controller** | Servicio que activa y desactiva controladores con `STRICT`, `activate_asap=False`, `timeout=5 s`. |
| **system plugins (Gazebo)** | Los 5 sistemas cargados en el mundo: `Physics`, `UserCommands`, `SceneBroadcaster`, `Contact`, `Sensors`. `inspection_cell.sdf`. |

## T

| Término | Definición |
|---|---|
| **taskCreateSync** | Creación de tarea periódica con flag de overrun. `start.pgx:45`. |
| **`TcpSocket`** | Implementación TCP declarada en `tcp_socket.hpp` pero **no compilada** y con un include inexistente (`src/communication/tcp_socket.cpp:18` incluye `"tcp_socket_impl.hpp"`). `socket_factory.cpp:31` lanza `std::runtime_error("TCP protocol not implemented yet")`. |
| **ThrottleLogMsg / `logThrottleMsg.pgx`** | Log de VAL3 limitado en frecuencia. `val3/userapp/ros2_server/Utils/`. |
| **tool0** | Marco del efector final. Padre de `camera_link`. |
| **trajectory_msgs/JointTrajectory** | Mensaje de trayectoria (puntos y timestamps). `FollowJointTrajectory.Goal.trajectory`. |
| **trayectoria (modo)** | Estado de la GUI donde el control es por posición: JTC activo, FVC inactivo. |
| **`Twist`** | `geometry_msgs/Twist`: velocidad lineal (3) + angular (3) en el marco del comando. |
| **TX2-60L** | Modelo del robot: 6 ejes. `robot_model:=tx2_60l`. |
| **`Tx260lKinematics`** | Clase de cinemática del TX2-60L en `tx2_60l_ik.py`, compartida por GUI, trajectory y servo. |

## U

| Término | Definición |
|---|---|
| **UDP** | Transporte del protocolo host↔CS9. `UDPSocket` / `UDPSocketFactory`. Sin conexión, sin garantía de orden. |
| **`update()`** | Método del `SystemInterface` que escribe los comandos. En el driver real envía el `RobotCommand` a 250 Hz. |
| **URDF** | Unified Robot Description Format. Generado por Xacro y publicado en `/robot_description`. |
| **use_gazebo_hardware** | Argumento de launch que activa `GazeboSimSystem` y el plugin `libgz_ros2_control-system.so`. |
| **use_mock_hardware** | Argumento de launch que activa `GenericSystem`. |
| **use_sim_time** | Si el nodo usa `/clock` en vez del reloj del sistema. `true` en el flujo Gazebo; `False` hardcodeado en `launch_robot_control.launch.py:144`. |

## V

| Término | Definición |
|---|---|
| **VAL3** | Lenguaje de programación del CS9. Ficheros `.pgx` en `staubli_robot_driver/val3/userapp/ros2_server/`. |
| **velocity control** | Control por velocidad. `JOINT_VELOCITY = 0x03` está declarado pero ⚠ rechazado en el host (`staubli_hardware_interface.cpp:551-557`) y el lado VAL3 registra `"not yet implemented"` (`updateJointVelocityControl.pgx`). |
| **`v_tool`** | Velocidad de la herramienta (6 componentes). Objetivo del servo. |
| **VTS** | Herramienta de validación de Staubli. **DESCONOCIDO**: no se referencia en el repositorio. |

## W

| Término | Definición |
|---|---|
| **watchdog** | Temporizador de seguridad que detiene el movimiento si un mensaje no llega. `/cmd_vel` y `/joint_states`, 0.2 s. `servo_node.py:139-150`. |
| **waypoint** | Punto de la trayectoria: `q` (6 posiciones), goal, flags y tiempo. `WaypointRecord` en `trajectory_builder.py`. |
| **`waypoint_flags`** | Bits de diagnóstico de un waypoint (solvability, límites). `trajectory_builder.py:100-130`. |
| **warehouse** | Persistencia de estados de MoveIt: `warehouse_ros_sqlite` en `~/.ros/warehouse_ros.sqlite`. |
| **world (Gazebo)** | Mundo de simulación: `staubli_robot_description/worlds/inspection_cell.sdf`. |
| **write()** | Alias de `update()` en la interfaz de hardware. |

## X

| Término | Definición |
|---|---|
| **Xacro** | Motor de plantillas XML que genera el URDF. `xacro staubli.urdf.xacro ...` se ejecuta como `Command` en `launch_gazebo.launch.py:90`. |
| **`x_robotState` / `x_robotCommand`** | Parámetros de referencia en los programas VAL3, de tipo `RobotState` y `RobotCommand`. |

## Z

| Término | Definición |
|---|---|
| **z (coordenada)** | `Twist.linear.z`; también el offset `-z 0.30` del spawning (`launch_gazebo.launch.py:155`). |
| **zero twist** | `Twist` con todos los ceros, usada para frenar. `gui.py:1998` (`_send_zero_twist`). |
| **ZMQ / YARP / VCS** | **DESCONOCIDO**: no aparecen en el repositorio. |

---

## Apéndice 1: rutas y ficheros clave

| Ruta | Contenido |
|---|---|
| `staubli_robot_driver/include/staubli_robot_driver/communication/protocol.hpp` | `MAGIC_NUMBER`, `PROTOCOL_VERSION`, `MessageType`, `MessageHeader`, puertos por defecto (8080 / 8081) |
| `staubli_robot_driver/include/staubli_robot_driver/types.hpp` | `CommandType`, `SafetyStatus`, `StatusFlag`, `ValidFieldFlag` |
| `staubli_robot_driver/include/staubli_robot_driver/hardware_interface.hpp` | `StaubliHardwareInterface`, `Mode`, `SupervisionState` |
| `staubli_robot_driver/src/communication/socket_factory.cpp` | Fábrica de sockets: UDP implementado, TCP lanza excepción (`:31`) |
| `staubli_robot_driver/src/communication/serialization/` | `decodeRobotStateMsg`, `encodeRobotCommandMsg`, `decodeHeader`, `encodeHeader`, `initHeader` |
| `staubli_robot_driver/src/staubli_hardware_interface.cpp` | `SystemInterface` real: `read()`, `write()`, `prepare_command_mode_switch()`; rechaza velocidad/par (`:551-557`) |
| `staubli_robot_driver/src/robot_driver.cpp` | API pública del driver; `diag_interface_ready` forzado (`:182`) |
| `staubli_robot_driver/val3/userapp/ros2_server/start.pgx` | Arranque: versión, BOM, `stopMove()`, `resetMotion()`, `autoConnectMove(true)`, `controlTask` a 4 ms |
| `staubli_robot_driver/val3/userapp/ros2_server/controlTask.pgx` | Bucle de control de la aplicación |
| `staubli_robot_driver/val3/userapp/ros2_server/Control/` | `updateJointPositionControl`, `updateJointVelocityControl` (⚠ "not yet implemented"), `stopJoint*Control` |
| `staubli_robot_driver/val3/userapp/ros2_server/Serialization/encodeHeader.pgx` | ⚠ escribe `nMsgSequenceNumber` en el campo `nPayloadSize` |
| `staubli_robot_driver/val3/userapp/ros2_server/Types/` | `MessageHeader`, `RobotCommand`, `RobotState`, `RobotCommandMsg`, `RobotStateMsg`, `ThrottleLogMsg` |
| `staubli_robot_description/urdf/staubli.urdf.xacro` | Xacro raíz: macro del brazo, cámara, `ros2_control` |
| `staubli_robot_description/urdf/include/staubli_camera.xacro` | `camera_link`, sensor Gazebo, visuales del indicador láser |
| `staubli_robot_description/r2c/staubli.r2c.xacro` | Selección de hardware y puertos (11000 / 11001) |
| `staubli_robot_description/worlds/inspection_cell.sdf` | `ground_plane`, `sun`, `inspection_table`, `robot_platform`, `part_1`; 5 system plugins |
| `staubli_robot_description/meshes/` | `laser_triangle.obj` y el resto de mallas del robot |
| `staubli_bringup/config/controllers.yaml` | JSB / JTC / FVC, `update_rate: 250`, `allow_partial_joints_goal: false` |
| `staubli_bringup/launch/launch_gazebo.launch.py` | RSP + GzServer + bridges + spawners + GUI de Gazebo |
| `staubli_bringup/launch/launch_gazebo_gui.launch.py` | Incluye `launch_gazebo` + `gui.launch.py`; fija `robot_model: "tx2_60l"` (`:58`) |
| `staubli_bringup/launch/launch_robot_control.launch.py` | Camino real / mock; `use_sim_time: False` hardcodeado (`:144`) |
| `staubli_bringup/launch/launch_moveit.launch.py` | `wait_for_robot_ready` + `staubli_moveit.launch.py` |
| `staubli_gui/staubli_gui/gui.py` | GUI PySide6; máquina de estados; goals; importación perezosa de `staubli_trajectory` |
| `staubli_gui/staubli_gui/controller_manager.py` | `switch()` con filtrado STRICT de 3 pasos (`:79-123`) |
| `staubli_gui/staubli_gui/frame_control.py` | `FrameControlPanel`: seis pares de botones ± → `Twist` |
| `staubli_gui/staubli_gui/user_points.py` | `UserPointStore`, `user_point_from_joint_position`, `rpy_degrees_to_quaternion_xyzw` |
| `staubli_joystick_servo/staubli_joystick_servo/servo_node.py` | `/cmd_vel` → DLS → `/forward_velocity_controller/commands`; watchdogs |
| `staubli_joystick_servo/config/servo_params.yaml` | `command_timeout`, `joint_state_timeout`, deadzone, rampas |
| `staubli_moveit_config/config/staubli_servo.yaml` | ⚠ `move_group_name: staubli_arm` (`:45`), `command_out_topic: /forward_position_controller/commands` (`:70`) |
| `staubli_moveit_config/config/kinematics.yaml` | `KDLKinematicsPlugin`, `search_resolution`, `timeout` |
| `staubli_moveit_config/config/joint_limits.yaml` | Límites por articulación y factores de escalado |
| `staubli_moveit_config/config/moveit_controllers.yaml` | `moveit_simple_controller_manager`, JTC, escalado 0.1 |
| `staubli_moveit_config/config/moveit.rviz` | Configuración de `rviz2_moveit` |
| `staubli_moveit_config/config/pilz_cartesian_limits.yaml` | Límites cartesianos de Pilz |
| `staubli_moveit_config/srdf/staubli.srdf.xacro` | Grupos, EOS y límites semánticos |
| `staubli_trajectory/staubli_trajectory/tx2_60l_ik.py` | `Tx260lKinematics` (FK / IK / Jacobiano), `solve_qdot_dls` |
| `staubli_trajectory/staubli_trajectory/points_config.py` | Lectura y validación de `points.yaml` |
| `staubli_trajectory/staubli_trajectory/trajectory_builder.py` | Waypoints → `JointTrajectory` |
| `staubli_trajectory/staubli_trajectory/joint_trajectory_client.py` | Cliente de `FollowJointTrajectory`; `_build_goal` (`:117`) |
| `staubli_trajectory/config/points.yaml` | Puntos P1/P2/P3, `point_type`, `planning_frame`, tolerancias |
| `patches/gz_ros2_control/fix_command_mode_switch.patch` | Parche manual para `gz_ros2_control` 3.0.8: sustituye el operador AND bitwise por OR al limpiar los bits de control |
| `.docker/docker-compose.yaml` | Camino Docker de `launch_demo.launch.py` |
| `.github/workflows/ci.yml` | CI de `jazzy` (`:21`) |
| `.github/workflows/ci-format.yml` | `pre-commit` en `ros:jazzy-ros-base` (`:15,24`) |
| `.github/workflows/docker.yml` | Build de imagen (`:35`) |
| `staubli_gui/plots/jogger/*.xml` | Configuración de plotjuggler (⚠ no la carga ningún nodo) |

---

## Apéndice 2: códigos del protocolo

Definidos en `communication/protocol.hpp` y `types.hpp`, y confirmados por
`initRobotStateMsg.pgx` e `initRobotCommandMsg.pgx`.

### `nMsgType` (tipo de trama)

| Valor | Significado | Payload |
|---|---|---|
| `0x01` | `ROBOT_STATE` | 124 bytes (`initRobotStateMsg.pgx:34`) |
| `0x02` | `ROBOT_COMMAND` | 47 bytes (`initRobotCommandMsg.pgx:35`) |
| `0x03` | `DIAGNOSTIC_DATA` | ⚠ nunca leído por el host (`robot_driver.cpp:182`) |

### `nMotionCmdType` / `CommandType` (`types.hpp:37-43`)

| Valor | Significado | Estado en el host |
|---|---|---|
| `0x01` | `STOP` | Implementado: `send_stop_all()` |
| `0x02` | `JOINT_POSITION` | Implementado: camino principal del robot real |
| `0x03` | `JOINT_VELOCITY` | ⚠ Rechazado en C++ (`:551-557`) y "not yet implemented" en VAL3 |
| `0x04` | `JOINT_TORQUE` | ⚠ Rechazado en C++ (`:551-557`) |
| `0x0A` | `DECELERATION_JOINT_POSITION` | Declarado, nunca generado |
| `0x0B` | `DECELERATION_JOINT_VELOCITY` | Declarado, nunca generado |

### `StatusFlag` (máscara de 16 bits, `types.hpp:86-93`)

| Bit | Valor | Significado |
|---|---|---|
| 0 | `0x0001` | `POWER_ON` |
| 1 | `0x0002` | `BRAKES_RELEASED` |
| 2 | `0x0004` | `MOTION_POSSIBLE` |
| 3 | `0x0008` | `IN_MOTION` |
| 4 | `0x0010` | `ERROR_STATE` |
| 5 | `0x0020` | `ESTOP_PRESSED` |

### `ValidFieldFlag` (máscara de 16 bits, `types.hpp:96-103`)

| Bit | Valor | Significado |
|---|---|---|
| 0 | `0x0001` | `JOINT_POSITIONS` |
| 1 | `0x0002` | `JOINT_VELOCITIES` |
| 2 | `0x0004` | `JOINT_TORQUES` |
| 3 | `0x0008` | `FT_SENSOR` |
| 4 | `0x0010` | `DIGITAL_INPUTS` |
| 5 | `0x0020` | `ANALOG_INPUTS` |

### `SafetyStatus` (`types.hpp:74-80`)

| Valor | Significado |
|---|---|
| `0x00` | `NO_SAFETY_STOP` |
| `0x01` | `WAIT_FOR_RESTART` |
| `0x02` | `SS1` — parada segura 1 |
| `0x03` | `SS2` — parada segura 2 |
| `0x04` | `WAIT_FOR_WMS` — a la espera del módulo de seguridad |

### Cabecera de trama (8 bytes)

| Offset | Tamaño | Campo | Valor |
|---|---|---|---|
| 0 | 2 B | número mágico | `0xABCD` |
| 2 | 1 B | `nProtocolVersion` | `1` |
| 3 | 1 B | `nMsgType` | ver tabla anterior |
| 4 | 2 B | `nMsgSequenceNumber` | contador incremental |
| 6 | 2 B | `nPayloadSize` | longitud del payload — ⚠ `encodeHeader.pgx` escribe aquí el número de secuencia |
| 8 | … | payload | datos del mensaje |

### Rango de control por cliente (`staubli_joystick_servo/config/servo_params.yaml`)

| Parámetro | Valor | Efecto |
|---|---|---|
| `max_linear` | 0.15 m/s | `clamp_vector` de la parte lineal |
| `max_angular` | 0.50 rad/s | `clamp_vector` de la parte angular |
| `deadzone_lin` / `deadzone_ang` | 0.01 | zona muerta por eje |
| `command_timeout` | 0.2 s | watchdog de `/cmd_vel` |
| `joint_state_timeout` | 0.2 s | watchdog de `/joint_states` |
| `damping_lambda` (DLS) | 0.05 | amortiguamiento de la Pseudoinversa |
| `singularity_threshold` | 0.02 | por debajo, aumenta el amortiguamiento efectivo |
