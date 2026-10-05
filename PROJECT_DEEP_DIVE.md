# PROJECT_DEEP_DIVE.md

Detalle exhaustivo, paquete por paquete y archivo por archivo, del repositorio
`~/staubli_ws/src/staubli_driver_ros2` en la rama `feature/joystick-control`
(commit `4d195ec`).

Todos los números de línea corresponden al commit indicado. Cuando un dato no
puede determinarse leyendo el código, se marca explícitamente como
**DESCONOCIDO**.

---

## Índice

1. [Convenciones y estructura](#1-convenciones-y-estructura)
2. [`staubli_robot_description`](#2-staubli_robot_description)
3. [`staubli_robot_driver`](#3-staubli_robot_driver)
4. [Aplicación VAL3](#4-aplicación-val3)
5. [`staubli_bringup`](#5-staubli_bringup)
6. [`staubli_moveit_config`](#6-staubli_moveit_config)
7. [`staubli_trajectory`](#7-staubli_trajectory)
8. [`staubli_joystick_servo`](#8-staubli_joystick_servo)
9. [`staubli_gui`](#9-staubli_gui)
10. [`staubli_driver_ros2` (metapaquete), CI, Docker, docs](#10-metapaquete-ci-docker-y-documentación)
11. [Hallazgos, incoherencias y riesgos](#11-hallazgos-incoherencias-y-riesgos)
12. [Inventario de pruebas](#12-inventario-de-pruebas)

---

## 1. Convenciones y estructura

### 1.1 Convenciones de código

| Elemento | Convención | Evidencia |
|---|---|---|
| Headers C++ | Guardas `STAUBLI_ROBOT_DRIVER__<SUBDIR>__<FILE>_HPP_` | `staubli_robot_driver/include/staubli_robot_driver/communication/protocol.hpp:17` |
| Namespace C++ | `staubli_robot_driver` | `src/robot_driver.cpp:26` |
| Estilo C++ | `clang-format` con `.clang-format.yaml`; `pre-commit` con `clang-format -fallback-style=none` | `.pre-commit-config.yaml:57-60` |
| Lint C++ | `ament_cppcheck` + `ament_cpplint` (`--linelength=100`) | `.pre-commit-config.yaml:62-78` |
| Python | `black --line-length=99`, `flake8 --extend-ignore=E501`, `pyupgrade --py36-plus`, `pydocstyle` (con muchas exclusiones) | `.pre-commit-config.yaml:36-55` |
| Docstrings Python | Español, con `D100-D107, D203, D212, D404` ignorados | `.pre-commit-config.yaml:47` |
| YAML | `check-yaml --allow-multiple-documents --unsafe` (necesario para `!degrees`) | `.pre-commit-config.yaml:25-26` |
| Comentarios | Mayoritariamente en **español** en los paquetes nuevos; inglés en el driver C++ heredado | — |
| Licencia | Cabecera Apache 2.0 en C++/scripts; sin cabecera en Xacro | — |
| Exclusiones | `staubli_robot_driver/val3/` excluido de todos los hooks | `.pre-commit-config.yaml:12` |

### 1.2 Grafo de archivos del driver C++ (`staubli_robot_driver`)

```
include/staubli_robot_driver/
  ├── types.hpp                    enums: MessageType, CommandType, OperationMode,
  │                                SafetyStatus, StatusFlag, ValidFieldFlag
  ├── communication/
  │   ├── protocol.hpp             MAGIC_NUMBER, PROTOCOL_VERSION, puertos,
  │   │                            FrameHeader (8 B), Message (base)
  │   ├── messages.hpp             RobotStateMessage, RobotCommandMessage,
  │   │                            DiagnosticDataMessage, ErrorLogEntry
  │   ├── serialization.hpp        primitivas wire (LE) + validate()
  │   ├── socket.hpp               interfaz abstracta Socket + SocketFactory
  │   ├── udp_socket.hpp           UDPSocketFactory / UDPSocket
  │   └── tcp_socket.hpp           TCPSocketFactory / TCPSocket (declarado)
  ├── real_time_socket_subscriber.hpp  hilo de recepción + MessageStatus
  ├── real_time_socket_interface.hpp   plantilla bilateral (sus+pub)
  ├── mock_robot_server.hpp        servidor UDP de pruebas
  ├── robot_driver.hpp             clase RobotDriver (fachada de comunicación)
  ├── robot_driver_helpers.hpp     helpers free-function sobre RobotCommandMessage
  └── staubli_hardware_interface.hpp  plugin ros2_control

src/
  ├── communication/{protocol,messages,socket_factory,udp_socket,tcp_socket}.cpp
  ├── communication/udp_socket_impl.hpp   implementación real (Boost.Asio)
  ├── real_time_socket_subscriber.cpp
  ├── real_time_socket_interface.cpp
  ├── robot_driver.cpp
  ├── staubli_hardware_interface.cpp      <-- PLUGINLIB_EXPORT_CLASS
  ├── mock_robot_server.cpp
  └── test_receive_robot_state.cpp         ejecutable de diagnóstico
```

---

## 2. `staubli_robot_description`

**Tipo**: `ament_cmake` · **Versión**: 0.1.4 · **Licencia**: Apache 2.0

### 2.1 Dependencias (`package.xml:14-24`)

`launch`, `launch_ros`, `robot_state_publisher`, `joint_state_publisher_gui`,
`rviz2`, `urdf`, `xacro`. Test: `ament_lint_auto`, `ament_lint_common`, `xacro`.

> Nota: la dependencia real de simulación (`ros_gz_sim`, `ros_gz_bridge`,
> `gz_ros2_control`) **no está declarada** aquí; se usa solo desde
> `staubli_bringup/launch/launch_gazebo.launch.py`, y ese paquete tampoco la
> declara.

### 2.2 `urdf/staubli.urdf.xacro` (Xacro raíz)

Genera el URDF completo. Parámetros (Xacro args):

| Arg | Default | Propósito |
|---|---|---|
| `robot_model` | `???` | Selecciona `config/<model>/` |
| `name` | `staubli` | Nombre del recurso |
| `robot_prefix` | `""` | Prefijo para robots múltiples |
| `robot_ip` | `0.0.0.0` | IP del robot real |
| `laser_distance` | `0.50` | Longitud (X) del indicador láser |
| `laser_width` | `0.30` | Anchura (Y) del indicador láser |
| `laser_line_width` | `0.008` | Grosor de la línea roja |
| `use_soft_limits` | `true` | Límites de seguridad en el URDF |
| `safety_position_margin` | `0.1` | |
| `safety_velocity_margin` | `0.1` | |
| `safety_k_position` | `0.1` | |
| `safety_k_velocity` | `10.0` | |
| `use_mock_hardware` | `true` | Backend de `ros2_control` |
| `mock_sensor_commands` | `false` | Solo mock |
| `use_gazebo_hardware` | `false` | Backend de `ros2_control` |
| `gazebo_controllers_file` | `""` | Ruta a `controllers.yaml` para el plugin Gazebo |
| `force_abs_paths` | `false` | Rutas absolutas vs `package://` |

Estructura generada:
1. `<link name="world"/>`.
2. `xacro:staubli_robot` (desde `staubli_macro.xacro`) con origen `xyz="0 0 0"`.
3. `xacro:staubli_camera` sobre `link_6`.
4. `xacro:staubli_ros2_control` (desde `r2c/staubli.r2c.xacro`).
5. Bloque `<gazebo><plugin filename="libgz_ros2_control-system.so">` **solo si**
   `use_gazebo_hardware:=true`, con `<parameters>$(arg gazebo_controllers_file)</parameters>`.

### 2.3 `r2c/staubli.r2c.xacro` (ros2 to control)

Es el punto donde se decide el backend de hardware (ver `PROJECT_OVERVIEW.md` §5.1).

Bloques exportados con hardware real:

- 6 articulaciones con posición + velocidad (+ aceleración).
- GPIO `supervision` con 10 state interfaces
  (`operation_mode`, `operation_mode_status`, `safety_status`,
  `control_sequence_delay`, `brakes_released`, `motion_possible`, `in_motion`,
  `in_error`, `estop_pressed`, `wait_for_ack`) — `r2c/staubli.r2c.xacro:95-108`.
- GPIO genéricos de `staubli_r2c_gpios.xacro` (digital/analog, entradas y salidas).

Parámetros de hardware inyectados (`r2c/staubli.r2c.xacro:48-57`):
`robot_prefix`, `robot_ip`, `control_port=11000`, `diagnostic_port=11001`,
`local_control_port=11000`, `local_diagnostic_port=11001`,
`num_digital_inputs`, `num_digital_outputs`, `num_analog_inputs`,
`num_analog_outputs`.

### 2.4 `urdf/include/staubli_camera.xacro`

Cámara eye-in-hand sobre `link_6`.

| Parámetro de la macro | Default | Nota |
|---|---|---|
| `parent_link` | (obligatorio) | `link_6` cuando se llama desde el Xacro raíz |
| `xyz` | `0 0 0.07` | **Misma Z que `tool0`** (`robot_model.yaml` `end_effector.xyz`) |
| `rpy` | `2.9391 -1.3969 -2.9714` | **Radianes**, Euler XYZ. Orientación hacia abajo y afuera |
| `update_rate` | `30` | Hz |
| `width` × `height` | `640` × `480` | |
| `hfov` | `1.0472` | rad (≈60°) |
| `laser_distance` | `0.50` | Escala X de la malla del triángulo |
| `laser_width` | `0.30` | Escala Y de la malla |
| `laser_line_width` | `0.008` | Tamaño de la caja de la línea |

> **Corrección importante**: el valor `0.7031` aparece únicamente en un
> comentario del Xacro (línea 8) describiendo un "+X roll" hipotético. El `rpy`
> realmente aplicado es `2.9391 -1.3969 -2.9714` (línea 30).

Contenido de `camera_link`:
- `inertial`: masa 0.05 kg, inercia 1e-5 en los tres ejes.
- 3 visuales de carcasa (`box` 0.04³, `box` 0.06×0.05×0.045, `cylinder` r=0.016
  l=0.008 con `rpy="0 1.57079632679 0"`).
- `laser_triangle_visual`: malla `package://staubli_robot_description/meshes/laser_triangle.obj`
  con `scale="${laser_distance} ${laser_width} 1.0"`, material
  `rgba="0.10 0.70 1.0 0.25"` (cian semitransparente).
- `laser_line_visual`: `box` de tamaño `laser_line_width × laser_width × laser_line_width`
  en `xyz="${laser_distance} 0 0"`, material `rgba="1.0 0.0 0.0 0.95"` (rojo).

Sensor Gazebo (`<gazebo reference="camera_link">`):
`<sensor name="camera" type="camera">` con `<topic>/camera/image_raw</topic>`,
`always_on=1`, `visualize=true`, `format R8G8B8`, `clip near=0.1 far=100`.

**El "laser marker" no es un marcador ROS.** Es puramente geometría de URDF:
- `meshes/laser_triangle.obj` → 3 vértices `(0,0,0)`, `(1,-0.5,0)`, `(1,0.5,0)`,
  una sola cara.
- `meshes/laser_triangle.mtl` → `newmtl laser_triangle_material`, `Kd 0.1 0.7 1.0`,
  `d 0.25` (transparencia).
- Se renderiza en Gazebo y RViz como parte del link. **No hay ningún publicador
  de `visualization_msgs/Marker` en todo el repositorio** (verificado por grep:
  cero coincidencias de `visualization_msgs`, `Marker`, `MarkerArray`).

### 2.5 `config/` (tres modelos de robot)

| Modelo | `robot_model.yaml` | `dynamics.yaml` | `kinematics.yaml` | `joint_limits.yaml` | `meshes.yaml` |
|---|---|---|---|---|---|
| `tx2_60` | sí | sí | sí | sí | sí |
| `tx2_60l` | sí | sí | sí | sí | sí |
| `tx2_60l_med` | sí | sí | sí | sí | sí |

`config/tx2_60l/robot_model.yaml`:
- `has_joint_torque_sensor: false`, `has_ft_sensor: false`.
- `end_effector`: parent `link_6`, `xyz [0,0,0.07]`, `rpy [0,0,0]` → frame
  `tool0`.
- `io`: 16 entradas digitales, 16 salidas digitales, 2 entradas analógicas,
  2 salidas analógicas.

`config/tx2_60l/joint_limits.yaml` (en **grados**, con el tag YAML `!degrees`):

| Joint | `min_position` | `max_position` | `max_velocity` | `max_acceleration` | `max_effort` |
|---|---|---|---|---|---|
| `joint_1` | −180° | +180° | 300 °/s | 1000 °/s² | 126.0 |
| `joint_2` | −127° | +127° | 300 °/s | 1000 °/s² | 47.0 |
| `joint_3` | −150° | +150° | 300 °/s | 1000 °/s² | 32.0 |
| `joint_4` | −270° | +270° | 300 °/s | 1000 °/s² | 7.0 |
| `joint_5` | −120° | +120° | 300 °/s | 1000 °/s² | 8.0 |
| `joint_6` | −270° | +270° | 500 °/s | 3000 °/s² | 2.0 |

`config/tx2_60l/kinematics.yaml` (cadena cinemática, metros):

| Joint | `xyz` | `axis` |
|---|---|---|
| `joint_1` | `[0, 0, 0.210]` | `[0, 0, 1]` |
| `joint_2` | `[0, 0.11735, 0.165]` | `[0, 1, 0]` |
| `joint_3` | `[0, 0.00925, 0.40]` | `[0, 1, 0]` |
| `joint_4` | `[0, −0.1066, 0.088]` | `[0, 0, 1]` |
| `joint_5` | `[0, 0, 0.362]` | ver fichero |
| `joint_6` | ver fichero | ver fichero |

`config/initial_positions.yaml`: las 6 articulaciones a `0.0` (HOME).

`config/meshes.yaml` (tx2_60l): visual en `.dae` (Collada) y colisión en `.stl`,
resueltos con `package://staubli_robot_description`.

### 2.6 `meshes/`

- `meshes/tx2_60/visual/*.dae` y `collision/*.stl` — 10 links
  (`base_link`, `link_1`, `link_2_L`, `link_3`, `link_4_L`, `link_5`, `link_6`,
  `link_6_L_med`, más variantes `link_2` y `link_4` en `.stl`).
- `meshes/laser_triangle.{obj,mtl}` — indicador láser.
- `meshes/tx2_60/README.md` — nota de 6 líneas sobre el origen de los meshes.

### 2.7 `launch/view_robot.launch.py`, `rviz/view_robot.rviz`, `test/`

- `view_robot.launch.py`: lanza `joint_state_publisher_gui` + `robot_state_publisher`
  + `rviz2` con `rviz/view_robot.rviz`.
- `test/test_default_urdf.py` (83 líneas) — valida el URDF por defecto.
- `test/test_launch_mocked.py` (17 líneas) — smoke test del launch con topics
  simulados.
- `env-hooks/staubli_robot_description.sh.in` — hook de entorno (contenido
  **no inspeccionado en detalle**: DESCONOCIDO qué variables exporta).

---

## 3. `staubli_robot_driver`

**Tipo**: `ament_cmake`, C++17 · **Versión**: 0.1.4

### 3.1 `CMakeLists.txt` — decisiones relevantes

- `set(CMAKE_CXX_STANDARD 17)`, `-Wall -Wextra -Wpedantic`.
- Dependencias: `rclcpp`, `std_msgs`, `sensor_msgs`, `geometry_msgs`,
  `control_msgs`, `realtime_tools`, `hardware_interface`, `pluginlib`,
  `rclcpp_lifecycle`. Usa **targets modernos** (`rclcpp::rclcpp`, `${std_msgs_TARGETS}`)
  porque `ament_target_dependencies()` fue eliminado en `ament_cmake` moderna
  (`staubli_robot_driver/CMakeLists.txt:90-92`).
- `find_package(Boost REQUIRED)` + `Boost::headers` porque en Ubuntu 26.04 /
  Boost 1.90 `Boost.System` es header-only y no existe como component
  localizable (`staubli_robot_driver/CMakeLists.txt:53-61`).
- `find_package(backward_ros REQUIRED)` para stack traces.
- **Dos librerías**:
  - `staubli_robot_driver` — `LIBRARY_SOURCES` (`:64-77`): `protocol.cpp`,
    `messages.cpp`, `socket_factory.cpp`, `udp_socket.cpp`,
    `real_time_socket_subscriber.cpp`, `real_time_socket_interface.cpp`,
    `robot_driver.cpp`, `mock_robot_server.cpp`. **No incluye `tcp_socket.cpp`.**
  - `staubli_robot_driver_hardware_interface` — solo `staubli_hardware_interface.cpp`.
- Exporta el plugin: `pluginlib_export_plugin_description_file(hardware_interface, staubli_hardware_interface.xml)`.
- Instalación de scripts Python (`upload_val3_server.py`, `download_logs.py`,
  `show_logs.py`, `remove_val3_server.py`) en `lib/staubli_robot_driver`.
- Instala el árbol `val3/` completo en `share/staubli_robot_driver/val3`.
- 7 gtests bajo `BUILD_TESTING`; se desactivan `ament_copyright` y
  `ament_cpplint` porque los ficheros ya llevan cabecera propia
  (`staubli_robot_driver/CMakeLists.txt:177-184`) y `ament_lint_auto_find_test_dependencies()` está
  comentado (`:170`).

### 3.2 Protocolo (`communication/protocol.hpp`, `communication/protocol.cpp`)

| Constante | Valor |
|---|---|
| `MAGIC_NUMBER` | `0xABCD` (43981 decimal) |
| `PROTOCOL_VERSION` | `1` |
| `DEFAULT_CONTROL_PORT` | `8080` |
| `DEFAULT_DIAGNOSTICS_PORT` | `8081` |
| `MAX_FRAME_SEQUENCE_NUMBER` | `65535` |

Formato de trama (`protocol.hpp:87-103`):

```
 ┌────────────┬───────────┬─────────────┬───────────┬─────────────┬──────────────┐
 │ Magic (2B) │ Proto (1B)│ Type  (1B)  │  Seq  (2B) │ Payload (2B) │   Payload    │
 │  uint16 LE │  uint8    │  uint8      │  uint16 LE│  uint16 LE  │  variable    │
 └────────────┴───────────┴─────────────┴───────────┴─────────────┴──────────────┘
    offset 0     offset 2     offset 3        offset 4     offset 6
                                                  Tamaño total de cabecera: 8 bytes
```

`Message` es la clase base abstracta con `get_serialized_size()`, `serialize()`,
`deserialize()` y sobrecargas para `std::vector<uint8_t>`.

### 3.3 Mensajes (`communication/messages.hpp`, `messages.cpp`)

**`RobotStateMessage`** (robot → host, `ROBOT_STATE = 0x01`):
`joint_positions[6]`, `joint_velocities[6]`, `joint_torques[6]`, `ft_sensor[4]`,
`digital_inputs[16]` (bool), `analog_inputs[4]`, más flags
`has_joint_positions`, `has_joint_velocities`, `has_joint_torques`,
`has_digital_inputs`, `has_analog_inputs`, `has_ft_sensor`, y campos de
supervisión: `operation_mode`, `operation_mode_status`, `safety_status`,
`sequence_delay`, `brakes_released`, `motion_possible`, `in_motion`,
`error_state`, `estop_pressed`.

Unidades: **grados** para posiciones y velocidades articulares, N·m para
pares.

**`RobotCommandMessage`** (host → robot, `ROBOT_COMMAND = 0x02`):
`command_type`, `command_reference[6]`, `digital_outputs[16]`,
`analog_outputs[4]`, `controller_period` (ms), `header.sequence_number`.

**`DiagnosticDataMessage`** (`DIAGNOSTIC_DATA = 0x03`):
`robot_firmware_version{major,minor,patch}`, `robot_controller_type`,
`error_log[10]` (`ErrorLogEntry`). **No se consume todavía** (ver §11.6).

### 3.4 Sockets

`socket.hpp` define la interfaz abstracta `Socket` (`connect`, `disconnect`,
`is_connected`, `is_receiving`, `start_receive_thread`, `stop_receive_thread`,
`send`, `receive`) y `SocketFactory`.

`socket_factory.cpp:27-35`:
```cpp
case ProtocolType::UDP: return UDPSocketFactory::create();
case ProtocolType::TCP: throw std::runtime_error("TCP protocol not implemented yet");
```

> **TCP no está implementado.** Además `src/communication/tcp_socket.cpp:18`
> incluye `"tcp_socket_impl.hpp"`, fichero **inexistente** en el repositorio. No
> rompe el build porque `CMakeLists.txt` no lo añade a `LIBRARY_SOURCES`.

`udp_socket.cpp` + `udp_socket_impl.hpp`: implementación real con **Boost.Asio**,
recepción asíncrona en hilo dedicado.

### 3.5 `RealTimeSocketSubscriber` / `RealTimeSocketInterface`

- `RealTimeSocketSubscriber<MessageSub>`: arranca un hilo de recepción
  (`start_receive_thread`), deserializa en un buffer circular y expone
  `read_message(Message&, MessageStatus&)` sin bloquear. `MessageStatus` contiene
  `time_since_received` y `lost_packages`.
- `RealTimeSocketInterface<MessageSub, MessagePub>`: hereda del anterior y añade
  `send_message()` (**bloqueante**, según el docstring de
  `real_time_socket_interface.hpp:47-56`).
- Se instancian explícitamente para los dos sentidos
  (`real_time_socket_interface.hpp:67-68`):
  `RealTimeSocketInterface<RobotStateMessage, RobotCommandMessage>` y el inverso.

### 3.6 `RobotDriver` (`robot_driver.hpp`, `robot_driver.cpp`)

Fachada de comunicación. `NetworkConfig` (`robot_driver.hpp:45-58`): `robot_ip`,
`local_ip`, `control_port`, `local_control_port`, `diagnostics_port`,
`local_diagnostics_port`.

**`connect(config, timeout_ms = 5000)`** — secuencia (`robot_driver.cpp:54-246`):
1. Valida `robot_ip` no vacío.
2. Desconecta si ya había conexión.
3. Conecta socket de control → crea `RealTimeSocketInterface<State, Command>`.
4. Conecta socket de diagnóstico → crea `RealTimeSocketSubscriber<DiagnosticData>`.
5. Comprueba `is_ready()` de ambas interfaces.
6. Bucle de espera (500 ms entre intentos) hasta el timeout leyendo estado.
   **`diag_interface_ready` se fuerza a `true`** (`:182`) — ver §11.6.
7. Comprueba `header.protocol_version == PROTOCOL_VERSION`; si no, desconecta.
8. Log de la versión de firmware; `TODO` de validar VAL3.

**`read_robot_state(state)`** (`:355-385`): verifica `is_ready()`, lee, valida
`message_type == ROBOT_STATE`, comprueba **obsolescencia** contra
`state_staleness_timeout_` (100 ms por defecto, `robot_driver.hpp:138-139`), y solo
entonces copia a la salida y actualiza `last_valid_sequence_id_`. Los paquetes
perdidos se contabilizan pero el log está comentado (`:379`).

**`send_robot_command(command)`** (`:323-353`): verifica readiness, avisa si un
STOP viene con referencia distinta de cero, y delega en
`control_interface_->send_message()`. `TODO(tpoignonec)`: validar límites de
posición/velocidad.

**`send_stop_all_command()`** (`:306-321`): construye un STOP con
`prepare_robot_command_message()` (que pone el sequence number actual, pone a
cero `command_reference`, `digital_outputs` y `analog_outputs`).

**`disconnect()`** (`:248-293`): resetea interfaces, para hilos de recepción,
cierra sockets y llama `reset_data()`.

### 3.7 `robot_driver_helpers.hpp`

Funciones libres para construir comandos:
`set_command_stop`, `set_command_joint_position`, `set_command_joint_velocity`,
`set_sequence_number`, `prepare_robot_command_message`.
`set_command_*` validan que el tamaño del vector coincida con
`command_reference.size()` (6).

### 3.8 `StaubliHardwareInterface` — el plugin de `ros2_control`

`staubli_hardware_interface.cpp:691-694`:
```cpp
PLUGINLIB_EXPORT_CLASS(staubli_robot_driver::StaubliHardwareInterface,
                       hardware_interface::SystemInterface)
```

**`on_init(params)`** (`:58-139`): lee `params.hardware_info.joints.size()`,
inicializa los vectores de posiciones/velocidades/esfuerzos/órdenes a `NaN`
(vía `std::fill`, sin `resize`, para no invalidar los punteros crudos de las
`CommandInterface`), parsea el número de E/S y valida contra
`MAX_DIGITAL_IO = 16` y `MAX_ANALOG_IO = 4`. Crea el `RobotDriver`.

**`on_configure(previous_state)`** (`:141-186`): lee `robot_ip`,
`control_port`, `diagnostic_port`, `local_control_port`,
`local_diagnostic_port`; construye `NetworkConfig`; llama
`connect(net_config, 5000)`.

**`on_activate`** (`:188-198`): fuerza `current_control_mode_ = STOP`.

**`on_cleanup`** (`:200-211`): `disconnect()`.

**`on_shutdown`** (`:213-227`): si el modo actual no es STOP, envía STOP; luego
desconecta.

**`on_error`** (`:229-244`): envía STOP si está conectado; si no, devuelve
`ERROR`.

**`export_state_interfaces()`** (`:246-346`): por articulación,
`HW_IF_POSITION`, `HW_IF_VELOCITY`, `HW_IF_EFFORT`; los 10 campos de
`supervision`; `digital_input_{i}` y `analog_input_{i}`.
**Las lecturas de salidas (digital/analog) están comentadas** (`:330-344`): el
plugin exporta comandos de salida en `export_command_interfaces()` pero **no**
exporta `StateInterface` para poder leerlas.

**`export_command_interfaces()`** (`:348-393`): por articulación
`HW_IF_POSITION`, `HW_IF_VELOCITY` y `HW_IF_ACCELERATION` (esta última solo para
que el limitador de `ros2_control` aplique límites, con `TODO` de retirarla);
más `digital_output_{i}` y `analog_output_{i}`.

**`read(time, period)`** (`:395-415`): `read_robot_state(state_msg_)` →
`update_state(state_msg_)`.

**`write(time, period)`** (`:417-496`): construye `RobotCommandMessage` con
`sequence_number` actual, `controller_period = 1e3 * period.seconds()` (ms) y el
`command_type` vigente. Controles de seguridad:
- Si el modo no es STOP y el robot está en error o `control_state == INVALID` →
  STOP + `ERROR`.
- Stop → referencia a cero.
- `JOINT_POSITION` / `JOINT_VELOCITY` → `rad2deg()` del comando.
- E/S digitales: `> 0.5` → `true`.
- **Detección de `NaN`** en la referencia → STOP + `ERROR`.

**`resolve_mode(start_interfaces)`** (`:498-513`): busca en orden `position`,
`velocity`, `effort`, `acceleration` dentro del nombre de cada interfaz.

**`prepare_command_mode_switch(start, stop)`** (`:515-576`):
- `JOINT_POSITION` → aceptado.
- `JOINT_VELOCITY` → **rechazado con `ERROR`**.
- `JOINT_TORQUE` → **rechazado con `ERROR`**.
- `STOP` o solo-aceleración → aceptado, se cachea como `STOP`.
- `TODO`: no se valida que las interfaces se paren/arranquen juntas; validación
  de contra-modo pendiente.

**`perform_command_mode_switch(start, stop)`** (`:578-622`): consume el modo
cacheado (o `STOP` si no se cacheó). Para `JOINT_POSITION` valida estado del
robot: rechaza si `error_state`, rechaza si `!motion_possible` (frenos
puestos), avisa con `WARN` si `operation_mode_status != 0` (HOLD). Después
**inicializa de forma segura los comandos**: copia posiciones actuales a
`hw_joint_position_commands_` con `std::copy` y pone a cero velocidad y
aceleración con `std::fill` (nunca `resize`, para no invalidar punteros).

**`update_state(state_msg)`** (`:624-687`): `deg2rad()` para posiciones y
velocidades; pares tal cual; E/S; los 10 campos de supervisión. Deriva
`wait_for_ack` de `safety_status == SafetyStatus::WAIT_FOR_RESTART`.
Devuelve `false` (→ `read` devuelve `ERROR`) si faltan posiciones o velocidades.

**Rango de índices**: muchos bucles usan `i < num_joints_ && i < 6`, es decir
están acotados a 6 articulaciones porque el protocolo del robot solo tiene 6
ejes (`cmd_msg.command_reference[i]`, `hw_joint_positions_[i]`).

### 3.9 `MockRobotServer` y herramientas

- `mock_robot_server.{hpp,cpp}`: servidor UDP que simula un robot CS9 para las
  pruebas.
- `src/test_receive_robot_state.cpp`: ejecutable instalado en
  `lib/staubli_robot_driver` que recibe e imprime el estado del robot. **Usa los
  puertos por defecto 8080/8081**, no los 11000/11001 del Xacro.
- `scripts/upload_val3_server.py` — transfiere la aplicación VAL3 al robot vía FTP.
- `scripts/download_logs.py` / `show_logs.py` — descarga y muestra logs.
- `scripts/remove_val3_server.py` — desinstala la aplicación.
- `staubli_robot_driver/staubli_ftp.py`, `staubli_ftp_utils.py` — cliente FTP y
  utilidades.

---

## 4. Aplicación VAL3

Ruta: `staubli_robot_driver/val3/` (contiene `COLCON_IGNORE`).

### 4.1 Estructura

```
val3/
├── COLCON_IGNORE
├── README.md
├── configs/sio.cfx
└── userapp/ros2_server/
    ├── ros2_server.pjx / .dtx      proyecto
    ├── start.pgx / stop.pgx         arranque/parada
    ├── controlTask.pgx              tarea de control
    ├── updateState.pgx              actualización de estado
    ├── Comm/
    │   ├── computeSequenceDelay.pgx
    │   ├── getNextSequenceNumber.pgx
    │   ├── readData.pgx / readDataOnce.pgx
    │   ├── resetControlSocket.pgx
    │   └── sendData.pgx
    ├── Control/
    │   ├── stopJointPositionControl.pgx
    │   ├── stopJointVelocityControl.pgx
    │   ├── updateJointPositionControl.pgx
    │   └── updateJointVelocityControl.pgx
    ├── Serialization/
    │   ├── decodeHeader.pgx
    │   ├── decodeRobotCommandMsg.pgx
    │   ├── encodeHeader.pgx
    │   ├── encodeRobotStateMsg.pgx
    │   ├── initHeader.pgx
    │   ├── initRobotCommandMsg.pgx
    │   └── initRobotStateMsg.pgx
    ├── Types/{MessageHeader,RobotCommand,RobotCommandMsg,RobotState,RobotStateMsg,ThrottleLogMsg}/
    └── Utils/logThrottleMsg.pgx
```

VAL3 es un lenguaje propio de Staubli; `.pgx` son programas, `.dtx`/`pjx`
contenedores de tipos/proyectos. El contenido completo de cada `.pgx` **no** se
ha transcrito en este documento (son ~40 ficheros); se documenta la lógica
relevante para el contrato con el host.

### 4.2 `encodeHeader.pgx` — BUG CONFIRMADO

Líneas 44-62 del código CDATA:

```
toBinary(x_header.nMagicNumber,       2, "2l", x_nData[0])  # offset 0
toBinary(x_header.nProtocolVersion,   1, "1",  x_nData[2])  # offset 2
toBinary(x_header.nMsgType,           1, "1",  x_nData[3])  # offset 3
toBinary(x_header.nMsgSequenceNumber, 2, "2l", x_nData[4])  # offset 4
toBinary(x_header.nMsgSequenceNumber, 2, "2l", x_nData[6])  # offset 6  <-- ERROR
```

El campo del offset 6 es `payload_size` en el protocolo C++
(`protocol.hpp:70`). `decodeHeader.pgx:50-51` decodifica `nPayloadSize`
correctamente. Detalle completo en §11.3.

### 4.3 `decodeHeader.pgx` — validaciones

- Offset final debe coincidir con `nHeaderSize`.
- `nMagicNumber != 43981` (0xABCD) → error.
- `nProtocolVersion != 1` → error.
- Sobre `nPayloadSize`: *"We don't really need this one, but let's decode it
  anyway"*.

---

## 5. `staubli_bringup`

**Tipo**: `ament_cmake` · **Versión**: 0.1.4

### 5.1 `config/controllers.yaml`

```yaml
controller_manager:
  ros__parameters:
    update_rate: 250                                   # Hz
    joint_state_broadcaster:      {type: .../JointStateBroadcaster}
    joint_trajectory_controller:  {type: .../JointTrajectoryController}
    forward_velocity_controller:  {type: .../ForwardCommandController}

joint_trajectory_controller:
  ros__parameters:
    joints: [joint_1 ... joint_6]
    command_interfaces: [position]
    state_interfaces: [position, velocity]
    action_monitor_rate: 20.0
    allow_nonzero_velocity_at_trajectory_end: true
    allow_partial_joints_goal: false
    interpolate_from_desired_state: true
    set_last_command_interface_value_as_state_on_activation: false
    constraints: {stopped_velocity_tolerance: 0.01, goal_time: 0.0}

forward_velocity_controller:
  ros__parameters:
    joints: [joint_1 ... joint_6]
    interface_name: velocity
```

Comentario clave (`:37-40`): al reactivar JTC tras el control manual (FVC), **no**
se toma el último comando de posición (que quedaría en HOME) sino el estado
medido real, para mantener la pose manual actual.

`allow_partial_joints_goal: false` es relevante: `joint_trajectory_controller`
rechaza goals con subconjunto de articulaciones.

### 5.2 `launch/launch_gazebo.launch.py` (269 líneas)

Orden de las acciones (`:255-268`): `log_info`, `robot_state_publisher_node`,
`gz_server`, `clock_bridge`, `camera_image_bridge`, `spawn_robot`,
`load_joint_state_broadcaster`, `load_joint_trajectory_controller`,
`load_forward_velocity_controller`, `gz_gui`.

| Pieza | Detalle |
|---|---|
| `robot_description` | `Command` que ejecuta `xacro staubli.urdf.xacro robot_model:=<arg> use_mock_hardware:=false use_gazebo_hardware:=true gazebo_controllers_file:=$CTRL_YAML` (`:90-102`) |
| `robot_state_publisher` | Con `use_sim_time` parametrizable (`:110-118`) |
| `gz_server` | `GzServer(world_sdf_file=inspection_cell.sdf, create_own_container=False, use_composition=False)` (`:134-138`) |
| `spawn_robot` | `ros_gz_sim create -topic /robot_description -name staubli -z 0.30 -allow_renaming true` (`:149-159`). El `-z 0.30` coloca el robot sobre la caja `robot_platform` de 0.30 m |
| `clock_bridge` | `parameter_bridge "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"` (`:167-172`) |
| `camera_image_bridge` | `parameter_bridge "/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image"`, nombre `camera_image_bridge` (`:179-185`) |
| `spawner` JSB | `joint_state_broadcaster -c /controller_manager --controller-manager-timeout 30` (`:196-205`) |
| `spawner` JTC | `joint_trajectory_controller ... --param-file controllers.yaml` (`:207-217`) |
| `spawner` FVC | `forward_velocity_controller ... --param-file controllers.yaml --inactive` (`:219-230`) |
| `gz_gui` | `ExecuteProcess(["gz","sim","-g"])`, condicionado a `gui:=true` (`:234-238`) |

Los 3 spawners usan `--controller-manager-timeout 30` porque el plugin
`gz_ros2_control` necesita el `robot_description` para inicializar el
`ResourceManager` antes de que el CM responda (comentario `:188-195`).

**Nota**: `controllers.yaml` se pasa dos veces con propósitos distintos: como
`<parameters>` del plugin SDF (donde el CM lo carga) y como `--param-file` del
spawner.

### 5.3 `launch/launch_gazebo_gui.launch.py` (64 líneas)

Solo declara `gui` (default `true`) e incluye:
- `launch_gazebo.launch.py` con `robot_model: "tx2_60l"` (**hardcodeado**, no
  propagado como arg) y `gui: <arg>`.
- `staubli_gui/gui.launch.py` sin argumentos (que a su vez incluye
  `joystick_servo.launch.py` con `use_sim_time: "true"`).

### 5.4 `launch/launch_robot_control.launch.py` (228 líneas) — camino upstream

| Arg | Default |
|---|---|
| `robot_model` | (sin default) |
| `use_mock_hardware` | `true` |
| `robot_ip` | `192.168.0.254` |
| `use_sim_time` | `false` |
| `start_controller` | `none` (choices: `none`, `joint_trajectory_controller`) |
| `gui` | `true` |

Flujo: `robot_state_publisher` (con `use_mock_hardware` y `robot_ip`) +
`controller_manager/ros2_control_node` (con `controllers.yaml`,
`use_sim_time: False` **hardcodeado** en `:144`) + `rviz2` + spawner de
`joint_state_broadcaster` (activo) + spawner de `joint_trajectory_controller`
con `--inactive`, que se activa después mediante
`OnProcessExit` + `ros2 control set_controller_state <ctrl> active` si
`start_controller` coincide (`:163-204`).

No carga `forward_velocity_controller` ni la GUI. No usa Gazebo.

### 5.5 `launch/launch_moveit.launch.py` (110 líneas)

Secuencia: `wait_for_robot_ready` → `OnProcessExit` → `include staubli_moveit.launch.py`.

Args: `robot_model` (sin default), `use_sim_time=false`,
`warehouse_sqlite_path=~/.ros/warehouse_ros.sqlite`, `launch_servo=false`, `gui=true`.
Mapea `gui` → `launch_rviz`.

### 5.6 `launch/launch_demo.launch.py` (91 líneas) — upstream

`launch_robot_control` (con `use_sim_time: "false"`, `start_controller:
joint_trajectory_controller`, `gui: "false"`) + `launch_moveit` (con `gui:
"true"`). `robot_model` default `tx2_60l`. Es el comando que usa
`.docker/compose.yaml`.

### 5.7 `scripts/wait_for_robot_ready.py`

Adaptado de `ur_robot_driver/scripts/wait_for_robot_description.py` (FZI, 2024).

- Suscribe a `robot_description` (`std_msgs/String`) con QoS
  `depth=1, TRANSIENT_LOCAL` (necesario porque `robot_state_publisher` publica
  el URDF "latched"; sin ello, un suscriptor tardío no lo recibe).
- Suscribe a `joint_states` con `depth=1, VOLATILE` (correcto para un flujo
  continuo; usar `TRANSIENT_LOCAL` sería incompatible con el broadcaster).
- Usa dos `Future` + un `Future` maestro `robot_is_ready_future`; log de espera
  cada 5 s con checkmarks ✓/✗.
- Sale con `spin_until_future_complete`.
- Los topics son **parámetros** (`description_topic`, `joint_states_topic`).

### 5.8 `worlds/inspection_cell.sdf` (272 líneas)

| Elemento | Detalle |
|---|---|
| `<world name="inspection_cell">` | |
| `<physics name="1ms" type="ignored">` | Paso fijo de 1 ms |
| Plugins | 5 sistemas (`:24-44`): `Physics`, `UserCommands`, `SceneBroadcaster`, `Contact`, `Sensors` (con `render_engine: ogre2`) |
| Luz | `directional` `sun` |
| `ground_plane` | Suelo |
| `inspection_table` | Mesa de inspección, tablero a `z=0.50`, patas de `0.450` (de `z=0` a `z=0.45`) |
| `robot_platform` | Caja de **0.30 m** de altura (destino del spawn `-z 0.30`) |
| `part_1` | Pieza a inspeccionar |

**El robot NO está en el fichero**: se inyecta desde `/robot_description`
(comentario `:16`). La celda de la GUI en `points.yaml:44` sitúa el centro de la
pieza en world `(0.0, 0.66, 0.54)`, que al restar el offset world→base_link
`(0, 0, 0.30)` da `part_center_base_link: [0.0, 0.66, 0.24]`.

### 5.9 `rviz/default.rviz` y `plotjuggler/`

- `rviz/default.rviz` — configuración para `launch_robot_control`.
- `plotjuggler/joint_trajectory_control.xml` — curva de estado del JTC. **No hay
  ningún launch ni script del repositorio que invoque PlotJuggler**: es un
  recurso preparado para uso manual. DESCONOCIDO si se usa en la práctica.

### 5.10 `test/`

Solo `test/.gitkeep`. **Sin pruebas** en `staubli_bringup`.

---

## 6. `staubli_moveit_config`

**Tipo**: `ament_cmake` · **Versión**: 0.1.4

### 6.1 Contenido

| Fichero | Contenido |
|---|---|
| `srdf/staubli.srdf.xacro` | Wrapper que carga `staubli_macro.srdf.xacro` |
| `srdf/staubli_macro.srdf.xacro` | Definición de grupos, frames, end-effector, colisiones |
| `config/kinematics.yaml` | `arm`: `kdl_kinematics_plugin/KDLKinematicsPlugin`, `kinematics_solver_search_resolution: 0.005`, `kinematics_solver_timeout: 0.5` |
| `config/joint_limits.yaml` | `default_velocity_scaling_factor: 0.1`, `default_acceleration_scaling_factor: 0.1`; por articulación `max_velocity` 1.5 rad/s (2.0 en `joint_6`), `max_acceleration` 5.0 (10.0 en `joint_6`) |
| `config/moveit_controllers.yaml` | `moveit_simple_controller_manager/MoveItSimpleControllerManager`; `joint_trajectory_controller` con `action_ns: follow_joint_trajectory`, `type: FollowJointTrajectory`, **`default: false`**, las 6 articulaciones |
| `config/pilz_cartesian_limits.yaml` | `max_trans_vel: 1.0`, `max_trans_acc: 2.25`, `max_trans_dec: -5.0`, `max_rot_vel: 1.57` |
| `config/staubli_servo.yaml` | Configuración de `moveit_servo` (ver §6.2) |
| `config/moveit.rviz` | Config de RViz para MoveIt |
| `launch/staubli_moveit.launch.py` | `move_group` + `servo_node` (condicional) + `rviz2` |
| `test/test_default_srdf.py` | 72 líneas, valida el SRDF por defecto |
| `.setup_assistant` | Marcas del Setup Assistant |

### 6.2 `config/staubli_servo.yaml` — parámetros

| Clave | Valor | Nota |
|---|---|---|
| `use_gazebo` | `false` | |
| `command_in_type` | `speed_units` | m/s y rad/s |
| `system_latency_compensation` | `0.04` s | |
| `publish_period` | `0.004` s | 250 Hz |
| `low_latency_mode` | `false` | |
| `command_out_type` | `std_msgs/Float64MultiArray` | |
| `publish_joint_positions` | `true` | Implica control de **posición** |
| `publish_joint_velocities/accelerations` | `false` | |
| `smoothing_filter_plugin_name` | `online_signal_smoothing::ButterworthFilterPlugin` | |
| `move_group_name` | `staubli_arm` | ⚠️ el nodo real se llama `move_group` (`:45`) |
| `planning_frame` | `base_link` | |
| `ee_frame` | `tool0` | |
| `robot_link_command_frame` | `tool0` | |
| `incoming_command_timeout` | `0.1` s | |
| `num_outgoing_halt_msgs_to_publish` | `4` | |
| `lower_singularity_threshold` | `100.0` | |
| `hard_stop_singularity_threshold` | `200.0` | |
| `joint_limit_margins` | `[0.1]×6` | rad |
| `cartesian_command_in_topic` | `~/delta_twist_cmds` | |
| `joint_command_in_topic` | `~/delta_joint_cmds` | |
| `joint_topic` | `/joint_states` | |
| `status_topic` | `~/status` | |
| **`command_out_topic`** | **`/forward_position_controller/commands`** | ⚠️ ver §11.2 (`:70`) |
| `check_collisions` | `true` | |
| `collision_check_rate` | `5.0` Hz | |
| `collision_check_type` | `threshold_distance` | |
| `self_collision_proximity_threshold` | `0.01` m | |
| `scene_collision_proximity_threshold` | `0.02` m | |

### 6.3 `launch/staubli_moveit.launch.py` (185 líneas)

Usa `MoveItConfigsBuilder(robot_name="staubli", package_name="staubli_moveit_config")`
con `.robot_description_semantic(Path("srdf")/"staubli.srdf.xacro", {prefix:"", robot_model})`
y `.planning_scene_monitor(publish_robot_description_semantic=...)`.

Nodos:
- `move_group` (`moveit_ros_move_group/move_group`) con
  `warehouse_ros_config` = `warehouse_ros_sqlite::DatabaseConnection`.
- `servo_node` (`moveit_servo/servo_node`) condicionado a `launch_servo:=true`,
  con `servo_params = {"moveit_servo": staubli_servo.yaml}`.
- `rviz2` (nombre `rviz2_moveit`) condicionado a `launch_rviz:=true`, con
  `config/moveit.rviz`.

**Nota** (`:179-180`): el comentario dice *"Make sure that robot state_publisher
starts before this launch file. See bringup package launch file"* — no hay
dependencia de orden real; `launch_moveit.launch.py` en bringup lo resuelve con
`wait_for_robot_ready`.

**Dependency gap**: `package.xml` **no declara** `moveit_servo` ni
`control_msgs`, aunque `staubli_moveit.launch.py:146` lanza `moveit_servo/servo_node`.

---

## 7. `staubli_trajectory`

**Tipo**: `ament_python` · **Versión**: 0.0.1

### 7.1 `config/points.yaml` (84 líneas)

Estructura:

```yaml
joint_names: [joint_1 ... joint_6]
point_type: cartesian       # 'cartesian' (IK) | 'joint' (diagnóstico, positions en rad)
planning_frame: base_link
orientation_mode: look_at_part_center   # 'manual' | 'look_at_part_center'
part_center_base_link: [0.0, 0.66, 0.24]
```

Con `look_at_part_center`, el eje **+Z de `tool0`** se calcula para apuntar al
centro de la pieza; un punto concreto puede sobreescribirlo añadiendo
`quaternion_xyzw`. Con `manual`, **cada punto debe** fijar `quaternion_xyzw`.

### 7.2 `staubli_trajectory/tx2_60l_ik.py` (345 líneas) — cinemática

| Elemento | Detalle |
|---|---|
| `_RobotYamlLoader` | `yaml.SafeLoader` extendido con el tag `!degrees` (`:44-53`) |
| `find_tx2_60l_config_dir()` | Localiza `config/tx2_60l/` (`:60`) |
| `load_tx2_60l_geometry()` | Lee `kinematics.yaml` + `joint_limits.yaml` (`:89`) |
| `JOINT_VELOCITY_LIMITS_RAD` | Límites de velocidad en rad/s (`:139`), reexportado |
| `_IK_POS_EPS` / `_IK_ROT_EPS` | Tolerancias 1e-4 m / 1e-3 rad (`:142-143`) |
| `IkResult` | Resultado de IK con flags de validez (`:148`) |
| `Tx260lKinematics.__init__(random_seed=42, n_random_seeds=30)` | |
| `.forward_kinematics(q) -> kdl.Frame` | `:195` |
| `.jacobian(q) -> np.ndarray` | Jacobiano 6×6 de `tool0` en `base_link` (`:205`) |
| `.inverse_kinematics(...)` | IK con multi-semilla aleatoria (`:224`) |
| `._check_solution(q, goal) -> IkResult` | Verificación por FK + tolerancias (`:284`) |
| `.check_joint_limits(q)` | Devuelve `(ok, lista_por_articulación)` (`:296`) |
| `.orientation_frame_looking_at(...)` | Quaternion para orientar `tool0` hacia la pieza (`:305`) |
| `.frame_from_position_quaternion(...)` | Construye `kdl.Frame` (`:334`) |
| `.quaternion_from_frame(frame)` | Inversa (`:343`) |

**Clave arquitectónica**: `staubli_joystick_servo` importa `Tx260lKinematics` y
`JOINT_VELOCITY_LIMITS_RAD` de este módulo, de modo que la geometría del robot
**no está duplicada** (comentario explícito en `servo_node.py:21-22`).

### 7.3 `staubli_trajectory/points_config.py` (148 líneas)

`PointsConfig(config_path)`: carga y valida el YAML. Contiene
`joint_names`, `point_type`, `planning_frame`, `orientation_mode`,
`part_center_base_link`, la lista de puntos y los parámetros temporales y de
tolerancia. Método `_validate()` (`:100`) comprueba coherencia (p.ej. que
`orientation_mode: manual` implica `quaternion_xyzw` en cada punto).

### 7.4 `staubli_trajectory/trajectory_builder.py` (189 líneas)

| Clase | Campos |
|---|---|
| `WaypointRecord` | Punto resuelto: q[6], goal cartesiano, `ik_ok`, flags de límites, tiempo |
| `TrajectoryPlan` | Lista de `WaypointRecord` + `JointTrajectory` |
| `TrajectoryBuilder(config, kinematics=None)` | |
| `.build() -> TrajectoryPlan` | `:69` — orquesta |
| `._solve_point(...)` | `:94` — orientación + IK + verificación de límites |
| `._build_joint_mode()` | `:154` — para `point_type: joint`, sin IK |
| `._to_joint_trajectory(records)` | `:177` — convierte a `trajectory_msgs/JointTrajectory` con timestamps |

### 7.5 `staubli_trajectory/joint_trajectory_client.py` (148 líneas)

`JointTrajectoryClient(node)`:
- `.wait_for_server(timeout_s=10.0)` (`:54`) — espera el servidor de la acción.
- `.send(trajectory, config) -> bool` (`:66`) — construye el goal y lo envía.
- `._build_goal(trajectory, config)` (`:117`) — `FollowJointTrajectory.Goal` con
  la trayectoria, una `JointTolerance` por articulación tomada de
  `config.goal_position_tolerance` / `goal_velocity_tolerance` /
  `goal_acceleration_tolerance`, y `goal_time_tolerance` como
  `builtin_interfaces/Duration`.
- `._on_feedback(feedback_msg)` (`:141`) — imprime el índice del waypoint
  alcanzado y las posiciones articulares reales.
- El topic de la acción es
  `/joint_trajectory_controller/follow_joint_trajectory`.

### 7.6 `staubli_trajectory/trajectory_node.py` (176 líneas)

Nodo `staubli_trajectory_node`. Argumentos de línea de comandos: `--config` (ruta
al YAML) y `--send` (boolean). **Por defecto solo calcula e imprime el plan**;
`--send` habilita la ejecución vía `JointTrajectoryClient`.

### 7.7 `setup.py`

`console_scripts`: `trajectory_node = staubli_trajectory.trajectory_node:main`.
`data_files` instala `config/*.yaml` → `share/<pkg>/config` y
`launch/*.launch.py` → `share/<pkg>/launch`.

### 7.8 Pruebas

**No existe directorio `test/`.** `staubli_trajectory` es el único paquete Python
sin pruebas, pese a tener la lógica más compleja (IK, builder, ActionClient).
`package.xml:26-29` declara `test_depend` de pytest que nunca se usa.

---

## 8. `staubli_joystick_servo`

**Tipo**: `ament_python` · **Versión**: 0.0.1

### 8.1 `staubli_joystick_servo/velocity_limiter.py` (175 líneas)

Funciones puras (testeadas sin ROS):

| Función | Responsabilidad |
|---|---|
| `apply_deadzone(lin, ang, dz_lin, dz_ang)` | Anula componentes por debajo del umbral |
| `clamp_vector(vec, max_abs)` | Recorta por norma máxima |
| `solve_qdot_dls(J, twist, lambda, sigma_thr)` | Resuelve `q̇ = Jᵀ(JJᵀ + λ²I)⁻¹ v` con SVD; devuelve `(q_dot, sigma_min, U, S)` |
| `scale_to_joint_velocity_limits(q_dot, limits)` | Escala proporcional si se supera algún límite |
| `class JointVelocityLimiter` | Rampa de aceleración/desaceleración; `update(target)`, `force_zero()`, `reset()` |

### 8.2 `staubli_joystick_servo/servo_node.py` (201 líneas)

Nodo `staubli_joystick_servo`.

**Interfaces**:
- Suscribe `geometry_msgs/Twist` en `/cmd_vel`, QoS depth 10.
- Suscribe `sensor_msgs/JointState` en `/joint_states`, QoS depth 10.
- Publica `std_msgs/Float64MultiArray` en `/forward_velocity_controller/commands`,
  QoS depth 10.
- Timer a `1/control_rate` (250 Hz por defecto).

**Bucle `_on_timer()`** (`:136-176`), en orden:
1. **Guarda de estado**: si no hay `/joint_states` o tienen más de
   `joint_state_timeout` (0.2 s), `limiter.reset()` + publicar ceros + `return`.
2. **Watchdog de comando**: si no hay `/cmd_vel` o tiene más de
   `command_timeout` (0.2 s), publicar `limiter.force_zero()` + `return`.
3. **Deadzone** por eje (0.01 lineal y angular).
4. Si el Twist es nulo tras el deadzone → `target = zeros(6)` (frenado progresivo
   vía el limitador).
5. Si no: `clamp_vector` a `max_linear_speed` (0.15 m/s) y
   `max_angular_speed` (0.5 rad/s), componer `[lin(3), ang(3)]`.
6. `jac = kinematics.jacobian(self._joint_positions)`.
7. `solve_qdot_dls(jac, twist_vec, damping_lambda=0.05, singularity_threshold=0.02)`.
8. `scale_to_joint_velocity_limits(q_dot, JOINT_VELOCITY_LIMITS_RAD)`.
9. `limiter.update(target)` y publicar.

**Seguridad declarada** (docstring `:13-19`): arranca publicando ceros hasta
tener `/joint_states`; watchdogs con publicación inmediata de ceros; reducción
del Twist y mayor amortiguamiento cerca de singularidad; los límites articulares
impiden velocidades explotadas.

`stop()` (`:182-183`) publica `force_zero()`; se invoca en el `finally` de
`main()`.

### 8.3 `config/servo_params.yaml` (29 líneas)

Valores documentados como *"CONSERVADORES para simulación"*. Todos los
parámetros coinciden con los `declare_parameter` de `servo_node.py`.
`control_rate: 250.0` con el comentario de que coincide con el `update_rate` del
`controller_manager`.

### 8.4 `launch/joystick_servo.launch.py` (56 líneas)

Solo `use_sim_time` (default `true`). Lanza `servo_node` con `servo_params.yaml`
+ `{"use_sim_time": <arg>}`.

### 8.5 Pruebas

`test/test_velocity_limiter.py` (248 líneas) + `test/conftest.py` (20 líneas).
Cubre las funciones puras y `JointVelocityLimiter` sin necesidad de ROS.

---

## 9. `staubli_gui`

**Tipo**: `ament_python` · **Versión**: 0.0.1 · **GUI**: PySide6

### 9.1 `staubli_gui/gui.py` (2101 líneas) — aplicación principal

**Estructura**:
- Docstring de 60 líneas que documenta las 5 fases de la GUI.
- Funciones de estilo: `_stylesheet()`, `_btn_style()`, `_label_style()`,
  `_make_btn()`, `_make_box()`, `_separator()`.
- `class CameraView(QLabel)` (`:312`) — visor de imagen con
  `set_frame(QImage)`, `resizeEvent` y `_render()` que escala manteniendo
  relación de aspecto.
- `class MainWindow(QMainWindow)` (`:353`) — todo el comportamiento.
- `main()` (`:2079`).

**Constantes de topics**:
- `_HOME_ACTION = "/joint_trajectory_controller/follow_joint_trajectory"` (`:163`)
- `_CMD_VEL_TOPIC = "/cmd_vel"` (`:194`)
- `_CAMERA_TOPIC = "/camera/image_raw"`, `_CAMERA_MSG = Image` (`:150-151`)

**`_setup_ros()`** (`:757`) crea:
- Suscripción a `/joint_states`.
- Suscripción a `/camera/image_raw` con QoS `sensor_data` (BEST_EFFORT, VOLATILE),
  compatible con el bridge. Comentario `:147-149` explica el criterio.
- `ActionClient` de `FollowJointTrajectory` (`:798`).
- Publicador de `/cmd_vel` (`:807`), **solo activo con control manual**.
- Clientes de `ListControllers` y `SwitchController`.
- `QTimer` que llama `rclpy.spin_once(...)` (`:815`).

**Cámara** — `_on_camera_image(msg)` (`:870-903`):
Convierte `sensor_msgs/Image` en `QImage` con `QImage.Format_RGB888`
**usando directamente el buffer de la mensagem**, sin OpenCV, sin `cv_bridge` y
sin numpy (comentario `:36-40`). Si `msg.encoding != "rgb8"`, muestra un aviso
en el visor. Llama a `camera_view.set_frame(image)`.

**`_publish_manual_twist()`** (`:842-861`): construye
`geometry_msgs/Twist` a partir del panel `FrameControlPanel` y lo publica **solo
si `self._manual_active`**.

**Máquina de estados de control** — el corazón de la lógica:

| Modo | JTC | FVC | Descripción |
|---|---|---|---|
| `trayectoria` | activo | inactivo | Goals por acción `FollowJointTrajectory` |
| `manual` | inactivo | activo | Control FRAME publicando `/cmd_vel` |
| `seguro` | inactivo | inactivo | Solo `joint_state_broadcaster` |
| `conflicto` | activo | activo | Estado prohibido (se detecta y se reporta) |
| `desconocido` | — | — | Sin información fiable |

Derivada en `controller_manager.py:181-203` (`derive_mode(states)`).
El plan de cambio en `controller_manager.py:206-227` (`switch_plan(mode)`)
garantiza la exclusividad e incluye siempre `joint_state_broadcaster` en
`activate`.

**Gestión de controladores** — `staubli_gui/controller_manager.py` (227 líneas):
- Módulo **independiente de Qt** (solo `rclpy`), pensado para ser testeable sin
  servidor ROS.
- `ControllerManager(node, switch_timeout_s=5.0)`.
- `.refresh(done_callback)` — `ListControllers` asíncrono → `dict nombre → estado`.
- `.switch(activate=(), deactivate=(), done_callback)` — flujo de 3 pasos:
  1. `refresh()` para conocer el estado real.
  2. Filtrar: activar solo los que **no** están `active`, desactivar solo los que
     **están** `active`. Esto es necesario porque **`STRICT` no es idempotente**
     (docstring `:12-14`): pedir activar algo ya activo hace fallar la llamada.
  3. Una sola llamada `SwitchController` con `strictness=STRICT`,
     `activate_asap=False`, `timeout=Duration(sec=5)`.
  4. Verificación posterior con `list_controllers`: confirma que los activados
     están `active` y los desactivados no.
- Si no hay nada que cambiar, responde `ok=True` con el mensaje *"los
  controladores ya estaban en el estado deseado"*.

**`_switch_to_mode(mode)`** (`:984`) y `_on_mode_switch_done()` (`:1030`):
orquestan el cambio y actualizan el indicador de modo. Hay: `_pause_status_poll()` /
`_resume_status_poll()` (`:1897`, `:1902`) para no consultar el CM durante un goal.

**Puntos y trayectorias** (`:1161`–`:1960`):
- `_on_punto(name)` — ir a un punto predefinido (P1/P2/P3 de `points.yaml`).
- `_on_capture_position()` — capturar la pose articular actual.
- `_on_new_point()`, `_on_edit_point()`, `_on_delete_point()` — CRUD de puntos
  de usuario.
- `_load_user_points()` / `_save_user_points()` — persistencia.
- `_refresh_trajectory_list()`, `_on_traj_move_up/down()`, `_on_traj_remove()`,
  `_on_traj_clear()`, `_on_add_to_trajectory()` — gestión de la lista ordenada
  de waypoints.
- `_build_single_goal()`, `_build_trajectory_goal()`,
  `_build_user_trajectory_goal()` — construcción de los tres tipos de goal.
- `_fill_goal_tolerances(goal)` — tolerancias.
- `_on_motion_goal_response()`, `_on_motion_feedback()` (`:1798`),
  `_on_motion_result()` (`:1834`), `_finish_motion()` (`:1884`) — gestión del
  goal y **retroalimentación visual** (resaltado de waypoints completados, commit
  `6cd1471`).
- `_on_run()` (`:1964`), `_on_stop()` (`:1998`), `_on_cancel_response()` (`:2030`)
  — ejecución, parada y cancelación.

**Reutilización de `staubli_trajectory`**: la GUI **no** reimplementa P1/P2/P3 ni
la cinemática. Usa importaciones perezosas (dentro de los métodos, no en el
import de módulo):
- `gui.py:1238` y `gui.py:1534` → `from staubli_trajectory.tx2_60l_ik import Tx260lKinematics`
- `gui.py:1919-1920` → `from staubli_trajectory.points_config import PointsConfig` y
  `from staubli_trajectory.trajectory_builder import TrajectoryBuilder`
- `gui.py:1924-1925` → `PointsConfig(str(config_path))` + `TrajectoryBuilder(config).build()`
- `gui.py:17` lo declara: *"TrajectoryBuilder); no se duplican coordenadas ni IK aquí"*.
- `gui.py:203`: los puntos de usuario almacenan *"posiciones articulares ya
  resueltas por TrajectoryBuilder (sin IK propia)"*.

`rpy_degrees_to_quaternion_xyzw` **no** viene de `staubli_trajectory`: se importa
desde el módulo propio `.user_points` (`gui.py:100-105`).

### 9.2 `staubli_gui/frame_control.py` (535 líneas)

`FrameControlPanel` (`gui.py:99`): widget Qt con **seis pares de botones ±**
(X, Y, Z, RX, RY, RZ) que generan un `Twist` cartesiano **normalizado** en
`base_link`. Según el docstring de `gui.py:43-50`, al desactivar el control
manual ("VOLVER A TRAYECTORIA") la GUI envía un `Twist=0` y restaura el
`joint_trajectory_controller`. **La GUI nunca calcula IK ni velocidades
articulares**: eso lo hace `staubli_joystick_servo`.

### 9.3 `staubli_gui/user_points.py` (248 líneas)

| Elemento | Detalle |
|---|---|
| `default_config_dir()` | `$XDG_CONFIG_HOME/staubli_gui` o `~/.config/staubli_gui` (`:42-51`) |
| `default_config_path()` | `<dir>/user_points.yaml` (`:52`) |
| `rpy_degrees_to_quaternion_xyzw(...)` | RPY en grados → quaternion `xyzw` (`:60`) |
| `user_point_from_joint_position(...)` | Crea un `UserPoint` desde posiciones articulares (`:137`) |
| `class UserPointStore` (`:173`) | `load()`, `save()`, `names()`, `all()`, `get()`, `has()`, `add()`, `update()`, `remove()` |

### 9.4 `launch/gui.launch.py` (42 líneas)

Incluye `joystick_servo.launch.py` con `use_sim_time: "true"` y lanza
`gui_node` (nombre `staubli_gui`, `output="screen"`).

### 9.5 `setup.py` / `setup.cfg`

`console_scripts`: `gui_node = staubli_gui.gui:main`.

Dependencias Python de GUI: `python3-pyside6.qtcore`, `python3-pyside6.qtgui`,
`python3-pyside6.qtwidgets`, `ament_index_python`. ROS:
`rclpy`, `action_msgs`, `sensor_msgs`, `geometry_msgs`, `controller_manager_msgs`,
`control_msgs`, `trajectory_msgs`.

### 9.6 Pruebas

| Fichero | Líneas | Qué cubre |
|---|---|---|
| `conftest.py` | 39 | Fixtures compartidas |
| `test_controller_manager.py` | 89 | `derive_mode`, `switch_plan` |
| `test_capture_position.py` | 166 | Captura de pose articular |
| `test_frame_control.py` | 239 | `FrameControlPanel` |
| `test_trajectory_feedback.py` | 355 | Retroalimentación de waypoints |
| `test_user_trajectory.py` | 176 | Trayectorias de usuario |

---

## 10. Metapaquete, CI, Docker y documentación

### 10.1 `staubli_driver_ros2` (metapaquete)

- `package.xml` (25 líneas): `ament_cmake`, v0.1.4, `build_depend` de
  `python3-sphinx` y `doxygen`, `exec_depend` de los 4 paquetes (ver §4.1 de
  OVERVIEW).
- `CMakeLists.txt` (33 líneas): `project(... NONE)`, opción `BUILD_DOCS` (OFF).
  Si se activa: `pip install -r requirements.txt` (con `--user` salvo que haya
  `VIRTUAL_ENV`/`CONDA_PREFIX`), target `Sphinx ALL` con
  `sphinx-build -b html sphinx sphinx/_build`.
- `requirements.txt` (408 bytes), `sphinx/`, `doxygen/`, `README.md` presentes.

### 10.2 GitHub Actions

| Workflow | Disparador | Contenido |
|---|---|---|
| `ci.yml` | push a `main`, PR, `workflow_dispatch`, cron semanal (domingo 00:00) | Contenedor `rostooling/setup-ros-docker:ubuntu-noble-latest`, `pip break-system-packages`, `ros-tooling/action-ros-ci@0.4.8` con `target-ros2-distro: jazzy` |
| `ci-format.yml` | `workflow_dispatch`, PR | Contenedor `ros:jazzy-ros-base`, `git config --global --add safe.directory`, instala `pre-commit` y ejecuta `pre-commit run --all-files` tras `source /opt/ros/jazzy/setup.sh` |
| `docker.yml` | **No inspeccionado en detalle** | DESCONOCIDO (presumible build/publicación de imagen) |
| `documentation.yml` | **No inspeccionado en detalle** | DESCONOCIDO (presumible publicación de Sphinx) |
| `.github/dependabot.yml` | Mantenimiento | Validado por el hook `check-dependabot` |

### 10.3 Docker

`.docker/Dockerfile`, `.docker/compose.yaml`, `.docker/.env`, `.docker/README.md`.

- `.env`: `STAUBLI_ROBOT_IP=192.168.0.254`, `STAUBLI_USE_MOCK_HARDWARE=true`.
- `compose.yaml`: servicio `staubli_driver_ros2_demo`, imagen
  `staubli_driver_ros2:0.1.2`, `network_mode: host`, `ipc: host`,
  `privileged: true`, monta `/tmp/.X11-unix` y `.Xauthority`, y lanza
  `ros2 launch staubli_bringup launch_demo.launch.py robot_ip:=... use_mock_hardware:=...`.

> La imagen es `0.1.2` mientras el metapaquete está en `0.1.4`: desalineación de
> versión.

### 10.4 Parche local: `patches/gz_ros2_control/fix_command_mode_switch.patch`

Corrige tres operadores `&` bitwise donde debía ser `|`, en
`gz_ros2_control/src/gz_system.cpp` (~línea 784), dentro del bloque que limpia
los bits de `joint_control_method` al parar interfaces:

```diff
-  static_cast<ControlMethod_>(VELOCITY & EFFORT);
+  static_cast<ControlMethod_>(VELOCITY | EFFORT);
```

Con `&` bitwise, `VELOCITY & EFFORT == 0`, así que al parar la interfaz de
posición se borraban **todos** los bits de control. El parche **no se aplica
automáticamente**; el workspace local muestra que el usuario lo aplicó a mano.

### 10.5 Documentación existente

- `README.md` raíz (documentación de usuario upstream).
- `staubli_robot_driver/val3/README.md` (instrucciones de despliegue VAL3).
- `staubli_robot_description/meshes/tx2_60/README.md`.
- `staubli_driver_ros2/sphinx/` + `doxygen/` (fuente de la documentación
  publicada en <https://icube-robotics.github.io/staubli_driver_ros2/>).
- `CHANGELOG.md`.

---

## 11. Hallazgos, incoherencias y riesgos

### 11.1 Distribuciones ROS divergentes — **riesgo alto**

| Fuente | Distribución |
|---|---|
| `README.md:7` | jazzy / Ubuntu 24.04 |
| `.github/workflows/ci.yml:21` | `target-ros2-distro: jazzy` |
| `.github/workflows/ci-format.yml:15,24` | `ros:jazzy-ros-base`, `/opt/ros/jazzy/setup.sh` |
| `.docker/*` | (no inspeccionado en detalle) |
| Entorno local | **lyrical / Ubuntu 26.04** |
| Commit `4cac784` | "Adapt driver for **ROS 2 Lyrical**" |

Adaptaciones a Lyrical presentes en el código que no existen en el estado
validado por la CI:
- `hardware_interface::HardwareComponentInterfaceParams` en
  `on_init(const ...& params)` (`staubli_hardware_interface.cpp:59`).
- Uso de `info_.hardware_info` / `info_.hardware_parameters` (`:104`, `:153`).
- `std::copy` / `std::fill` en lugar de `resize` para no invalidar punteros
  (`:609-612`, `:85-96`).
- `src/communication/socket_factory.cpp` y
  `src/communication/udp_socket_impl.hpp` (nuevos en la rama).
- `real_time_socket_interface.hpp:35` (deriva de `RealTimeSocketSubscriber`).

**Conclusión**: la CI de `main` no valida el código de esta rama. Cualquier
cambio en el driver C++ no está cubierto por pruebas automáticas de la
distribución en la que realmente se ejecuta.

### 11.2 `MoveIt Servo` apunta a un controlador inexistente — **riesgo alto para `launch_servo:=true`**

- `staubli_moveit_config/config/staubli_servo.yaml:70`:
  `command_out_topic: /forward_position_controller/commands`.
- `command_out_type: std_msgs/Float64MultiArray` + `publish_joint_positions: true`
  es exactamente la firma de `forward_position_controller`.
- `staubli_bringup/config/controllers.yaml` solo define
  `joint_state_broadcaster`, `joint_trajectory_controller` y
  `forward_velocity_controller`.
- **Cero coincidencias** de `forward_position_controller` fuera de ese YAML
  (verificado por grep en todo el repositorio).

Consecuencia: `moveit_servo/servo_node` publicaría en un topic sin suscriptor, y
ningún nodo movería el robot. El control cartesiano real de la rama es
`staubli_joystick_servo` (por velocidad), que sí está cableado.

### 11.3 Bug confirmado en `encodeHeader.pgx` — **riesgo alto para robot real**

`val3/userapp/ros2_server/Serialization/encodeHeader.pgx`, código CDATA líneas
~44-58:

| Offset | Campo | Valor escrito | Correcto |
|---|---|---|---|
| 0 | magic | `nMagicNumber` | ✅ |
| 2 | protocol | `nProtocolVersion` | ✅ |
| 3 | msg type | `nMsgType` | ✅ |
| 4 | sequence | `nMsgSequenceNumber` | ✅ |
| 6 | **payload size** | **`nMsgSequenceNumber`** | ❌ |

Contraste con el lado C++ (`protocol.hpp:60-80`):

```cpp
struct FrameHeader {
    uint16_t magic_number    = MAGIC_NUMBER;
    uint8_t  protocol_version = PROTOCOL_VERSION;
    uint8_t  message_type     = ...;
    uint16_t sequence_number  = 0;
    uint16_t payload_size     = 0;     // <-- offset 6
    static size_t get_serialized_size() { return 8; }
};
```

Y con `decodeHeader.pgx:50-51`, que sí lee `nPayloadSize`.

El `sequence_number` empieza en 0 y crece; el `payload_size` de
`RobotStateMessage` es un valor fijo distinto de 0. En cuanto ambos difieran,
la deserialización C++ rechaza la trama. **Impacto**: rompe la comunicación con
el robot real. **No afecta a la simulación** (no hay UDP).

### 11.4 Discrepancia de puertos UDP

| Origen | Control | Diagnóstico |
|---|---|---|
| `protocol.hpp:45,50` (defaults C++) | 8080 | 8081 |
| `r2c/staubli.r2c.xacro:50-53` (producción) | 11000 | 11001 |
| `test_receive_robot_state.cpp` (usa los defaults) | 8080 | 8081 |

Vía Xacro mandan los valores del Xacro, así que los defaults C++ quedan como
código muerto en el flujo normal. Pero `RobotDriver::connect(robot_ip, timeout)`
(`robot_driver.cpp:47-52`) usa los defaults, igual que
`test_receive_robot_state`. Riesgo medio: diagnóstico manual sobre el puerto
equivocado.

Además `local_control_port == control_port == 11000`. Para UDP esto funciona
porque los puertos locales pueden coincidir con los remotos, pero es una
configuración que merece revisión.

### 11.5 TCP declarado pero no implementado

- `src/communication/tcp_socket.cpp:18` incluye `"tcp_socket_impl.hpp"` →
  **el fichero no existe**.
- `staubli_robot_driver/CMakeLists.txt:64-77` no compila `tcp_socket.cpp`, así que el build pasa.
- `socket_factory.cpp:31` lanza `std::runtime_error("TCP protocol not
  implemented yet")`.
- `staubli_robot_driver/include/staubli_robot_driver/communication/tcp_socket.hpp` sí declara la interfaz.

Conclusión: soporte TCP **placeholder**, muerto y no compilado. Riesgo bajo
(ningún flujo lo usa) pero confuso para quien lea el árbol.

### 11.6 Diagnósticos a medio implementar

| Ubicación | Estado |
|---|---|
| `robot_driver.cpp:182` | `diag_interface_ready = true;` con la lectura real comentada y `TODO(tpoignonec)` |
| `robot_driver.cpp:220-232` | Comprobación de `robot_controller_type == CS9` **comentada** |
| `robot_driver.cpp:240` | `TODO`: validar versiones de VAL3 soportadas |
| `robot_driver.cpp:393-409` | `read_diagnostic_data()` implementado pero nunca llamado |
| `staubli_hardware_interface.cpp:330-344` | Exportación de **lecturas** de salidas digital/analógica comentada |
| `robot_driver.cpp:345` | `TODO`: validar límites de posición/velocidad antes de enviar |
| `staubli_hardware_interface.cpp:524-525, 534-535` | `TODO`: validar que todas las interfaces se paran/arrancan juntas |
| `staubli_hardware_interface.cpp:371` | `TODO`: retirar la interfaz `acceleration` cuando el r2c soporte límites |

### 11.7 Control de velocidad y par rechazado en hardware real

`staubli_hardware_interface.cpp:551-557`:

```cpp
case CommandType::JOINT_VELOCITY:
    RCLCPP_ERROR(..., "Velocity control is not yet supported, switch rejected");
    return hardware_interface::return_type::ERROR;
case CommandType::JOINT_TORQUE:
    RCLCPP_ERROR(..., "Torque/effort control is not yet supported, switch rejected");
    return hardware_interface::return_type::ERROR;
```

**Consecuencia directa**: `forward_velocity_controller` no puede activarse
contra el robot real. Como `staubli_joystick_servo` publica en
`/forward_velocity_controller/commands`, **el control manual FRAME de esta rama
solo es funcional en Gazebo o mock**, nunca en el robot CS9. Es coherente con
`README.md:13` ("only position control is supported") y con los avisos de
seguridad de `README.md:25-27`.

### 11.8 Otros puntos verificados

| Hallazgo | Evidencia | Impacto |
|---|---|---|
| Enlace roto en README | `README.md:52` → `staubli_robot_driver/README.md` (no existe; el real es `val3/README.md`) | Menor |
| Descripción de GUI obsoleta | `staubli_gui/package.xml:6-8` dice "Fase 1: solo presentación visual (sin conexión ROS)" | Menor |
| Versión de imagen Docker desfasada | `compose.yaml` usa `0.1.2`, metapaquete en `0.1.4` | Menor |
| `move_group_name: staubli_arm` | `staubli_servo.yaml:34` vs nodo `move_group` real | Medio si `launch_servo:=true` |
| `use_sim_time` ignorado en mock | `launch_robot_control.launch.py:144` pone `{"use_sim_time": False}` hardcodeado | Bajo |
| `robot_model` no propagado | `launch_gazebo_gui.launch.py:58` fija `"tx2_60l"` en vez de propagar el arg | Bajo |
| `staubli_trajectory` sin tests | No existe `test/` pese a declarar `test_depend` de pytest | Medio |
| Metapaquete desactualizado | Faltan `staubli_gui`, `staubli_joystick_servo`, `staubli_trajectory` como `exec_depend` | Bajo |
| `output` de la GUI a `screen` | `gui.launch.py` no usa `emulate_tty`; con `rclpy.spin_once` en `QTimer` los prints pueden salir desordenados | Bajo |
| Artefactos en el árbol | `__pycache__/*.cpython-314.pyc` y `.pytest_cache/` presentes | Higiene |
| `plotjuggler/` sin invocar | `joint_trajectory_control.xml` no lo lanza ningún archivo | Informativo |
| `tcp_socket` sin compilar | Ver §11.5 | Bajo |
| `moveit_servo` y `control_msgs` sin declarar | `staubli_moveit_config/package.xml` vs `staubli_moveit.launch.py:146` | Medio |
| `Env-hooks` sin inspeccionar | `staubli_robot_description/env-hooks/staubli_robot_description.sh.in` | DESCONOCIDO |
| `.github/workflows/docker.yml` y `documentation.yml` | No leídos | DESCONOCIDO |
| Contenido de los ~40 `.pgx` de VAL3 | Solo se analizaron `encodeHeader` y `decodeHeader` | Parcialmente DESCONOCIDO |

---

## 12. Inventario de pruebas

| Paquete | Ficheros | Líneas | Framework | Estado |
|---|---|---|---|---|
| `staubli_robot_driver` | 7 `.cpp` | 2 095 | `ament_add_gtest` | Registrados en CMake (`:174-215`) |
| `staubli_robot_description` | 2 `.py` | 100 | pytest | Activos |
| `staubli_moveit_config` | 1 `.py` | 72 | pytest | Activo |
| `staubli_gui` | 5 `.py` + conftest | 1 064 | pytest | Activos |
| `staubli_joystick_servo` | 1 `.py` + conftest | 268 | pytest | Activo |
| `staubli_bringup` | 0 (`.gitkeep`) | 0 | — | **Sin pruebas** |
| `staubli_trajectory` | 0 | 0 | — | **Sin pruebas** |
| `staubli_driver_ros2` | 0 | 0 | — | **Sin pruebas** |

Detalle de los gtests del driver:

| Test | Líneas | Cubre |
|---|---|---|
| `test_primitives_serialization.cpp` | 170 | Serialización de primitivas little-endian |
| `test_message_serialization.cpp` | 336 | `RobotState/Command/DiagnosticData` |
| `test_sockets.cpp` | 369 | `Socket`, `UDPSocket` |
| `test_rt_socket_interface.cpp` | 307 | `RealTimeSocketSubscriber/Interface` |
| `test_robot_driver.cpp` | 425 | `RobotDriver` con mock server |
| `test_mock_robot_server.cpp` | 197 | `MockRobotServer` |
| `test_staubli_hardware_interface.cpp` | 291 | `StaubliHardwareInterface` (con `.hpp` auxiliar) |

**Cobertura notable**: la comunicación C++ tiene buena cobertura de unit tests.
Lo que **no** tiene pruebas: la lógica Python de trayectoria (IK, builder,
cliente de acción), la GUI más allá de sus helpers puros, y los launch files.
No hay `launch_testing` en la práctica, aunque `staubli_robot_driver` lo declara
como `test_depend` (`package.xml:27`) y `test/launch/` solo contiene `.gitkeep`.
