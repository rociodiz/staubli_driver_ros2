# Estructura del paquete `staubli_robot_driver`

Este documento explica, archivo por archivo, cómo está construido el paquete `staubli_robot_driver`.
Está pensado para alguien que está aprendiendo ROS 2 y `ros2_control`: primero se describe qué
problema resuelve el paquete, después cada pieza, y al final cómo todas las piezas se conectan.

## Qué hace este paquete

El paquete conecta un robot **Staubli** (controlador CS9) con ROS 2. Para ello hace dos cosas
a la vez:

1. **En el PC (Linux, este paquete)**: implementa un plugin de `ros2_control` que traduce los
   mensajes de ROS 2 en mensajes de red, y una aplicación auxiliar en Python que transfiere
   ficheros al controlador por FTP.
2. **Dentro del controlador CS9** (`val3/userapp/ros2_server`): implementa la otra mitad del
   protocolo, escrita en lenguaje **VAL3** (el lenguaje propio de Staubli), que habla el robot.

El enlace de datos entre ambas mitades es **UDP** con un formato binario propio, descrito en
`include/staubli_robot_driver/communication/protocol.hpp`.

## Mapa rápido del árbol

```
staubli_robot_driver/
├── package.xml                       Metadatos y dependencias ROS 2
├── CMakeLists.txt                    Compilación, instalación y tests
├── staubli_hardware_interface.xml    Declaración del plugin para ros2_control
├── include/staubli_robot_driver/     Cabeceras públicas (API del paquete)
│   ├── communication/                Formato de mensajes y sockets
│   └── (raíz)                        Driver, interfaz de hardware, mock
├── src/                              Implementaciones C++ y ejecutables auxiliares
├── staubli_robot_driver/             Paquete Python (cliente FTP)
├── scripts/                          Utilidades CLI instalables
├── test/                             Tests unitarios (GoogleTest)
└── val3/                             Aplicación VAL3 que se copia al robot
```

---

## 1. Metadatos y sistema de build

### `package.xml`

Manifiesto del paquete. Declara el nombre (`staubli_robot_driver`), versión `0.1.4` y el tipo
de build `ament_cmake`. La lista de `<depend>` (`rclcpp`, `hardware_interface`, `pluginlib`,
`realtime_tools`, etc.) coincide con los `find_package` de CMake: son las piezas de ROS 2 que
el driver necesita. En `<test_depend>` aparecen `ament_cmake_gtest` y `launch_testing`.

Para un lector novel: este fichero no contiene lógica, pero es el que permite que
`colcon build` y `rosdep` resuelvan las dependencias correctamente.

### `CMakeLists.txt`

Es el guion de compilación. Lo importante para entender el diseño:

- **Dos librerías compartidas**:
  - `staubli_robot_driver` (`CMakeLists.txt:83`): el núcleo de comunicación. Compila
    `LIBRARY_SOURCES` (`CMakeLists.txt:64-80`): protocolo, mensajes, fábrica de sockets,
    socket UDP, suscriptor/editor de sockets y `robot_driver.cpp`.
  - `staubli_robot_driver_hardware_interface` (`CMakeLists.txt:114`): solo
    `staubli_hardware_interface.cpp`, porque ese fichero necesita `hardware_interface` y
    `pluginlib`, mientras que el núcleo de red no.
- **Boost como dependencia de cabeceras** (`CMakeLists.txt:61,110`): el socket UDP usa
  Boost.Asio; con Boost ≥ 1.90 es header-only, por eso se enlaza `Boost::headers`.
- **Instalación** (`CMakeLists.txt:142-172`): headers a `include/`, librerías a `lib/`,
  el paquete Python con `ament_python_install_package`, los scripts a `lib/<paquete>/` y
  **todo `val3/` a `share/<paquete>/`**. Esto último es la clave: los scripts de Python
  localizan la aplicación VAL3 instalada con `get_package_share_directory()`.
- **Tests** (`CMakeLists.txt:174-238`): solo se compilan si `BUILD_TESTING` está activo, y
  solo dentro de `if(BUILD_TESTING)`.
- **Export del plugin** (`CMakeLists.txt:274-277`): registra `staubli_hardware_interface.xml`
  para que `ros2_control` pueda cargarlo en tiempo de ejecución.

### `staubli_hardware_interface.xml`

Ficha de identidad del plugin: dice qué clase implementa el hardware y de qué clase base hereda.

- `library path="staubli_robot_driver_hardware_interface"` → la segunda librería de CMake.
- `base_class_type="hardware_interface::SystemInterface"` → `ros2_control` esperará una
  clase que derive de ahí.
- `staubli_robot_driver::StaubliHardwareInterface` → la clase exportada con `PLUGINLIB_EXPORT_CLASS`
  al final de `src/staubli_hardware_interface.cpp:693-696`.

---

## 2. Headers de `include/staubli_robot_driver/communication/`

Este subdirectorio es la "capa de red": define qué es un socket, cómo es un mensaje y cómo se
convierten los datos a bytes.

### `socket.hpp`

Define la **interfaz abstracta** `Socket` (`socket.hpp:40`): el contrato que cualquier
transporte (UDP o TCP) debe cumplir. Métodos: `connect`, `disconnect`, `is_connected`, `send`,
`receive_once`, `start_receive_thread`, `stop_receive_thread`, `is_receiving`.

Dos detalles importantes para aprender:

- Hay **dos sobrecargas de `connect`**: una con solo puertos y otra que además permite fijar la
  IP local (`socket.hpp:59-72`). En UDP esto importa, porque el socket debe saber a qué IP y
  puerto local "enlazarse" (bind).
- `receive_once` avisa en su documentación que el `std::vector` de destino debe estar
  preasignado con el tamaño máximo de buffer (`socket.hpp:99-101`). El tamaño máximo del paquete
  está en la constante `MAX_SOCKET_PACKET_SIZE = 1024` (`socket.hpp:32`).

También define la **fábrica** `SocketFactory` (`socket.hpp:130`) con el enumerado
`ProtocolType { UDP, TCP }` y el método estático `create()`. Es el punto único por el que el
resto del código pide sockets, de modo que cambiar de protocolo no obliga a tocar el driver.

### `udp_socket.hpp`

Declara `UDPSocket`, la implementación de `Socket` para UDP, y `UDPSocketFactory`.

Es una **fachada fina**: todos los métodos delegan en un `std::unique_ptr<UDPSocketImpl>`
(`udp_socket.hpp:77`). El motivo es que la implementación real, que usa Boost.Asio, es un
detalle internoprivado y no debe aparecer en la API pública. `UDPSocketFactory::create()` existe
para poder construir un socket UDP directamente, saltándose `SocketFactory`.

### `tcp_socket.hpp`

Declara `TCPSocket` con la misma forma que `UDPSocket`, es decir, la pieza equivalente **pero sin
implementar**. Estado actual, comprobado en el código:

- `src/communication/tcp_socket.cpp` incluye `tcp_socket_impl.hpp`, un fichero que **no existe**
  en el paquete ni en el historial del repositorio.
- `tcp_socket.cpp` **no está en `LIBRARY_SOURCES`**, por lo que ni siquiera se compila.
- `src/communication/socket_factory.cpp:31-32` lanza `std::runtime_error("TCP protocol not
  implemented yet")` cuando se pide `ProtocolType::TCP`.
- `test/unit_tests/communication/test_sockets.cpp:119-120` contiene el caso de prueba TCP
  comentado con la nota `Add TCP when implemented`.

En resumen: es andamiaje preparado para un futuro transporte TCP. Cuál sería exactamente su
comportamiento es **DESCONOCIDO**.

### `protocol.hpp`

Define el **formato del protocolo**, es decir, el acuerdo de bytes entre el PC y el robot:

- `MAGIC_NUMBER = 0xABCD` (`protocol.hpp:37`) y `PROTOCOL_VERSION = 1` (`protocol.hpp:42`):
  los dos valores con los que un mensaje se identifica como propio de este protocolo.
- `DEFAULT_CONTROL_PORT = 8080` y `DEFAULT_DIAGNOSTICS_PORT = 8081` (`protocol.hpp:47-52`):
  valores por defecto **en C++**.
- `FrameHeader` (`protocol.hpp:62`): la cabecera de 8 bytes de cada trama, con `serialize()`,
  `deserialize()` y `get_serialized_size() == 8`.
- `class Message` (`protocol.hpp:106`): clase base abstracta de todos los mensajes, con la
  interfaz común `serialize` / `deserialize` / `get_serialized_size` y sobrecargas que trabajan
  directamente sobre `std::vector<uint8_t>`.

El comentario de `protocol.hpp:89-105` documenta el formato en texto, lo cual es muy útil como
referencia rápida:

```
| Magic (2 B) | Versión (1 B) | Tipo (1 B) | Secuencia (2 B) | Tamaño payload (2 B) |
```

Todos los números multi-byte son **little endian**, igual que en la parte VAL3 (ver
`encodeHeader.pgx`).

### `serialization.hpp`

Conjunto de plantillas y funciones `inline` para convertir valores individuales y `std::array`
a bytes y de vuelta: `serialize_type` / `deserialize_type` para `uint8_t`, `uint16_t`, `uint32_t`,
`float` y `double`, y `serialize_array` / `deserialize_array` (`serialization.hpp:139-159`) para
arrays de tamaño fijo.

Son la capa más baja del paquete: no saben nada de robots, solo de bytes. Existe una
conversión explícita de `double` a `float` (`serialization.hpp:81-87`), porque el protocolo
transmite los valores articulares en precisión simple mientras que ROS 2 trabaja en `double`.

### `messages.hpp`

Declara los **mensajes concretos** del protocolo, todos heredando de `Message`:

- `RobotStateMessage` (`messages.hpp:64`): lo que el robot envía (posiciones, velocidades,
  par de las 6 articulaciones, estado de seguridad, flags de bits, etc.).
- `RobotCommandMessage` (`messages.hpp:141`): lo que el PC envía (tipo de comando, referencia de
  posición o velocidad, salidas digitales y analógicas, periodo de control).
- `DiagnosticDataMessage` (`messages.hpp:193`): datos de diagnóstico.

Cada clase implementa `serialize()` y `deserialize()` usando las funciones de `serialization.hpp`
y respetando el orden de campos que espera el otro extremo.

---

## 3. Otros headers de `include/staubli_robot_driver/`

### `types.hpp`

Enumeraciones que fijan los **números que viajan por el cable**, y por eso están en un header
propio y sin dependencias:

- `MessageType` (`types.hpp:24`): `ROBOT_STATE=0x01`, `ROBOT_COMMAND=0x02`, `DIAGNOSTIC_DATA=0x03`.
- `CommandType` (`types.hpp:31`): `STOP`, `JOINT_POSITION`, `JOINT_VELOCITY`, `JOINT_TORQUE`, más
  dos valores (`0x0A`, `0x0B`) que el comentario indica que usa el servidor VAL3 y que no son
  válidos como orden enviada desde el PC.
- `OperationMode`, `SafetyStatus`, `StatusFlag` y `ValidFieldFlag` (`types.hpp:69-85`): los dos
  últimos son máscaras de bits que se empaquetan en el mensaje de estado para ahorrar bytes.

### `robot_driver.hpp`

Declala clase `RobotDriver`, que es la **API principal del paquete** para hablar con el robot:

- `struct NetworkConfig` (`robot_driver.hpp:49-60`): IP del robot, IP local y los cuatro puertos
  (control/diagnósticos, local/remoto). Es el objeto que `ros2_control` rellena con los
  parámetros del URDF.
- `connect` / `disconnect` / `is_connected` (`robot_driver.hpp:79-99`).
- `send_robot_command()` y `send_stop_all_command()` (`robot_driver.hpp:105-113`).
- `read_robot_state()` y `read_diagnostic_data()` (`robot_driver.hpp:126-133`).
- `set_state_staleness_timeout()`, `set_diagnostic_staleness_timeout()` y
  `get_current_message_sequence()` (`robot_driver.hpp:140-155`).

Miembros privados a destacar: dos `std::shared_ptr<Socket>` (control y diagnósticos) y dos
interfaces de socket en tiempo real (`robot_driver.hpp:171-177`).

### `robot_driver_helpers.hpp`

Funciones `inline` sin estado para construir comandos de forma segura y repetible:
`set_command_stop`, `set_command_joint_position`, `set_command_joint_velocity` y
`prepare_robot_command_message` (`robot_driver_helpers.hpp:31-81`). La última es la más
importante: deja el comando en estado **STOP** y pone a cero las salidas, de forma que un
comando nunca nace asumiendo un modo de control.

### `real_time_socket_subscriber.hpp`

Plantilla `RealTimeSocketSubscriber<MessageType>` (`real_time_socket_subscriber.hpp:64-65`): el
lado que **recibe**. Aspectos clave:

- `struct MessageStatus` (`real_time_socket_subscriber.hpp:47`): informa si el mensaje leído es
  nuevo o es una repetición del anterior.
- `read_message()` (`:94`) no solo devuelve el último mensaje, sino que además comprueba paquetes
  perdidos y antigüedad de los datos usando el número de secuencia de la cabecera
  (documentado en `:70-86`).
- `update_message()` (`:107`) es el callback que invoca el hilo de recepción del socket.
- Hay tres instanciaciones explícitas al final del fichero (`:125-127`) para
  `DiagnosticDataMessage`, `RobotCommandMessage` y `RobotStateMessage`.

### `real_time_socket_interface.hpp`

Plantilla `RealTimeSocketInterface<MessageSub, MessagePub>` (`:41`), que **hereda** del
suscriptor anterior y le añade `send_message()` (`:54`) e `is_ready()` (`:60`). Es decir: una misma
clase puede recibir un tipo de mensaje y enviar otro. Las dos combinaciones que se usan son
`<RobotStateMessage, RobotCommandMessage>` (driver real) y `<RobotCommandMessage, RobotStateMessage>`
(mock), instanciadas explícitamente en `:69-70`.

### `staubli_hardware_interface.hpp`

Declara `StaubliHardwareInterface`, la clase que `ros2_control` carga como plugin. En el header
se ven tres bloques:

- Los **métodos del ciclo de vida de `SystemInterface`**: `on_init`, `on_configure`,
  `on_activate`, `read`, `write`, y los dos del cambio de modo
  (`prepare_command_mode_switch` / `perform_command_mode_switch`).
- Los punteros a las **interfaces de estado y comando** de `ros2_control`
  (`hardware_interface::StateInterface` / `CommandInterface`, exportadas en
  `staubli_hardware_interface.hpp:66-68`). Son la superficie que ven los controladores ROS 2.
- Los **buffers de estado y mando**: `hw_joint_positions_`, `hw_joint_velocities_`,
  `hw_joint_efforts_`, `hw_joint_position_commands_`, `hw_digital_inputs_`, etc.
  (`:126-152`), además del modo de control activo (`current_control_mode_`, `:99`).

### `mock_robot_server.hpp`

Declara `MockRobotServer`, un **robot simulado** para poder probar el driver sin el equipo físico.
Aporta su propia `NetworkConfig` (`mock_robot_server.hpp:51-59`) y una API de cuatro operaciones:
`init`, `is_ready`, `disconnect`, `send_mock_robot_state`, `send_mock_diagnostic_data` y
`get_last_received_command` (`mock_robot_server.hpp:79-114`).

La idea clave es que **invierte los papeles de la conexión**: el mock se conecta a los puertos
locales del driver en lugar de esperar que el driver se conecte a él.

---

## 4. `src/` — implementaciones y ejecutables auxiliares

### 4.1 Núcleo de comunicación: `src/communication/`

- **`protocol.cpp`** — implementa `FrameHeader::serialize()` y `FrameHeader::deserialize()`
  escribiendo y leyendo los 8 bytes en little endian, y las utilidades de la clase base
  `Message`. Es el punto donde la definición en el header se convierte en bytes reales.
- **`messages.cpp`** — implementa el `serialize`/`deserialize` de `RobotStateMessage`,
  `RobotCommandMessage` y `DiagnosticDataMessage`. Aquí es donde se empaquetan las máscaras
  `StatusFlag` y `ValidFieldFlag`, y donde se comprueba que el buffer sea suficientemente grande.
- **`socket_factory.cpp`** — implementación de `SocketFactory::create()`. Para `UDP` devuelve un
  `UDPSocket` mediante `UDPSocketFactory`; para `TCP` lanza la excepción indicada antes
  (`:31-32`). Su tamaño es mínimo porque solo hace de punto de entrada a la red.
- **`udp_socket.cpp`** — la fachada. Cada método de `UDPSocket` delega en `impl_`. Añade
  comprobaciones de tamaño de paquete frente a `MAX_SOCKET_PACKET_SIZE`.
- **`udp_socket_impl.hpp`** — **la única implementación real de un socket**, header-only
  (clase `UDPSocketImpl` con Boost.Asio). Ahí están el `io_context`, el `socket` de Asio, el
  `steady_timer` para los tiempos de espera y el hilo de recepción. Al ser privada, los demás
  archivos no dependen de Boost.
- **`tcp_socket.cpp`** — el andamiaje descrito en el apartado 2. No se compila.

### 4.2 Lectura y escritura en tiempo real

- **`real_time_socket_subscriber.cpp`** — lógica de la plantilla del suscriptor: arranca el hilo
  de recepción del socket, valida cabecera (magic, versión, tamaño), deserializa en el mensaje
  del tipo parametrizado y lleva la cuenta del número de secuencia para detectar huecos y
  mensajes antiguos.
- **`real_time_socket_interface.cpp`** — añade el envío: serializa el mensaje de salida con
  `serialize()` y llama a `Socket::send()`.

### 4.3 Driver del robot

- **`robot_driver.cpp`** — la orquestación. Al inicializar crea dos sockets por
  `SocketFactory::create(UDP)`, uno de control y otro de diagnósticos, y dos
  `RealTimeSocketInterface`. `connect()` valida la configuración y espera a que las interfaces
  estén listas; `read_robot_state()` delega en la interfaz de control y aplica los tiempos de
  antigüedad configurados; `send_robot_command()` delega en la misma interfaz. También expone
  `get_current_message_sequence()`, que la interfaz de hardware usa para rellenar el número de
  secuencia del comando.

### 4.4 Puente con `ros2_control`

- **`staubli_hardware_interface.cpp`** — el fichero más grande y el que conecta todo lo anterior
  con ROS 2. Se divide en cuatro bloques legibles:
  1. **Inicialización** (`on_init`, `on_configure`): lee los parámetros de red del URDF
     (`robot_ip`, puertos), construye el `RobotDriver` y registra las interfaces de estado y
     comando.
  2. **Ciclo** (`read`, `write`): `read()` pide el estado al driver y lo vuelca en los punteros de
     `ros2_control` (`:406-412`); `write()` traduce radianes a grados, decide STOP o posición
     según el modo activo, empaqueta salidas digitales/analógicas, rechaza `NaN` con un comando
     de parada y envía (`:441-497`).
  3. **Cambio de modo** (`:500-624`): `resolve_mode()` deduce el modo a partir del nombre de las
     interfaces; `prepare_command_mode_switch()` valida y guarda el modo; `perform_command_mode_switch()`
     lo confirma, comprueban que el robot no esté en error ni con frenos puestos, inicializa los
     comandos a la posición actual y pone velocidades y aceleraciones a cero.
  4. **Traducción de unidades y estado** (`:626-689`): `update_state()` convierte grados a
     radianes y vuelca el estado (estado de seguridad, modo de operación, etc.) en las interfaces
     de supervisión.

### 4.5 Utilidades de prueba en C++

- **`mock_robot_server.cpp`** — implementación del mock. En el constructor crea dos sockets UDP;
  en `init()` **conecta en sentido inverso** (a los puertos locales del driver, `:77-88`) y monta
  una `RealTimeSocketInterface` invertida. `send_mock_robot_state()` publica un estado falso,
  `get_last_received_command()` devuelve la última orden recibida por el driver, y el diagnóstico
  se envía serializando a mano (`:198-220`).
- **`test_receive_robot_state.cpp`** — ejecutable independiente (se instala en
  `lib/<paquete>/`) que abre un socket UDP contra `192.168.0.254:11000`, cuenta los mensajes que
  llegan y responde con un `ACK` de texto. Es una prueba manual de conectividad, no un test
  automático: no usa el protocolo binario.

---

## 5. Paquete Python: `staubli_robot_driver/`

Este es el **lado de gestión del robot**, independiente del bucle de control. Usa FTP porque es
el mecanismo que el controlador CS9 expone para transferir aplicaciones y leer logs, y está
envuelto en un nodo de ROS 2 para que sus parámetros (`robot_ip`, `username`, `password`) se puedan
configurar igual que en cualquier otro nodo.

### `__init__.py`

Vacío. Existe únicamente para que el directorio se reconhezca como paquete Python importable
(`import staubli_robot_driver.staubli_ftp`).

### `staubli_ftp.py`

Clase `StaubliFTP`, que hereda de `rclpy.node.Node` (`:29`) y envuelve la librería estándar
`ftplib`. Detalles:

- En `__init__` (`:40-56`) inicializa ROS 2, crea el nodo `staubli_ftp_client`, declara los
  parámetros `robot_ip` (por defecto `192.168.0.254`), `username` y `password`, y llama a
  `connect()`.
- `connect()` / `disconnect()` / `is_connected` (`:91-118`): conexión y cierre del servidor FTP.
- Operaciones de directorios remotos: `remote_directory_exists`, `create_remote_directory` y
  `delete_remote_directory` (`:122-184`), esta última recursiva.
- Transferencia de ficheros: `upload_file` (`STOR`), `upload_directory` (recursiva),
  `download_file` (`RETR`) y `download_directory` (`:188-251`).

### `staubli_ftp_utils.py`

Funciones de nivel superior que combinan `StaubliFTP` con las rutas concretas del controlador:

- `download_logs()` (`:36-99`): baja una lista fija de ficheros de `/log`
  (`arm.json`, `staubli-update.log`, `system.log`, `thread_usage.json`, `user.log`, `user.old`)
  al directorio local indicado.
- `upload_application()` (`:102-144`): sube la aplicación VAL3 a `/usr/usrapp/<nombre>`,
  borrando antes la versión anterior si existe.
- `upload_sio_config()` (`:147-182`): sube la configuración SIO, exigiendo que el fichero local
  se llame `sio.cfx` y escribiéndola en `/usr/configs/sio.cfx`.
- `green_text()` y `blue_text()`: pequeños ayudantes para colorear la salida de consola.

---

## 6. `scripts/`

Son los puntos de entrada que el usuario final ejecuta. Todos se instalan en `lib/<paquete>/`
(`CMakeLists.txt:160-166`), así que quedan disponibles como comandos tras `source install/setup.bash`.

| Script | Qué hace |
|---|---|
| `upload_val3_server.py` | Crea el cliente FTP, sube `share/<paquete>/val3/userapp/ros2_server` al controlador y, opcionalmente (`--ros-args -p upload_sio_config:=false`), también `sio.cfx`. Recuerda en el log que hay que recargar la aplicación desde el teach pendant (`Disk -> ros2_server`). |
| `remove_val3_server.py` | Borra de forma recursiva `/usr/usrapp/ros2_server` del controlador. |
| `download_logs.py` | Descarga los logs del robot a `.staubli_robot_logs/`. |
| `show_logs.py` | **Visualizador local**: no habla con el robot. Convierte `user.log` a CSV usando `jq` y `bash` (`:76-88`), y lo muestra coloreando `ERROR` en rojo y `WARN` en amarillo, en un `less` paginado. Por defecto solo muestra lo ocurrido desde el último arranque (busca la última entrada de tipo `run`). |

---

## 7. `test/`

### Estructura

- `test/launch/` — contiene únicamente `.gitkeep`. No hay tests de integración con `launch_testing`
  en el paquete, aunque `launch_testing` y `launch_ros` estén declarados en `package.xml`.
- `test/unit_tests/communication/` — tests de la capa de red.
- `test/unit_tests/` — tests del driver y de la interfaz de hardware.

### Propósito de cada test

| Test | Qué verifica |
|---|---|
| `communication/test_primitives_serialization.cpp` | Que `serialize_type` / `deserialize_type` y sus variantes de array hagan ida y vuelta sin pérdida, incluida la conversión `double` → `float`. |
| `communication/test_message_serialization.cpp` | Que la cabecera, el estado del robot, el comando y los datos de diagnóstico se serialicen y deserialicen correctamente, y que se detecten buffers demasiado pequeños. |
| `communication/test_sockets.cpp` | Comportamiento de los sockets sobre `127.0.0.1` con un par de sockets: envío, recepción, tiempos de espera, estado de conexión y tamaño de datos. Está parametrizado por tipo de socket, aunque hoy solo se instancia UDP (el caso TCP está comentado). |
| `communication/test_rt_socket_interface.cpp` | La comunicación bidireccional en tiempo real: recepción de mensajes, detección de paquetes perdidos y secuencia correcta, usando dos sockets loopback. |
| `test_robot_driver.cpp` | `RobotDriver` contra sockets simulados: conexión, desconexión, lectura de estado y control de mensajes inválidos. |
| `test_mock_robot_server.cpp` | El propio `MockRobotServer`: construcción, `init`, `is_ready`, envío de estado y de diagnóstico, y recepción de comandos. |
| `test_staubli_hardware_interface.hpp` | **Cabecera auxiliar** con la fixture compartida: levanta el `MockRobotServer` en un hilo, fabrica una configuración de hardware mínima y carga el plugin con `pluginlib` (por eso se prueba como caso especial en `CMakeLists.txt:225-237`, enlazando también la librería de la interfaz de hardware). |
| `test_staubli_hardware_interface.cpp` | La interfaz de hardware: construcción, `on_init` con y sin configuración, `on_configure`, carga del plugin, exportación de interfaces de estado y mando, y el ciclo `read` / `write`. |

Todos se registran con GoogleTest a través de la función auxiliar `add_unit_test`
(`CMakeLists.txt:189-204`).

---

## 8. `val3/` — la parte que vive dentro del robot

Todo este directorio se instala en el PC (`share/<paquete>/val3`) pero **no se ejecuta en el PC**:
son ficheros que se copian al controlador CS9. `val3/COLCON_IGNORE` está ahí para que `colcon`
ignore la carpeta al recorrer el workspace, ya que no es un paquete ROS.

### Archivos de configuración y documentación

- `val3/README.md` — instrucciones manuales: requisitos (controlador **CS9**, VAL3 `8.x` o
  superior, addons `motion` y `velocity`), configuración de red (`192.168.0.1/24` en el PC,
  controlador en `192.168.0.254`) y los puertos de los sockets SIO. Se Tested con un **TX2-60L**.
- `val3/configs/sio.cfx` — configuración de la pila SIO del controlador. Define un puerto serie y
  dos **sockets UDP**: `control` (puerto 11000, "publish state and receive commands") y
  `diagnostics` (puerto 11001), ambos apuntando a `192.168.0.1`, terminador de línea `10` y sin
  timeout. Esta configuración habilita los sockets en el sistema operativo del CS9.

### Estructura de la aplicación `userapp/ros2_server/`

| Fichero | Papel |
|---|---|
| `ros2_server.pjx` | **Descriptor del proyecto**: lista los 22 programas, la base de datos y los 6 tipos. Es el punto de entrada que el teach pendant lee para cargar la aplicación. |
| `ros2_server.dtx` | **Base de datos**: declara `siSocketControl` (enlazado a `UdpSocket\control` de `sio.cfx`), el flag `bIsOverrun`, la descripción de movimiento `mMdesc` y contadores (`nOverruns`, `nErrorCode`, `nInError`). |
| `start.pgx` | Rutina `start`. Comprueba la versión de VAL3, avisa si la velocidad de monitorización está por debajo del 100 %, limpia el buffer del socket y **crea la tarea de control** con periodo de 4 ms (`taskCreateSync "controlTask"`, `:45`). |
| `stop.pgx` | Rutina `stop`: limpia el buffer del socket y detiene la tarea. |
| `controlTask.pgx` | **El corazón de la aplicación**: el bucle de 4 ms que publica el estado, recibe el comando, comprueba los retardos de secuencia, ejecuta el movimiento y gestiona el error. |
| `updateState.pgx` | Rellena la estructura `RobotState` leyendo el robot real: `isPowered()`, `esStatus()`, `workingMode()`, `herej()` (posiciones), `$getJntSpeedCmd` (velocidades) y `getJointForce` (pares). Deja marcados como no disponibles sensor F/T y entradas digitales/analógicas. |

### Carpetas con las subrutinas

- **`Comm/`** — todo lo que toca la red, sin importar: `sendData` (escribe con `sioSet` y verifica
  que se escribieron todos los bytes), `readData` (reintenta hasta 2 veces) y `readDataOnce`,
  `resetControlSocket`, `getNextSequenceNumber` y `computeSequenceDelay` (calcula el retardo
  entre el número de secuencia recibido y el propio).
- **`Control/`** — las cuatro operaciones de movimiento: `updateJointPositionControl`,
  `stopJointPositionControl`, `updateJointVelocityControl` y `stopJointVelocityControl`.
- **`Serialization/`** — el espejo exacto de `protocol.hpp`/`messages.hpp` en VAL3:
  `initHeader` (cabecera de 8 bytes, magic `43981` = `0xABCD`, versión 1),
  `encodeHeader`/`decodeHeader`, `encodeRobotStateMsg`/`decodeRobotCommandMsg` y los
  `init*Msg` correspondientes. El layout se escribe con `toBinary(valor, tamaño, "2l"/"4.0l")`,
  es decir, little endian, igual que en C++.
- **`Types/`** — declaraciones de tipos, cada uno como una carpeta con su propio `.pjx`
  (proyecto), `.dtx` (datos), `start.pgx` y `stop.pgx`: `MessageHeader`, `RobotState`,
  `RobotStateMsg`, `RobotCommand`, `RobotCommandMsg` y `ThrottleLogMsg`. Los tipos están separados
  en proyectos propios porque es la forma en que el entorno de programación de Staubli los
  organiza.
- **`Utils/`** — `logThrottleMsg.pgx`, un log que se emite con intervalo mínimo para no inundar
  el log del controlador con el mismo aviso (se usa para el retardo de secuencia).

### Qué código corre en cada lado

- **PC (este paquete)**: ROS 2, `ros2_control`, C++ (sockets, serialización, driver) y los scripts
  Python de gestión por FTP.
- **Controlador CS9**: la aplicación `ros2_server` en VAL3, que habla el mismo protocolo binario
  y ejecuta el movimiento real.

Los ficheros son los mismos antes y después del despliegue: el `ros2_server.pjx` que se ve en el
repositorio es exactamente el que se carga en el robot. `upload_val3_server.py` se limita a
copiarlo.

---

# Cómo encaja todo

### 1. El arranque, de fuera hacia dentro

1. `ros2_control` arranca el nodo del robot y busca el plugin declarado en
   `staubli_hardware_interface.xml`, que apunta a `staubli_robot_driver_hardware_interface`.
2. `pluginlib` carga esa clase, es decir, `StaubliHardwareInterface`, y llama a `on_init()` y
   `on_configure()` con la configuración del URDF.
3. La interfaz de hardware crea un `RobotDriver` y le pasa la configuración de red.
4. `RobotDriver` pide dos sockets a `SocketFactory` (UDP), levanta las interfaces en tiempo real
   y se conecta al robot.
5. En paralelo, en el CS9, la aplicación `ros2_server` ya está cargada y arrancada desde el teach
   pendant: `start.pgx` ha creado `controlTask` con periodo de 4 ms.

### 2. El ciclo de control (cada milisegundo)

```
Controlador ROS 2 (topic / command)
        │  rad/s
        ▼
StaubliHardwareInterface::write()      ← convierte a grados, decide STOP/posición
        │  RobotCommandMessage
        ▼
RobotDriver::send_robot_command()
        │
        ▼
RealTimeSocketInterface<RobotState, RobotCommand>::send_message()
        │  serialize()  →  bytes
        ▼
Socket (UDP)  ──────────────────────────────►  CS9: controlTask
                                                       │ readData
                                                       │ decodeRobotCommandMsg
                                                       │ update/stop JointPosition|VelocityControl
CS9: controlTask                                   │ updateState + encodeRobotStateMsg
        │  RobotStateMessage  ◄──── serialize() ───┘ sendData
        ▼
RobotDriver (hilo de recepción)
                                                       │  RealTimeSocketSubscriber
StaubliHardwareInterface::read()  ◄── rad/s ◄───────────┘  buffers de ros2_control
        │
        ▼
Controlador ROS 2 (topic / state)
```

Tres cosas que conviene entender de este esquema:

- **El bucle es simétrico y dirigido por el robot.** El robot envía su estado cada 4 ms; el PC
  responde con el comando. Los números de secuencia viajan en ambos sentidos y ambos lados
  calculan el retardo con `computeSequenceDelay` (VAL3) y con el suscriptor de C++.
- **La latencia se trata como un error de seguridad.** En `controlTask.pgx` el retardo supera
  umbrales escalonados: aviso por encima de 3, error fatal si el robot se está moviendo y el
  retardo supera 5, o error fatal sin movimiento por encima de 25. Al fallar, el robot para y
  reinicia el socket.
- **Las unidades se convierten solo en el borde.** VAL3 razona en grados y ROS 2 en radianes; la
  conversión ocurre en `staubli_hardware_interface.cpp` al entrar y al salir, nunca en la capa de
  red.

### 3. El diagnóstico

Existe un segundo socket, `diagnostics`, declarado en `sio.cfx` (puerto 11001) y cableado en
`RobotDriver` y `MockRobotServer`, y existe el mensaje `DiagnosticDataMessage`. Sin embargo, la
aplicación `ros2_server` de este repositorio **solo usa `siSocketControl`**; ningún `.pgx` envía
diagnósticos. Qué componente del CS9 publica esos mensajes es **DESCONOCIDO** a partir de este
paquete.

### 4. La gestión (fuera del bucle)

El bucle de control es de tiempo real y no se puede interrumpir. Por eso el despliegue se hace
por otra vía, FTP, en scripts independientes:

```
scripts/upload_val3_server.py
   └── StaubliFTP ──FTP──► CS9: /usr/usrapp/ros2_server   (ficheros .pgx/.pjx/.dtx)
   └── StaubliFTP ──FTP──► CS9: /usr/configs/sio.cfx      (sockets UDP del sistema)
   └── el usuario recarga la app desde el teach pendant

scripts/download_logs.py ──► .staubli_robot_logs/
scripts/show_logs.py      ──► CSV coloreado en el terminal
scripts/remove_val3_server.py ──FTP──► borra /usr/usrapp/ros2_server
```

### 5. Qué se puede probar sin robot

`MockRobotServer` invierte la conexión y habla el protocolo real, de modo que los tests
ejercitan serialización, sockets, driver e interfaz de hardware contra `127.0.0.1`. Eso permite
validar casi toda la cadena en la CI, salvo el comportamiento real del hardware y el lenguaje
VAL3.

---

## Archivos más importantes

Ordenados por los que conviene leer primero para entender el paquete:

1. **`include/staubli_robot_driver/communication/protocol.hpp`** — el formato de la trama; sin esto
   nada de lo demás tiene sentido.
2. **`include/staubli_robot_driver/communication/messages.hpp`** — qué campos viajan.
3. **`src/robot_driver.cpp`** — el orquestador: sockets, envío, recepción y validez.
4. **`src/staubli_hardware_interface.cpp`** — el puente con `ros2_control` (conversión de unidades
   y cambio de modo).
5. **`val3/userapp/ros2_server/controlTask.pgx`** — el bucle equivalente del lado del robot.
6. **`include/staubli_robot_driver/communication/serialization.hpp`** y
   `val3/userapp/ros2_server/Serialization/` — los dos lados de la misma serialización.
7. **`staubli_robot_driver/staubli_ftp.py`** + **`scripts/upload_val3_server.py`** — cómo se
   despliega la aplicación VAL3.
8. **`CMakeLists.txt`** — qué se compila realmente (y, por tanto, qué código está activo).

## Observaciones y puntos marcados como DESCONOCIDO

- **TCP está sin implementar**: existen `tcp_socket.hpp` y `tcp_socket.cpp`, pero la
  implementación (`tcp_socket_impl.hpp`) no aparece en el repositorio, el `.cpp` no se compila y la
  fábrica lanza una excepción. El comportamiento previsto es **DESCONOCIDO**.
- **Dos juegos de puertos distintos**: los valores por defecto en C++ son 8080/8081
  (`protocol.hpp:47-52`), mientras que `val3/configs/sio.cfx` configura 11000/11001. El valor
  efectivo en ejecución procede de los parámetros de `ros2_control` definidos fuera de este paquete
  (URDF), por lo que cuál se usa realmente es **DESCONOCIDO** dentro del paquete.
- **Cabecera en el lado VAL3**: `encodeHeader.pgx:58-61` escribe `nMsgSequenceNumber` dos veces
  seguidas (posiciones 4-5 y 6-7 del búfer), mientras que en C++ esos dos últimos bytes son
  `sequence_number` y `payload_size` respectivamente. Es una observación literal del código; la
  intención del autor es **DESCONOCIDO**.
- **Sensor de par y entradas digitales/analógicas**: el mensaje de estado contempla esos campos y
  el `MockRobotServer` los envía, pero `updateState.pgx:95-108` los marca como no disponibles, así
  que el robot real no los envía todavía.
- **`test/launch/`** está vacío: los tests de integración con `launch_testing` anunciados en
  `package.xml` no existen en este paquete.
- **Valores no documentados**: el significado exacto de `nRobotStatus` frente a `workingMode()` y
  de los bits de E/S en `decodeRobotCommandMsg` no queda determinado por los ficheros de este
  paquete.
