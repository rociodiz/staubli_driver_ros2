# Explicación del paquete `staubli_bringup`

Guía de estudio del paquete `staubli_bringup`. Está escrita para alguien que está aprendiendo ROS 2
y que quiere entender **qué papel juega este paquete dentro del proyecto `staubli_driver_ros2`** y
**cómo se arranca cada parte del sistema**.

La idea central es que `staubli_bringup` **no implementa nada**. Es la capa que *ensambla*: es el
único paquete del repositorio que conoce a todos los demás y sabe en qué orden arrancarlos, con
qué configuración y con qué argumentos. Por eso está lleno de *launch files*.

Todo lo que sigue está contrastado con el contenido real de los ficheros de la carpeta y con los
launch de los otros paquetes del repositorio que los consumen. Cuando algo no puede determinarse a
partir de los ficheros, se indica explícitamente.

---

## 1. Qué es `staubli_bringup` y qué papel tiene en el proyecto

### 1.1 El concepto de "bringup"

"**Bringup**" es un término habitual en ROS 2 para el paquete que permite **levantar un robot
completo y listo para usar**. Su contenido típico son *launch files* y ficheros de configuración
de alto nivel. Por convención se llama `*_bringup` y casi siempre contiene algo como:

```
ros2 launch <paquete_bringup> <fichero_de_arranque>.launch.py
```

El nombre es descriptivo: *bring up* es una expresión del inglés que significa "poner en pie",
"levantar", "arrancar". Este paquete pone en pie el robot.

### 1.2 El papel dentro del repositorio

El repositorio `staubli_driver_ros2` está partido en piezas muy especializadas:

| Paquete | Qué hace | ¿Lo usa `staubli_bringup`? |
|---|---|---|
| `staubli_robot_description` | Genera el URDF del robot (Xacro, mallas, `r2c/`). | Sí, invoca su Xacro. |
| `staubli_robot_driver` | Habla con el robot real por UDP e implementa la `hardware_interface`. | No directamente: solo a través del URDF. |
| `staubli_moveit_config` | Configuración de MoveIt (SRDF, cinemática, controladores). | Sí, incluye su launch. |
| `staubli_gui` | Interfaz gráfica de control. | Sí, desde `launch_gazebo_gui`. |
| `staubli_joystick_servo` | Teleoperación por velocidad sin MoveIt. | Indirectamente (lo lanza `staubli_gui`). |
| `staubli_trajectory` | Cálculo y envío de trayectorias fuera de MoveIt. | No. |
| **`staubli_bringup`** | **Ensambla todo lo anterior.** | Es el orquestador. |

La consecuencia práctica es que **`staubli_bringup` es el único sitio donde se decide qué procesos
existen a la vez**. Y esa decisión no es trivial, porque hay varios modos de ejecución
mutuamente excluyentes: con Gazebo o sin Gazebo, con hardware simulado o con el robot real, con
MoveIt o sin MoveIt, con RViz o sin RViz. Cada launch file de esta carpeta es **una combinación
concreta** de esas decisiones.

### 1.3 Lo que contiene, y lo que no contiene

**Contiene:**

- Cinco *launch files*, que son programas de Python que describen qué procesos arrancar.
- La configuración del `controller_manager` (`config/controllers.yaml`).
- Un mundo de simulación para Gazebo (`worlds/inspection_cell.sdf`).
- Una configuración de RViz (`rviz/default.rviz`).
- Un layout de PlotJuggler para analizar trayectorias (`plotjuggler/`).
- Un pequeño script de espera (`scripts/wait_for_robot_ready.py`).

**No contiene, y es importante saberlo:**

- Ni una línea de C++ ni de Python que implemente lógica del robot.
- El modelo geométrico (eso es `staubli_robot_description`).
- La configuración de MoveIt (eso es `staubli_moveit_config`).
- El driver del robot (eso es `staubli_robot_driver`).

Y hay una observación curiosa: aunque el `package.xml` declara `staubli_gui` como dependencia,
**no declara `ros_gz_sim`, `ros_gz_bridge`, `robot_state_publisher`, `staubli_moveit_config` ni
`staubli_joystick_servo`**, que sí se usan en los launch. Verificado por comparación entre los
`package="..."` de los launch y los `<exec_depend>` del manifiesto. Además declara
`joint_state_publisher`, que no se usa en ningún launch de esta carpeta. Son inconsistencias del
manifiesto que no impiden nada en la práctica (si los paquetes están instalados, funcionan), pero
que conviene conocer.

---

## 2. Estructura de la carpeta

```
staubli_bringup/
├── package.xml                      Dependencias (con varias ausentes, ver §1.3)
├── CMakeLists.txt                   Instalación de todo; no compila código
├── config/
│   └── controllers.yaml             Configuración del controller_manager y sus controladores
├── launch/
│   ├── launch_demo.launch.py        Atajo: control + MoveIt en un comando
│   ├── launch_robot_control.launch.py  ros2_control (mock, Gazebo-less, o robot real)
│   ├── launch_gazebo.launch.py      Gazebo Sim + gz_ros2_control + puentes gz<->ROS
│   ├── launch_gazebo_gui.launch.py  Gazebo + ventana + GUI de control
│   ├── launch_moveit.launch.py      Espera al robot y arranca MoveIt (+ RViz)
│   └── __pycache__/                 Artefactos de Python (no es código fuente)
├── worlds/
│   └── inspection_cell.sdf          Mundo de Gazebo: suelo, luz, mesa, plataforma, pieza
├── rviz/
│   └── default.rviz                 Vista de RViz "solo robot" (sin MoveIt)
├── plotjuggler/
│   └── joint_trajectory_control.xml Layout de gráficas de posiciones y velocidades
├── scripts/
│   ├── wait_for_robot_ready.py      Espera a /robot_description y /joint_states
│   └── __pycache__/                 Artefactos de Python
└── test/
    └── .gitkeep                     Carpeta vacía (placeholder de Git)
```

Nótese que hay **dos carpetas de `__pycache__`**: son directorios que Python crea automáticamente
al importar un módulo. Contienen ficheros `.pyc` (bytecode compilado) y **no son código fuente**.
Se mencionan porque `install(DIRECTORY launch scripts ...)` los copia también al `share` instalado,
y porque verlas ahí resulta sorprendente si no se sabe qué son.

---

## 3. Las cinco formas de arrancar el robot, y por qué no son la misma cosa

Esta es la parte del proyecto donde más se confunde todo el mundo, así que conviene separar los
cinco conceptos con precisión. Son cosas distintas y, en general, excluyentes entre sí.

### 3.1 Resumen comparativo

| | Qué arranca | Dónde se ejecuta la física | Qué hardware se necesita | Launch file |
|---|---|---|---|---|
| **Gazebo** | Simulador Gazebo Sim | En Gazebo | Ninguno | `launch_gazebo.launch.py` |
| **ros2_control** | `ros2_control_node` + controladores | En **ningún sitio** (no hay física) | Ninguno, o el robot real | `launch_robot_control.launch.py` |
| **RViz** | Solo el visualizador | Ninguna | Ninguno | Nunca solo: se incluye en los demás |
| **MoveIt** | `move_group` | Ninguna (planifica trayectorias) | Ninguno | `launch_moveit.launch.py` |
| **Robot real** | El driver UDP | En el controlador CS9 | El robot físico | `launch_robot_control.launch.py use_mock_hardware:=false` |

### 3.2 Las cinco ideas, una por una

**Gazebo = el simulador.** Gazebo Sim calcula física: gravedad, contactos, colisiones, dinámica de
las articulaciones. Es el único componente del proyecto que **simula de verdad**. En Gazebo, el
robot no se dibuja: se *simula*. Se arranca con `launch_gazebo.launch.py` y se le da un mundo
(`worlds/inspection_cell.sdf`). Detalle importante: el robot **no** está en el fichero del mundo;
se genera aparte desde el topic `/robot_description` (ver §5.2).

**ros2_control = el sustituto del robot.** `ros2_control` es la capa que hace que un robot sea
"programable" desde ROS 2 con una interfaz común. Por encima de la física, `ros2_control` ofrece
*controladores* (como `joint_trajectory_controller`) que aceptan objetivos de trayectoria y los
traducen a órdenes de las articulaciones. Se arranca con `launch_robot_control.launch.py`.

La clave para entender este proyecto: **`ros2_control` no es una simulación por sí mismo**. Con
`use_mock_hardware:=true` (el valor por defecto) no hay física en absoluto: las posiciones se
simulan en memoria. Los ángulos de las articulaciones "se mueven" porque un plugin ficticio
responde, no porque exista un motor. Por eso el launch se llama `launch_robot_control` y no
`launch_simulation`: es el modo "sin nada", útil para probar la cadena de comunicación sin robot.

**RViz = solo el visualizador.** RViz **no simula y no controla**: únicamente dibuja. Y en este
paquete **no existe un launch para RViz**, porque RViz no funciona solo: necesita el URDF y las
transformaciones que publica `robot_state_publisher`. Por eso siempre aparece como una más de las
cosas que lanza otro launch. Hay además dos configuraciones distintas: `rviz/default.rviz` (esta
carpeta, solo robot) y `config/moveit.rviz` (en `staubli_moveit_config`, con el panel
MotionPlanning). Cuál se use depende de si hay MoveIt.

**MoveIt = el planificador.** MoveIt calcula *cómo* llegar de una posición a otra: resuelve la
cinemática inversa, busca un camino sin colisiones y decide velocidades. Se arranca con
`launch_moveit.launch.py`. Es fundamental entender que **MoveIt no mueve el robot**: produce
trayectorias. Para moverlo hace falta además un controlador de `ros2_control` que las ejecute
(por eso `staubli_moveit_config` declara `moveit_controllers.yaml`).

**El robot real = el mismo `ros2_control`, con otro plugin por debajo.** Con
`use_mock_hardware:=false`, el URDF generado selecciona el plugin
`staubli_robot_driver/StaubliHardwareInterface` en lugar del ficticio. A partir de ahí, las
órdenes dejan de quedarse en memoria y viajan por UDP al controlador del robot. **La diferencia
con el modo simulado está en una sola línea del URDF**, pero el efecto es enorme: se pasa de
"memoria" a "red y máquina real".

### 3.3 Por qué los modos no se mezclan (y cómo se combinan)

Las combinaciones están permitidas, pero con un matiz importante: el hardware se elige **una sola
vez**, a nivel del URDF, y de él dependen tanto `ros2_control` como Gazebo.

La decisión se toma con dos argumentos del Xacro de `staubli_robot_description`:
`use_mock_hardware` y `use_gazebo_hardware`. El URDF generado elige el plugin según estos
argumentos (`staubli.urdf.xacro:111-120` y `staubli.urdf.xacro:95-102`).

La consecuencia práctica es que **Gazebo y el `ros2_control` "de verdad" son mutuamente
excluyentes**: si el URDF se genera con `use_gazebo_hardware:=true`, el `ros2_control_node` de
`launch_robot_control.launch.py` no debe usarse, porque el plugin de Gazebo ya crea su propio
`controller_manager`. Por eso `launch_gazebo.launch.py` no lanza `ros2_control_node`, y este dato
merece recordarse porque resulta contraintuitivo.

Lo mismo con MoveIt: se puede usar MoveIt tanto en simulación como contra el robot real, porque en
ambos casos lo que hace es planificar. Lo que cambia es quién ejecuta la trayectoria. En el robot
real, el URDF es el mismo pero con el plugin real, y MoveIt no se entera de la diferencia.

---

## 4. Los conceptos de ROS 2 que aparecen en esta carpeta

Antes de entrar fichero por fichero, conviene tener claros los cinco conceptos que todo launch de
este paquete usa.

### 4.1 Launch files: programas que describen procesos

Un *launch file* es un programa de Python que devuelve un `LaunchDescription`, es decir una **lista
de acciones** que el sistema de launch ejecuta. Cada acción es una pieza: `Node` (arrancar un nodo
de ROS 2), `ExecuteProcess` (ejecutar cualquier programa), `IncludeLaunchDescription` (incluir otro
launch) o `DeclareLaunchArgument` (declarar un parámetro de entrada).

Un concepto clave: **los argumentos de launch no son parámetros**. `robot_model:=tx2_60l` en la
línea de órdenes es un argumento que se *sustituye* por texto antes de arrancar nada. Se usa
`LaunchConfiguration("robot_model")` paraodor dentro del launch donde se necesita el valor. Y como
el valor no se conoce hasta la hora de ejecutar, la sustitución es perezosa: por eso
`PathJoinSubstitution` y `Command` (que ejecutan `xacro`) existen como objetos "a resolver más
tarde".

### 4.2 Nodos, paquetes y ejecutables

En `Node(package="controller_manager", executable="spawner", ...)`:
- `package`: el paquete de ROS 2 que aporta el programa.
- `executable`: el nombre del programa dentro de ese paquete.
- `name`: el nombre del nodo en el grafo (si no se indica, se usa el del launch o el del
  ejecutable). El nombre **no** es único en el grafo: puede haber varios nodos con el mismo nombre
  en distintos namespaces, y a veces se lanzan duplicados a propósito.

### 4.3 El `controller_manager` y los *spawners*

`ros2_control` se organiza en dos niveles:

1. El **`controller_manager`** (el nodo `ros2_control_node`): lee el URDF, descubre qué
   articulaciones e interfaces hay, y administra los controladores.
2. Los **controladores**: `joint_state_broadcaster`, `joint_trajectory_controller`,
   `forward_command_controller`. Cada uno controla un conjunto de articulaciones.

Los controladores **no** se crean solos. Se cargan después con un *spawner*, que es un programa
que llama a un servicio del `controller_manager` pidiendo que cargue y active un controlador. En
este paquete, `executable="spawner"` aparece muchas veces, y es siempre la misma idea. Cada
*spawner* se lanza como proceso independiente y se autocomprueba; por eso se lanzan "en paralelo" y
no en serie. La diferencia entre cargarlos y activarlos es clave: `--inactive` los carga pero no
los activa, y `default: false` en la configuración de MoveIt indica que MoveIt no los activará
por su cuenta.

### 4.4 `ros2_control` frente a `ros_gz_control`

Son dos bibliotecas distintas, y confundirlas es fácil porque se parecen. En este proyecto:

- `ros2_control` es la capa de **control** (controladores, commanders, estados).
- `ros_gz_control` (el plugin `gz_ros2_control`) es el **puente entre Gazebo y `ros2_control`**.

En el modo Gazebo de este proyecto, `ros2_control` sigue estando presente, pero su *hardware* es
Gazebo, no un robot. El plugin `GazeboSimSystem` traduce entre el `controller_manager` y la
simulación. Es decir: los controladores no cambian de un modo a otro; lo que cambia es quién
"escucha" por debajo. Y como el URDF incluye el plugin `<gazebo><plugin>` solo cuando se pasa
`use_gazebo_hardware:=true`, la elección del hardware sigue siendo una decisión del URDF.

### 4.5 Puentes entre Gazebo y ROS 2

Gazebo y ROS 2 son dos sistemas con transportes de mensajes distintos. `ros_gz_bridge` traduce
entre ellos. En `launch_gazebo.launch.py` hay dos puentes:

- **`/clock`**: Gazebo publica el reloj de la simulación; ROS 2 lo necesita para que los nodos con
  `use_sim_time:=true` estén sincronizados con la simulación. Sin este puente, el tiempo de ROS y
  el de Gazebo divergen.
- **`/camera/image_raw`**: el sensor de cámara del robot publica en Gazebo, y el puente lo expone
  en ROS 2 como mensaje estándar para que herramientas de visión puedan leerlo.

### 4.6 Cargar un modelo en Gazebo con `ros_gz_sim create`

Gazebo no lee URDF de forma nativa: usa su propio formato, **SDF**. La conversión la hace
`ros_gz_sim create`, que suscribe al topic `/robot_description`, recibe el URDF como texto, lo
convierte internamente a SDF y llama al servicio de creación de modelos de Gazebo.

Y aquí está el punto más importante de toda esta integración: **la conversión conserva el bloque
`<gazebo><plugin>`** del URDF, que es donde está el `gz_ros2_control`. Por eso funciona: Gazebo
crea el modelo y el plugin se carga con él. Ese `<gazebo><plugin>` lo añade
`staubli_robot_description` en su Xacro cuando recibe `use_gazebo_hardware:=true`.

### 4.7 QoS y `TRANSIENT_LOCAL`

Un detalle fino pero importante aparece en `wait_for_robot_ready.py`: el topic `/robot_description`
se suscribe con `durability=TRANSIENT_LOCAL`. En ROS 2, por defecto un topic es *volátil*: si te
suscribes tarde, pierdes los mensajes anteriores. Con `TRANSIENT_LOCAL` ("transitorio local"), el
publicador *retiene* el último mensaje y lo entrega a cualquier suscriptor que llegue después.

`robot_state_publisher` publica el URDF de esta forma, precisamente para que quien quiera leerlo
(RViz, MoveIt) pueda hacerlo en cualquier momento. En cambio, `/joint_states` se suscribe como
`VOLATILE` a propósito, porque es un flujo continuo y no tiene sentido retener. El script documenta
esta decisión en sus comentarios, y es un buen ejemplo de por qué el QoS hay que pensarlo topic a
topic.

---

## 5. Fichero por fichero

### 5.1 `CMakeLists.txt`

**Qué es.** Un CMake de 22 líneas, con `build_type: ament_cmake`.

**Qué hace.** No compila nada. Solo instala. Dos instrucciones con propósitos distintos:

```cmake
install(DIRECTORY config launch rviz scripts test worlds
        DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/wait_for_robot_ready.py
        DESTINATION lib/${PROJECT_NAME})
```

La primera copia las carpetas al `share` del paquete. La segunda instala el script en `lib/<paquete>/`,
que es **la ubicación donde ROS 2 busca los ejecutables**: por eso en el launch se puede escribir
`Node(package="staubli_bringup", executable="wait_for_robot_ready.py")` y ROS lo encuentra
automáticamente, sin dar la ruta completa. Nótese que `plotjuggler/` **no** aparece en ninguna de
las dos instrucciones (ver §5.6).

**Relación.** Es la razón por la que los `FindPackageShare("staubli_bringup")` de los launch
encuentran los ficheros de `config/`, `launch/`, `worlds/` y `rviz/`.

### 5.2 `launch/launch_gazebo.launch.py` — la simulación completa

**Qué es.** El launch más complejo del paquete, y el que tiene la cabecera de documentación más
detallada: describe la arquitectura completa en seis pasos (`:15-25`).

**Qué hace.** Levanta Gazebo Sim con el robot dentro, de principio a fin. Los procesos que arranca:

| Proceso | Tipo | Función |
|---|---|---|
| `robot_state_publisher` | `Node` | Publica `/robot_description` (el URDF). |
| `GzServer` | `ros_gz_sim` | El servidor de simulación, con el mundo `inspection_cell.sdf`. |
| `parameter_bridge` (`/clock`) | `ros_gz_bridge` | Traduce el reloj de Gazebo a ROS 2. |
| `parameter_bridge` (`/camera/image_raw`) | `ros_gz_bridge` | Traduce la imagen de la cámara. |
| `ros_gz_sim create` | `Node` | Convierte el URDF en SDF y carga el modelo en Gazebo. |
| 3 × `spawner` | `controller_manager` | Cargan los controladores sobre el `controller_manager` de Gazebo. |
| `gz sim -g` | `ExecuteProcess` | La ventana de Gazebo, solo si `gui:=true`. |

**Los detalles que hay que entender, en orden:**

**a) El URDF se genera con los argumentos de Gazebo** (`:90-102`):

```python
" robot_model:=", LaunchConfiguration("robot_model"),
" use_mock_hardware:=false",
" use_gazebo_hardware:=true",
" gazebo_controllers_file:=", controllers_yaml,
```

 esos dos argumentos son la clave: `use_mock_hardware:=false` desactiva el plugin ficticio y
`use_gazebo_hardware:=true` activa el de Gazebo. El resultado es un URDF cuyo bloque
`<ros2_control>` apunta a `gz_ros2_control/GazeboSimSystem` y que además lleva el tag
`<gazebo><plugin filename="libgz_ros2_control-system.so">`. Este URDF se lo pasa como parámetro
`robot_description` a `robot_state_publisher`, que es quien lo publica en el topic.

**b) El mundo se carga, pero el robot no está en él** (`:120-138`). El comentario es explícito:
`inspection_cell.sdf` contiene el suelo, el sol y los objetos fijos, y el robot se genera aparte.
Se usa `GzServer(world_sdf_file=inspection_cell_sdf)`. El comentario también avisa de un detalle
real: `GzServer` **exige** un fichero de mundo no vacío, y aborta si no se le da ninguno. Por eso
el paquete trae su propio mundo en vez de usar el `empty.sdf` por defecto.

**c) El robot se carga desde el topic** (`:140-159`):

```python
Node(package="ros_gz_sim", executable="create",
     arguments=["-topic", "/robot_description", "-name", "staubli",
                "-z", "0.30", "-allow_renaming", "true"])
```

El `create` se suscribe a `/robot_description`, espera a que haya mensaje, y entonces inserta el
modelo en el mundo de Gazebo. Los comentarios (`:140-147`) explican las tres cosas no obvias:
que la conversión URDF→SDF conserva el `<plugin>` (y por eso `gz_ros2_control` se carga con el
modelo), que `-z 0.30` coloca el robot encima de la plataforma del mundo, y que
`-allow_renaming true` evita que falle si ya existe un modelo con ese nombre.

**d) El reloj y la cámara se puentean** (`:161-185`). El `parameter_bridge` de `/clock` es
**imprescindible**: sin él, los nodos con `use_sim_time:=true` (como el propio
`robot_state_publisher`) se desincronizarían de la simulación. El de la cámara expone el sensor
definido en `staubli_camera.xacro` del paquete de descripción a un topic de ROS 2 estándar.

**e) Los controladores se cargan sobre el `controller_manager` de Gazebo** (`:187-230`). Y aquí
está la sutileza más importante de todo el launch, explicada en los comentarios (`:188-195`): en
el modo Gazebo, **el `controller_manager` no lo crea `ros2_control_node`, lo crea el propio plugin
`gz_ros2_control` dentro de Gazebo**, en `/controller_manager`. Por eso los *spawners* apuntan a
`-c /controller_manager`. Y por eso este launch **no** lanza `ros2_control_node`: ya hay uno,
dentro del simulador. Para que funcione, el URDF incluye `<parameters>` con la ruta a
`controllers.yaml`, de modo que el plugin cargue la configuración de los controladores.

Los tres *spawners* tienen una diferencia: `joint_state_broadcaster` y
`joint_trajectory_controller` se cargan activos, mientras que `forward_velocity_controller` se
carga con `--inactive`. El motivo está en `controllers.yaml`: si se cargaran activos a la vez,
competirían por las mismas articulaciones. Se deja preparado y desactivado para poder activarlo más
tarde bajo demanda.

Los *spawners* de este launch usan además `--controller-manager-timeout 30`, que les da hasta 30
segundos para esperar a que el `controller_manager` esté disponible (el plugin necesita el URDF
antes de responder).

**f) Un `LogInfo` de ayuda** (`:242-253`): al arrancar imprime en verde los comandos útiles para
comprobar que todo funciona (`ros2 control list_controllers`, `ros2 topic echo /joint_states`, y un
`ros2 action send_goal` de ejemplo). Es un detalle pequeño pero muy útil en la práctica: el
launch se aut documenta en la terminal.

**g) La ventana de Gazebo es opcional** (`:232-238`): `gz sim -g` se lanza con
`IfCondition(gui)`, que por defecto es `false`. Es decir, por defecto Gazebo corre sin ventana
("headless"), y la interfaz se abre aparte o con `gui:=true`.

**Relación con el resto.** Usa el Xacro y la cámara de `staubli_robot_description`, el
`controllers.yaml` de esta misma carpeta, el `inspection_cell.sdf` de `worlds/`, y el
`gz_ros2_control` de `ros_gz_control`. Todo el URDF se genera antes de arrancar nada, con `xacro`.

### 5.3 `launch/launch_gazebo_gui.launch.py` — simulación con interfaz

**Qué es.** Un launch muy corto (64 líneas) cuyo propósito es claro: **componer tres cosas en un
solo comando**: Gazebo, su ventana, y la interfaz gráfica de control.

**Qué hace.** Solo dos acciones, ambas `IncludeLaunchDescription`:

```python
IncludeLaunchDescription(PythonLaunchDescriptionSource(gazebo_launch),
                        launch_arguments={"robot_model": "tx2_60l",
                                          "gui": LaunchConfiguration("gui")}),
IncludeLaunchDescription(PythonLaunchDescriptionSource(gui_launch))
```

La primera incluye `launch_gazebo.launch.py` (de esta carpeta). La segunda incluye
`staubli_gui/launch/gui.launch.py`, que a su vez lanza la GUI y el servo del joystick. Y
`gui.launch.py` (en `staubli_gui`) hace lo propio con `staubli_joystick_servo/launch/joystick_servo.launch.py`.

**El concepto importante: `IncludeLaunchDescription`.** Permite que un launch reutilice otro como
si sus procesos fueran suyos. Así, en vez de repetir la definición del robot, del mundo y de los
controladores, este fichero simplemente *delega*. Es la forma correcta de componer: cada launch
tiene una responsabilidad, y los launch "de nivel superior" los combinan. El resultado es que la
ventana de Gazebo y la interfaz de control quedan sincronizadas con la simulación desde el primer
momento, sin pasos manuales.

**Cuándo se usa.** Cuando se quiere trabajar con la celda de inspección completa, simulada y con
interfaz gráfica, en un único comando. La cabecera del fichero (`:15-26`) lo describe como
"arranque COMPLETO de la celda".

**Detalle a tener en cuenta:** el `robot_model` está **fijado a `tx2_60l`** en el código, no es un
argumento de este launch. Es decir, este launch es específico del modelo largo; no se puede elegir
otro modelo pasándolo por línea de órdenes (el valor se ignora silenciosamente). Esto contrasta
con `launch_gazebo.launch.py`, donde `robot_model` sí es un argumento con valor por defecto
`tx2_60l`.

### 5.4 `launch/launch_robot_control.launch.py` — `ros2_control` sin Gazebo

**Qué es.** El launch del control "puro", sin simulador y sin MoveIt.

**Qué hace**, en el mismo orden en que ya se explicó:

1. Genera el URDF con `xacro`, pasando `robot_model`, `use_mock_hardware` y `robot_ip`.
2. Lanza `robot_state_publisher` con ese URDF.
3. Lanza `ros2_control_node` con `config/controllers.yaml` como parámetros.
4. Carga `joint_state_broadcaster` con un *spawner*.
5. Carga `joint_trajectory_controller` con un *spawner* y `--inactive`.
6. Si `start_controller` lo indica, lo activa con `ros2 control set_controller_state`.
7. Lanza RViz con `rviz/default.rviz` si `gui:=true`.

**Los detalles que hay que entender, en orden:**

**a) El URDF se genera con `Command` y se pasa como parámetro** (`:100-118`). Se usa el mismo
mecanismo que en el modo Gazebo: un `Command` ejecuta `xacro` y captura la salida, y se envuelve
en `ParameterValue(..., value_type=str)`. El `value_type=str` es necesario porque, por defecto,
ROS interpretaría el texto como una cadena literal en vez de ejecutar el comando. Este es el
mecanismo canónico para pasar un URDF generado a un nodo, y es exactamente el mismo patrón en
`launch_gazebo.launch.py` y en `staubli_robot_description/view_robot.launch.py`.

**b) El URDF decide el hardware** (`:100-118`): los argumentos `use_mock_hardware` y `robot_ip`
deciden si el plugin es ficticio o el driver real. Con `use_mock_hardware:=true` (por defecto) el
URDF selecciona el plugin ficticio; con `false` y una `robot_ip` concreta, el driver real. **El
launch no cambia de comportamiento: lo que cambia es el URDF.** Esta es una de las cosas más
importantes que se pueden aprender de este paquete: *la diferencia entre "simular" y "hablar con el
robot real" no está en el launch, está en un argumento del Xacro que el launch le pasa.*

**c) `ros2_control_node` recibe `config/controllers.yaml` como parámetro** (`:130-145`). Aquí está
el punto conceptual clave: el `controller_manager` **necesita conocer qué controladores hay
disponibles y cómo se configuran, y esa información está en el YAML, no en el URDF**. El URDF
describe el robot; el YAML describe los controladores. Por eso `controllers.yaml` se pasa como
parámetros al nodo. Su contenido se explica en §5.8.

**d) Los controladores se cargan progresivamente** (`:147-215`). Se usa un bucle sobre una lista
`controllers_to_load = ["joint_trajectory_controller"]` y una función que construye el par
*spawner* + activación. El *spawner* se lanza con `--inactive`, y la activación se hace después
con un `OnProcessExit`, es decir, **solo cuando el proceso de carga ha terminado**. Esto es
importante: activar un controlador antes de que esté cargado fallaría. Y la activación es
condicional: solo si `start_controller` coincide con el nombre del controlador. El argumento
`start_controller` tiene `choices=["none", "joint_trajectory_controller"]` y vale `none` por
defecto, es decir, por defecto se carga pero **no** se activa.

**e) El bucle permite crecer.** El comentario `# Add your controllers here` (`:159`) señala la
lista `controllers_to_load` como el sitio donde añadir más. La estructura está preparada para
cargar varios controladores.

**f) RViz es opcional** (`:216-225`): con `gui:=true` (por defecto) lanza `rviz2` con
`rviz/default.rviz`. Esto es exactamente el "RViz de solo robot" del §3: sin MoveIt, sin panel de
planificación.

**Los tres modos de este launch**, que conviene tener claros:

| Comando | Qué pasa |
|---|---|
| `ros2 launch staubli_bringup launch_robot_control.launch.py` | Modo simulado por defecto: plugin ficticio, no hay física. RViz abierto. |
| `... use_mock_hardware:=false robot_ip:=192.168.0.254` | Robot real: el URDF selecciona el driver real, que habla por UDP. |
| `... start_controller:=joint_trajectory_controller` | Además, activa el controlador de trayectorias para poder enviar objetivos. |

**Observación verificada:** el `ros2_control_node` de este launch se lanza con
`{"use_sim_time": False}` **fijo** (`:144`), no con el `LaunchConfiguration` del argumento
`use_sim_time` que se declara (`:66-72`). Es decir, aunque se pase `use_sim_time:=true` a este
launch, el `controller_manager` receive False. En la práctica no importa, porque este launch es
precisamente el que se usa sin Gazebo (y sin Gazebo no hay reloj de simulación que sincronizar).

### 5.5 `launch/launch_moveit.launch.py` — el planificador

**Qué es.** Un launch medio, pensado para lanzarse **después** de que el robot ya esté en marcha.

**Qué hace**, en orden:

1. Declara los argumentos `robot_model`, `use_sim_time`, `warehouse_sqlite_path`, `launch_servo` y
   `gui`.
2. Lanza `wait_for_robot_ready.py` (§5.7) como nodo, y registra un evento: **cuando ese proceso
   termine con éxito, incluye el launch de MoveIt**.
3. Incluye `staubli_moveit_config/launch/staubli_moveit.launch.py`, que es quien arranca
   `move_group` y el RViz con el panel de planificación.

**El concepto más importante: `OnProcessExit`.** Es un `RegisterEventHandler` que permite encadenar
lanzamientos según el resultado de un proceso. La cabecera lo explica: *lanzar MoveIt cuando la
descripción del robot esté disponible*. Sin este mecanismo, `move_group` arrancaría en paralelo con
`robot_state_publisher` y se quedaría esperando un URDF que todavía no existe. El script de espera
es, por tanto, un **guardián**: no hace nada útil por sí mismo, solo espera el momento oportuno
para lanzar lo siguiente. Es un patrón muy útil en ROS 2 para resolver problemas de orden de
arranque.

**Diferencia clave con los otros bringup:** `launch_robot_control` y `launch_gazebo` son
"autosuficientes" (seuw robot_description ellos mismos). `launch_moveit` es **dependiente**: da por
hecho que hay un URDF y un `controller_manager` ya en marcha. El comentario de
`staubli_moveit.launch.py` lo dice: "Make sure that robot_state_publisher starts before this launch
file".

**Relación.** Es el único fichero de esta carpeta que **no** genera el URDF. Lo recibe del
`robot_state_publisher` que haya arrancado antes.

### 5.6 `launch/launch_demo.launch.py` — el atajo

**Qué es.** El launch más pequeño (91 líneas) y el más cómodo para empezar.

**Qué hace.** Su docstring lo resume: *"Launch demo with robot control and MoveIt"*. Y lo que hace
es exactamente eso: **incluye los otros dos** (`launch_robot_control` y `launch_moveit`) y los
coordena:

- Al launch de control le pasa `use_mock_hardware=true`, `gui=false` y
  `start_controller=joint_trajectory_controller`.
- Al launch de MoveIt le pasa `gui=true`.

El detalle importante está en esas dos líneas: `gui=false` para el control y `gui=true` para
MoveIt. Es un pequeño truco de diseño: **cada sub-launch abre su propia ventana de RViz, y así se
evita tener dos visores compitiendo por el mismo topic**, dejando solo el de MoveIt (que es el
completo, con el panel de planificación). También `start_controller=joint_trajectory_controller`
hace que el controlador quede activo desde el principio, listo para aceptar objetivos de
trayectoria, que es justo lo que MoveIt necesita para ejecutarlos.

**Cuándo se usa.** Cuando se quiere "levantar el robot y que todo funcione": un solo comando y el
robot aparece en RViz, con MoveIt listo para planificar. Es la respuesta a "¿por dónde empiezo?".

**Relación.** Es la composición de los otros dos, y por eso no repite código: usa
`IncludeLaunchDescription` con los dos launch. Se podría añadir un tercer sub-launch (por ejemplo
Gazebo) sin tocar este fichero.

### 5.7 `scripts/wait_for_robot_ready.py` — el guardián del arranque

**Qué es.** Un nodo de ROS 2 en Python, con 131 líneas, cuyo único propósito es **esperar**.

**Qué hace.** Se suscribe a dos topics:

- `robot_description` (tipo `String`): el URDF.
- `joint_states` (tipo `sensor_msgs/JointState`): el estado de las articulaciones.

Cuando ha recibido **ambos**, fija su `Future` y termina. Y como el launch de MoveIt está encadenado
a su salida (ver §5.5), terminar este proceso es la señal de "adelante, ya puedes arrancar".

**Por qué los dos y no solo uno.** Porque MoveIt necesita las dos cosas: el URDF para saber cómo
es el robot, y el estado actual para saber dónde está. Un URDF sin estado no permite planificar
nada, y un estado sin URDF no se puede interpretar. Por eso espera a los dos.

**Los detalles de QoS** (`:59-76`) son un ejemplo didáctico de por qué el QoS importa, y están
comentados en el propio código:

- `robot_description` se suscribe con `TRANSIENT_LOCAL` (con `depth=1`): el URDF es un dato fijo
  que se publica una vez, y el modo transitorio local garantiza que lo reciba aunque el script
  arranque tarde. El comentario explica que es lo que hace `robot_state_publisher`.
- `joint_states` se suscribe con `VOLATILE`: es un flujo continuo, y retener el último mensaje no
  tiene sentido. El comentario explica además que usar `TRANSIENT_LOCAL` aquí provocaría una
  incompatibilidad de QoS con el publicador.

Esa incompatibilidad es un concepto de ROS 2 que conviene tener presente: si el publicador y el
suscriptor no coinciden en *durabilidad*, **no se conectan**, y el síntoma típico es un nodo que
"no recibe nada" sin error visible.

**Detalles de robustez.** El script registra un `Future` por topic, y cada mensaje marca el suyo
como recibido. Cuando ambos están listos, llama a `emit_robot_ready()`. Además crea un temporizador
que, cada 5 segundos, comprueba si el robot sigue sin estar listo y lo avisa por log. Se escriben
los nombres completos de los topics en los mensajes de log, lo cual es útil para diagnosticar si
el URDF o los estados no llegan.

### 5.8 `config/controllers.yaml` — el corazón del control

**Qué es.** Un YAML de 56 líneas, y probablemente **el fichero más importante de esta carpeta**,
porque define cómo se comporta el robot cuando se le manda una trayectoria.

**Qué contiene, y qué significa cada bloque:**

**a) `controller_manager.ros__parameters`** (`:1-12`): declara el `controller_manager` y, dentro,
qué controladores **existen** y de qué tipo. El formato es anidado y un poco particular: el
contenido va bajo `ros__parameters`, que es la clave que ROS 2 usa para los parámetros de un nodo.

```yaml
controller_manager:
  ros__parameters:
    update_rate: 250  # Hz
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController
    forward_velocity_controller:
      type: forward_command_controller/ForwardCommandController
```

- `update_rate: 250` es la frecuencia del bucle interno: el `controller_manager` actualiza 250
  veces por segundo. Es un parámetro de rendimiento y estabilidad.
- Cada `type` dice qué clase implementar. Nótese que `forward_velocity_controller` es de tipo
  `forward_command_controller`: **el nombre del controlador y el de su tipo no coinciden**, porque
  `forward_command_controller` es un tipo genérico y este proyecto lo usa para velocidad.

**b) `joint_trajectory_controller.ros__parameters`** (`:14-44`): los ajustes del controlador de
trayectorias, que es el que ejecuta las órdenes de MoveIt:

| Parámetro | Significado |
|---|---|
| `joints` | Las seis articulaciones que gestiona. |
| `command_interfaces: [position]` | Por qué interfaz manda órdenes. |
| `state_interfaces: [position, velocity]` | De qué interfaces lee el estado. |
| `action_monitor_rate: 20.0` | Cada cuánto revisa la trayectoria que le llega. |
| `allow_nonzero_velocity_at_trajectory_end: true` | Permite terminar con velocidad no nula (para trayectorias que continúan). |
| `allow_partial_joints_goal: false` | Exige que la trayectoria cubra **todas** las articulaciones. |
| `interpolate_from_desired_state: true` | Parte del estado deseado al iniciar. |
| `set_last_command_interface_value_as_state_on_activation: false` | Al reactivarse, **no** adopta el último comando como estado inicial. |
| `constraints.stopped_velocity_tolerance: 0.01` | Umbral para considerar el robot parado. |
| `constraints.goal_time: 0.0` | Margen de tiempo para el objetivo (0 = exacto). |

El parámetro `set_last_command_interface_value_as_state_on_activation: false` merece atención
especial, porque lleva un comentario que explica un problema real (`:37-39`): al reactivar el
controlador de trayectorias después de haber estado en control manual, **no** debe tomar el último
comando de posición como estado, porque ese comando se habría quedado en la posición "HOME"
guardada. En su lugar, debe leer el estado medido real del robot, que refleja la posición en la que
el control manual lo dejó. Es un ejemplo de un parámetro que parece técnico pero que esconde una
decisión de diseño importante.

**c) `forward_velocity_controller.ros__parameters`** (`:46-56`): mucho más simple, porque el
`forward_command_controller` es genérico. Solo necesita saber qué articulaciones y por qué interfaz
manda:

```yaml
joints: [joint_1 ... joint_6]
interface_name: velocity
```

Este es el controlador que usa el control manual por velocidad, y también el servo del joystick
(§5.9).

**Relación con los demás ficheros.** Es el parámetro que se le pasa a `ros2_control_node` en
`launch_robot_control.launch.py` (§5.4c), y cuya ruta se inyecta en el URDF en el modo Gazebo
(§5.2a). Los seis `joints` deben coincidir con los del URDF, y con los que
`staubli_moveit_config/moveit_controllers.yaml` le dice a MoveIt que use. Es el mismo conjunto de
articulaciones en los tres sitios: URDF, controlador y MoveIt.

### 5.9 `worlds/inspection_cell.sdf` — la escena de simulación

**Qué es.** Un SDF (el formato nativo de Gazebo) de 271 líneas que define una celda de inspección:
el entorno donde vive el robot.

**Concepto clave: URDF frente a SDF.** SDF es el formato de Gazebo, y es un superconjunto del URDF
(URDF es, de hecho, un subconjunto de SDF). Gazebo trabaja con SDF; el proyecto trabaja con URDF.
Por eso hace falta el paso de conversión del que se hablaba en §4.6. Aquí, el fichero del mundo
está **escrito directamente en SDF**, porque es un fichero estático que no necesita generarse.

**Qué contiene**, y qué significa cada elemento:

**a) La cabecera (`:2-16`)** documenta la distribución de la escena: el plano del suelo, una mesa
de inspección de 1.0 × 0.6 m con la superficie a z = 0.50 m, una plataforma elevada de
0.30 × 0.30 × 0.30 m bajo el robot, y una pieza de trabajo. También deja claro algo importante:
**el robot no forma parte de este fichero**, porque se carga aparte desde `/robot_description`.

**b) `<physics>`** (`:19-23`): la física de la simulación. `max_step_size: 0.001` significa que la
simulación avanza en pasos de **1 milisegundo**, es decir, 1 kHz. `real_time_factor: 1.0` indica
que la simulación debe ir en tiempo real. Es la configuración estándar para control de robots,
porque un paso pequeño evita que los contactos sean imprecisos. El atributo `name="1ms"` es
descriptivo. El `type="ignored"` indica que el nombre es solo una etiqueta.

**c) Los `<plugin>` de sistema** (`:24-38`): son los **sistemas internos de Gazebo** que se
activan en el mundo. Cada uno aporta una capacidad:
- `Physics`: el motor de física.
- `UserCommands`: servicios para pausar, reiniciar o ejecutar cosas desde la API.
- `SceneBroadcaster`: publica el estado de la escena, y es de donde sale el `/clock` que luego se
  puentea.
- `Contact`: detección de contactos (colisiones).
- `Sensors`: los sensores, con el motor de renderizado `ogre2` (que es el que dibuja la cámara).

Nótese el contraste con §4.4: estos son plugins **de Gazebo**, mientras que
`libgz_ros2_control-system.so` es un plugin **de modelo**, que se añade con el robot.

**d) El sol** (`:40-52`): una luz direccional con sombras, necesaria para ver nada.

**e) El suelo** (`:54-79`): un plano de 100 × 100 m, estático, con su material gris.

**f) La mesa de inspección** (`:81-181`): un modelo estático, con la `<pose>` que lo coloca en el
mundo. Cada elemento visual lleva su `<pose>` local y su `<material>` (el color), y cada uno tiene
un `<collision>` gemelo: la superficie de 1.0 × 0.6 × 0.05 m y cuatro patas cilíndricas de
0.45 m. La diferencia entre `<visual>` y `<collision>` es la misma que en el URDF: lo que se ve y
lo con lo que se chocan.

**g) La plataforma del robot** (`:183-205`): una caja de 0.30 m de lado, con la `<pose>`
`0 0 0.15`, es decir, centrada a media altura para que su superficie superior quede exactamente a
**0.30 m**, que es donde se coloca el robot (el `-z 0.30` del `create` en §5.2c). El comentario
explica el porqué: el robot real tiene un controlador o pedestal debajo, y esto lo simula.

**h) La pieza de trabajo** (`:207-230`): `part_1`, una caja verde de 0.08 m (8 cm) sobre la mesa,
a z = 0.54. Es el objeto con el que el robot interactuaría en la celda de inspección.

**Cuándo se usa.** Solo en el modo Gazebo. Es el "escenario" de la simulación; el robot y los
controladores se añaden encima, pero el entorno (suelo, mesa, luz) viene de aquí.

**Detalle a tener en cuenta:** los comentarios de la cabecera y del launch `launch_gazebo` mencionan
"three workpieces" (tres piezas) y una mesa centrada en (0.00, 0.50), pero el fichero real solo
tiene **una** pieza (`part_1`) y la mesa está en `y = 0.75`. Los valores de las `<pose>` son los que
mandan, y son coherentes entre sí (la pieza cae sobre la mesa, el robot sobre la plataforma); es la
documentación en comentarios la que se ha quedado desfasada respecto al fichero.

### 5.10 `rviz/default.rviz` — la vista "sin MoveIt"

**Qué es.** Una configuración de RViz de 248 líneas. Se pasa con `-d` en el launch, que es la
forma en que RViz sabe qué mostrar al arrancar.

**Qué contiene:**

- **Panel de displays**: con un `Grid` (rejilla de referencia) y dos displays principales.
- **Display `TF`** (`:53-79`): muestra los ejes de los frames. Interesantemente, solo tiene
  `base_link`, `tool0` y `world` activados, y los seis `link_N` desactivados. Esto es una decisión
  de legibilidad: con todos los frames activados se verían seis ejes extra en medio del robot, y
  solo interessan los tres principales.
- **Display `RobotModel`** (`:96-160`): dibuja el robot desde el topic `/robot_description`, no desde
  un fichero. Con `Description Source: Topic` y el topic `/robot_description`, RViz espera a que
  alguien publique el URDF, que es exactamente lo que hace `robot_state_publisher`. Es el mismo
  patrón que usa `staubli_robot_description`, pero aquí con el topic configurado en vez del
  parámetro.
- **`Fixed Frame: world`** (`:168`): el frame de referencia de la vista, que coincide con el link
  raíz del URDF. Si este frame no existiera en el árbol de TF, RViz mostraría un error de
  "fixed frame" y no dibujaría nada.
- **Herramientas** (`:172-201`): `Interact`, `MoveCamera`, `Select`, `FocusCamera`, `Measure`,
  `SetInitialPose`, `SetGoal` y `PublishPoint`, cada una con su topic por defecto. Son las
  herramientas estándar de RViz; `PublishPoint` y `SetGoal` son útiles para ver qué topics hay
  disponibles en el sistema.
- **Vista `Orbit`** (`:206-217`): la cámara 3D guardada, con distancia, `Pitch` y `Yaw`.

**Cuándo se usa.** Es el RViz de `launch_robot_control.launch.py`, es decir, el modo en el que
quieres **ver el robot yMANDAR órdenes**, sin MoveIt. No se usa en `launch_moveit` ni en
`launch_gazebo`, que usan otras configuraciones (o ninguna).

**Comparación con la de MoveIt.** El `rviz` de `staubli_moveit_config` es parecido (tiene `Grid`,
`TF` y `RobotModel`) pero **añade el display `MotionPlanning`**, que es el que permite planificar
desde la interfaz. La diferencia entre los dos RViz es, en la práctica, la diferencia entre "ver
el robot" y "planificar con el robot".

### 5.11 `plotjuggler/joint_trajectory_control.xml` — analizar las trayectorias

**Qué es.** Un fichero XML de 308 líneas que es un **layout de PlotJuggler**, una herramienta
gratuita de representación gráfica de datos de ROS 2. El fichero no es un programa ni una
configuración de ROS: es la disposición de las gráficas de una sesión concreta.

**Qué contiene.** Dos pestañas:

- **`Joint Position`**: seis gráficas (una por articulación), cada una con dos curvas:
  `/joint_states/joint_N/position` (la posición real medida) frente a
  `/joint_trajectory_controller/controller_state/reference/positions[N-1]` (la posición de
  referencia, es decir, la que el controlador está Following).
- **`Joint velocity`**: el mismo esquema para las velocidades.

**Por qué es útil, y qué revela.** Superponer "lo que se pide" y "lo que se obtiene" es
precisamente cómo se depura un problema de control: si las dos curvas se solapan, el robot sigue
las órdenes; si hay desfase o error constante, hay un problema de seguimiento. Por eso tiene
también las gráficas de error (posición y velocidad), que son las que delatan un controlador mal
sintonizado.

**El concepto de fondo: el topic `controller_state`.** Es un topic que publica el
`joint_trajectory_controller` con su estado interno (referencia, error, coordenadas objetivo). Es
una de las mejores herramientas de diagnóstico de `ros2_control`, y conocerlo es lo que separa ver
"que el robot se mueve" de entender "por qué se mueve así".

**Cuándo se usa.** Se abre aparte con PlotJuggler: `ros2 run plotjuggler plotjuggler` y luego se
carga el layout. No lo lanza ningún launch de esta carpeta. Su ubicación natural sería un
directorio `config/` (es una configuración, no un script), pero aquí está en el suyo propia.

**Detalle a tener en cuenta:** **este directorio no se instala.** El `CMakeLists.txt` instala
`config launch rviz scripts test worlds`, y `plotjuggler` no está en la lista, así que el layout no
estará disponible en el `share` instalado (habrá que abrirlo desde el repositorio). Además, hay un
error de etiquetado menor en la primera pestaña: el panel del sexto gráfico se llama "Joint 5"
cuando es `joint_6`; en la segunda pestaña sí está bien. Solo afecta al título de la gráfica, no a
los datos.

### 5.12 `test/.gitkeep` — la carpeta vacía

**Qué es.** Un fichero sin contenido, cuyo único propósito es que Git registre la carpeta `test/`.

**El concepto: por qué se pone un fichero vacío.** Git no versiona carpetas, solo ficheros. Si la
carpeta `test/` estuviera vacía, no se añadiría al repositorio. El `.gitkeep` es la convención
para "esta carpeta existe y es intencional". Es un detalle de Git, no de ROS.

**Relación con el resto.** El `CMakeLists.txt` instala la carpeta `test/`, así que el sistema de
build espera que exista. Y el `package.xml` declara `ament_lint_auto` y `ament_lint_common` como
dependencias de test, que son los paquetes de *linter* de ROS 2 (comprobaciones de estilo:
copyright, formato, etc.). Sin embargo, **no hay ningún bloque `if(BUILD_TESTING)` en el
`CMakeLists.txt`**, así que esos linters nunca se ejecutan automáticamente. Es decir, el paquete
tiene la infraestructura de tests declarada pero no activa.

---

## 6. El flujo completo de arranque

### 6.1 Los cinco modos, con su comando exacto

| Modo | Comando | Qué processes aparecen |
|---|---|---|
| Simulación simple | `ros2 launch staubli_bringup launch_gazebo.launch.py` | Gazebo (sin ventana), robot, 3 controladores, 2 puentes |
| Simulación + GUI | `ros2 launch staubli_bringup launch_gazebo_gui.launch.py` | Todo lo anterior + ventana de Gazebo + GUI + servo del joystick |
| Control sin nada | `ros2 launch staubli_bringup launch_robot_control.launch.py` | `ros2_control_node`, 2 controladores, `robot_state_publisher`, RViz |
| Control + MoveIt | `ros2 launch staubli_bringup launch_demo.launch.py` | Lo anterior + `move_group` + RViz con panel de planificación |
| Robot real | `ros2 launch staubli_bringup launch_robot_control.launch.py use_mock_hardware:=false robot_ip:=<IP> start_controller:=joint_trajectory_controller` | Igual que "control sin nada", pero con el driver real |

### 6.2 Qué proceso arranca cada launch, en detalle

Aquí está el mapa nodo por nodo de cada launch, que es lo que hay que memorizar:

**`launch_gazebo.launch.py`** arranca 8 o 9 procesos:

1. `robot_state_publisher` — publica el URDF con el plugin de Gazebo.
2. `GzServer` — el simulador, con `inspection_cell.sdf`.
3. `parameter_bridge` (reloj) — `/clock` de Gazebo a ROS.
4. `parameter_bridge` (cámara) — imagen de la cámara a ROS.
5. `ros_gz_sim create` — inserta el modelo en Gazebo desde el URDF.
6. `spawner` de `joint_state_broadcaster` — empieza a publicar estados.
7. `spawner` de `joint_trajectory_controller` — ejecuta trayectorias.
8. `spawner` de `forward_velocity_controller` — cargado pero **inactivo**.
9. `gz sim -g` — la ventana, solo con `gui:=true`.

**`launch_robot_control.launch.py`** arranca 4 o 5:

1. `robot_state_publisher`.
2. `ros2_control_node` — el `controller_manager`.
3. `spawner` de `joint_state_broadcaster`.
4. `spawner` de `joint_trajectory_controller` (inactivo, se activa si se pide).
5. `rviz2` — solo con `gui:=true`.

**`launch_moveit.launch.py`** arranca 2 o 3:

1. `wait_for_robot_ready.py` — el guardián, que termina al recibir el URDF y los estados.
2. `move_group` — el planificador.
3. `rviz2` y/o `servo_node` — opcionales.

**`launch_demo.launch.py`** no arranca nada propio: incluye los otros dos.

**`launch_gazebo_gui.launch.py`** tampoco: incluye Gazebo y la GUI.

### 6.3 Cómo se comunican entre sí

Los procesos de un mismo launch no se hablan directamente entre ellos. Se comunican **a través de
topics, servicios y el grafo de ROS 2**, y esa es la libertad que hace que un launch se pueda
recomponer. Las conexiones que se dan en este paquete son:

**El URDF es el eje de todo.** El launch lo genera una vez con `xacro` y lo pasa como parámetro a
`robot_state_publisher`, que lo publica en `/robot_description`. A partir de ahí, **cualquier otro
proceso puede leerlo**: el `create` de Gazebo, RViz, MoveIt. Y es `robot_state_publisher` quien
traduce `/joint_states` en el árbol de transforms `/tf`, que es lo que permite que todos vean el
robot en la misma posición. Esta es la comunicación más importante del sistema, y es
*publish-subscribe*: nadie llama a nadie.

**El `controller_manager` se comunica por servicios.** Los `spawner` no escriben en un topic: llaman
a un servicio del `controller_manager` pidiendo que cargue o active un controlador. Por eso un
`spawner` es un proceso que termina cuando su trabajo está hecho, y por eso se lanzan varios a la
vez.

**Gazebo y ROS 2 se comunican por puentes.** `parameter_bridge` traduce topics entre los dos
transportes: el reloj y la cámara. Sin el puente del reloj, los nodos ROS no estarían
sincronizados con la física.

**El plugin conecta Gazebo y `ros2_control` por dentro.** En el modo Gazebo, el
`controller_manager` vive dentro del simulador (lo crea el plugin), y los `spawner` se conectan a
él por servicio. No hay ningún proceso de `ros2_control` visible en este modo: es la diferencia
más contraintuitiva de todo el proyecto.

**MoveIt se conecta por topics y actions.** Recibe objetivos de planificación (actions de
`moveit_msgs`), devuelve trayectorias, y cuando se ejecuta, envía la trayectoria al
`joint_trajectory_controller` a través de su action server. Ese controlador es el puente final
entre la planificación y el movimiento.

### 6.4 El flujo de datos completo, de principio a fin

Para el modo más completo (el que usa `launch_demo`, es decir, control + MoveIt):

```
  1. config/<modelo>/*.yaml  +  urdf/*.xacro  +  r2c/*.xacro
                    |
                    |  xacro  (con use_mock_hardware y robot_ip)
                    v
  2. URDF completo (geometria + <ros2_control> + <gazebo>)
                    |
                    |  como parametro
                    v
  3. robot_state_publisher  ----------> /robot_description   (durable)
                    |                          |
                    |                          +--> RViz: dibuja el robot
                    |                          +--> (Gazebo create, si aplica)
                    |                          +--> move_group: lo lee
                    v
  4. /joint_states  -->  /tf  (arbol de transforms)
                    |
                    v
  5. controller_manager  (creado por ros2_control_node o por el plugin de Gazebo)
                    |
                    |  servicios
                    v
  6. joint_trajectory_controller  <-- action follow_joint_trajectory
                    |                          ^
                    v                          |
  7. interfaces de posicion          move_group planifica y ejecuta
                    |                          ^
                    v                          |
  8. el robot (memoria, UDP, o Gazebo)  RViz (panel MotionPlanning)
```

Y el arranque de todo esto, en el orden en que ocurre:

```
  ros2 launch staubli_bringup launch_demo.launch.py robot_model:=tx2_60l
     |
     +--> incluye launch_robot_control  (gui:=false)
     |        |
     |        +--> xacro genera el URDF            [instante 0]
     |        +--> robot_state_publisher            [arranca]
     |        +--> ros2_control_node (CM)           [arranca]
     |        +--> spawner joint_state_broadcaster   [carga y activa]
     |        +--> spawner joint_trajectory          [carga, activa]
     |
     +--> incluye launch_moveit  (gui:=true)
              |
              +--> wait_for_robot_ready.py            [espera URDF + estados]
              |        (cuando los recibe, termina)
              v
              +--> move_group                        [arranca con toda la config]
              +--> rviz2 con panel MotionPlanning    [se conecta y dibuja]
```

El orden importa, y hay dos mecanismos que lo garantizan: el **`xacro` se ejecuta antes de que
arranque nada** (porque el URDF se genera como un comando, no como un proceso vivo), y el
**`OnProcessExit`** garantiza que MoveIt no arranque hasta que el robot está listo.

### 6.5 La decisión clave: mock, Gazebo o robot real

Toda la diferencia entre los modos está en los argumentos que se le pasan al Xacro. Esta es la
tabla que resume el corazón del proyecto:

| Argumento del Xacro | `use_mock_hardware` | `use_gazebo_hardware` | Plugin de hardware resultante |
|---|---|---|---|
| Simulación simple (`launch_robot_control` por defecto) | `true` | `false` | Ficticio (no hay física) |
| Robot real (`use_mock_hardware:=false robot_ip:=<IP>`) | `false` | `false` | El driver real (UDP) |
| Gazebo (`launch_gazebo`) | `false` | `true` | El plugin `gz_ros2_control` (dentro de Gazebo) |

Los tres modos se seleccionan, por tanto, **sin cambiar de launch** (salvo Gazebo, que tiene el suyo
porque necesita el simulador). Es un diseño bastante elegante: el launch aporta los procesos
(instaladores, planificador, visualizador) y el URDF aporta la personalidad del hardware.

---

## 7. Resumen final

`staubli_bringup` es la capa de **integración** del proyecto. No implementa nada del robot: su
trabajo es decidir **qué procesos existen, con qué configuración, en qué orden y con qué
argumentos**, y eso lo hace con cinco launch files que, combinados con `IncludeLaunchDescription`
y `OnProcessExit`, dan flexibilidad para cubrir desde la simulación completa hasta el robot real.

Las cinco ideas que conviene separar bien, porque se confunden constantemente:

- **Gazebo** es el único que **simula** física, y el robot se le carga desde el topic
  `/robot_description` (no está en el fichero del mundo).
- **`ros2_control`** es la capa que hace al robot programable, y **no simula por sí mismo**: con el
  plugin ficticio solo mueve números en memoria. Su `controller_manager` **lo crea el proceso
  `ros2_control_node`**, salvo en Gazebo, donde lo crea el plugin dentro del simulador.
- **RViz** solo dibuja, y nunca se lanza solo: necesita el URDF y los transforms de
  `robot_state_publisher`. Hay dos configuraciones distintas según haya MoveIt o no.
- **MoveIt** solo **planifica**: produce trayectorias, no las mueve. Para moverlas hace falta un
  controlador de `ros2_control`, que es lo que declara
  `staubli_moveit_config/moveit_controllers.yaml`.
- **El robot real** no es un launch aparte: es el mismo `launch_robot_control` con
  `use_mock_hardware:=false` y una `robot_ip`. La diferencia entre simular y hablar con la máquina
  real está en un argumento del Xacro, no en el launch.

Y el otro gran aprendizaje del paquete es el patrón de **espera y encadenamiento**: el script
`wait_for_robot_ready.py` no hace nada útil salvo esperar a que existan el URDF y los estados, y
eso es justamente lo que permite que MoveIt arranque en el momento justo. Es la solución a un
problema muy común en ROS 2, y ver un launch que la usa enseñado así (esperar a un proceso, y
lanzar el siguiente cuando termina) es probablemente lo más útil que se puede aprender de esta
carpeta.
