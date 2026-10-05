# Explicación del paquete `staubli_trajectory`

> Documento de estudio del paquete `staubli_trajectory` del workspace
> `staubli_ws`, escrito leyendo el código fuente y **ejecutándolo** para
> comprobar cada afirmación.
>
> Todo lo que sigue está verificado contra el código real. Cuando algo no se
> puede comprobar dentro de este repositorio, se dice explícitamente
> ("Esto se comprueba en...") en lugar de suponerlo.

---

## 1. Qué es `staubli_trajectory` y qué papel tiene en el proyecto

`staubli_trajectory` es el paquete que convierte **puntos cartesianos escritos
en un YAML** en **ángulos de articulación** y, opcionalmente, los envía al
controlador del robot.

Su problema que resuelve: el controlador del robot
(`joint_trajectory_controller`) **solo entiende ángulos**, nunca coordenadas
cartesianas. Un operario piensa "lleva la herramienta a (0, 0.55, 0.38) metros",
pero el hardware necesita saber "gira joint_1 a 1.53 rad, joint_2 a 0.44 rad…".
Esa conversión de "posición en el espacio" a "ángulos de las ruedas" es la
**cinemática inversa** (IK), y este paquete es una implementación pequeña y
directa de ella.

La cadena completa es:

```
config/points.yaml
        │  (posiciones cartesianas del TCP + orientación + tiempos)
        ▼
  PointsConfig            ← capa 1: QUÉ puntos visitar
        │
        ▼
  TrajectoryBuilder       ← capa 2: CÓMO viajar (cartesiano → articulaciones)
        │     │
        │     └── Tx260lKinematics  (FK/IK con PyKDL)
        │
        ▼
  trajectory_msgs/JointTrajectory   ← resultado: q1..q6 con tiempos
        │
        ▼ (solo con --send)
  JointTrajectoryClient  ← capa 3: ENVÍO al controlador
        │
        ▼
  /joint_trajectory_controller/follow_joint_trajectory
```

La decisión de diseño más importante del paquete es que **por defecto no mueve
nada**. Al arrancar sin argumentos, el nodo solo calcula e imprime un informe en
terminal. El movimiento físico exige pasar `--send` explícitamente. Esto se
comprueba en `trajectory_node.py:121-125`:

```python
if not self._send:
    self.get_logger().info(
        "Modo simulación (sin envío). Usa '--send' para mover el robot."
    )
    return plan.all_solved
```

### 1.1 Dónde encaja dentro del proyecto

Este paquete es una **alternativa ligera a MoveIt**, no un sustituto completo:

| Necesitas… | Usa |
|---|---|
| Un barrido de puntos sobre una pieza, con cálculo e impresión, sin dependencias pesadas | **`staubli_trajectory`** (este paquete) |
| Planificación con colisiones, rutas y órdenes arbitrarias, escena 3D | `staubli_moveit_config` + `move_group` |

El propio `package.xml:7-10` lo declara explícitamente:

```xml
Trayectorias cartesianas configurables para el Staubli TX2-60L sin MoveIt:
lee config/points.yaml, resuelve la IK del TCP tool0 (frame base_link) con
PyKDL y envía las articulaciones al joint_trajectory_controller mediante
/joint_trajectory_controller/follow_joint_trajectory.
```

Además, **otros dos paquetes del proyecto dependen de este** y reutilizan su
código (se detalla en la sección 12):

- `staubli_gui` importa `PointsConfig`, `TrajectoryBuilder` y
  `Tx260lKinematics` (`staubli_gui/staubli_gui/gui.py:1238,1534,1919-1920`).
- `staubli_joystick_servo` importa `Tx260lKinematics` y
  `JOINT_VELOCITY_LIMITS_RAD` (`staubli_joystick_servo/staubli_joystick_servo/servo_node.py:36`).

Es decir, este paquete es también la **fuente única de verdad de la geometría
del robot** para el resto del proyecto.

---

## 2. Estructura de la carpeta

```
staubli_trajectory/
├── config/
│   └── points.yaml                  ← LA ÚNICA ZONA A EDITAR por el usuario
├── launch/
│   └── trajectory.launch.py         ← arranque con `ros2 launch`
├── resource/
│   └── staubli_trajectory           ← marcador vacío (resource_index de ament)
├── staubli_trajectory/              ← el código Python
│   ├── __init__.py                  ← una línea de docstring
│   ├── points_config.py             ← capa 1: carga y valida el YAML
│   ├── tx2_60l_ik.py                ← modelo cinemático (FK/IK con PyKDL)
│   ├── trajectory_builder.py        ← capa 2: cartesiano → JointTrajectory
│   ├── joint_trajectory_client.py   ← capa 3: envío al controlador
│   └── trajectory_node.py           ← el nodo ROS 2 que orquesta todo
├── package.xml                      ← metadatos y dependencias
├── setup.py                         ← instalación (entry point)
├── setup.cfg                        ← dónde se instalan los ejecutables
└── README_EXPLICADO.md              ← este documento
```

### 2.1 Observaciones importantes sobre la estructura

**No existe `CMakeLists.txt`.** El paquete es de tipo `ament_python`
(python puro), declarado en `package.xml:31-33`:

```xml
<export>
  <build_type>ament_python</build_type>
</export>
```

y construido con `buildtool_ament_python` (`package.xml:15`). Esto implica:

- No hay C++, no hay `find_package`, no hay `ament_target_dependencies`.
- La instalación la hace `setuptools` mediante `setup.py`.
- El ejecutable **no** es un script suelto en `scripts/`; es un *entry point*
  declarado en `setup.py:41-45`:

  ```python
  entry_points={
      "console_scripts": [
          "trajectory_node = staubli_trajectory.trajectory_node:main",
      ],
  },
  ```

  Esto coincide con el comentario de `setup.py:9-11`: "genera el ejecutable
  'trajectory_node', de modo que se lanza con `ros2 run staubli_trajectory
  trajectory_node` igual que antes (sin `scripts/`)". Si buscas un fichero
  ejecutable con un `#!/usr/bin/env python3` en la carpeta, **no lo
  encontrarás**: no existe.

**No existe carpeta `test/`.** Aunque `package.xml:26-29` declara
`test_depend` de `ament_copyright`, `ament_flake8`, `ament_pep257` y
`python3-pytest`, y `setup.py:40` declara `tests_require=["pytest"]`, **no hay
ningún test escrito en este paquete**. Los `test_depend` están ahí por
herencia de la plantilla estándar de `ros2 pkg create`. Esto se comprueba en
`staubli_joystick_servo/test/conftest.py:19` y `staubli_gui/test/conftest.py:20`,
que **añaden con** `sys.path` el código fuente de este paquete hermano para
poder testearlo: no hay que instalar nada para que sus tests lo importen.

**`resource/staubli_trajectory` está vacío (0 bytes).** Es lo correcto: es un
marcador cuya *existencia* vale, no su contenido. `setup.py:26` lo instala en
`share/ament_index/resource_index/packages` para que `colcon` lo registre en el
índice de paquetes.

**`__init__.py` contiene una sola línea:**

```python
"""Paquete 'staubli_trajectory': control del TX2-60L sin MoveIt."""
```

No exporta nada. Los imports entre módulos son **absolutos y explícitos** desde
los hermanos (`from .points_config import PointsConfig`), y desde fuera son
absolutos (`from staubli_trajectory.tx2_60l_ik import Tx260lKinematics`).

### 2.2 Qué instala el paquete

`setup.py:25-30` define cuatro carpetas de destino:

```python
data_files=[
    ("share/ament_index/resource_index/packages", ["resource/" + PACKAGE_NAME]),
    ("share/" + PACKAGE_NAME, ["package.xml"]),
    (os.path.join("share", PACKAGE_NAME, "config"), glob("config/*.yaml")),
    (os.path.join("share", PACKAGE_NAME, "launch"), glob("launch/*.launch.py")),
],
```

| Origen | Destino tras compilar | Para qué |
|---|---|---|
| `resource/staubli_trajectory` | `share/ament_index/resource_index/packages/` | registro en el índice de ament |
| `package.xml` | `share/staubli_trajectory/` | metadatos |
| `config/*.yaml` | `share/staubli_trajectory/config/` | que el nodo encuentre el YAML en ejecución |
| `launch/*.launch.py` | `share/staubli_trajectory/launch/` | que `ros2 launch` lo encuentre |

Ese `share/.../config/points.yaml` es exactamente lo que consulta
`_default_config_path()` (`trajectory_node.py:36-39`) mediante
`get_package_share_directory("staubli_trajectory")`. **Ese es el motivo de que
`config/` se instale**: sin `data_files`, el nodo no encontraría su
configuración tras compilar.

`setup.cfg` (6 líneas) fija dónde acaban los ejecutables:

```ini
[develop]
script_dir=$base/lib/staubli_trajectory

[install]
install_scripts=$base/lib/staubli_trajectory
```

Es decir, el ejecutable `trajectory_node` acaba en
`install/staubli_trajectory/lib/staubli_trajectory/trajectory_node`, que es la
ruta que `ros2 run` busca.

---

## 3. Conceptos que hay que entender antes de seguir

Esta sección explica la teoría **antes** de aplicarla al código, porque el
paquete usa vocabulario concreto (TCP, tool0, base_link, cadena cinemática,
semillas) que solo se entiende con estos conceptos delante.

### 3.1 Marco de referencia (`frame`)

Un **frame** es un sistema de coordenadas: origen + orientación, más 3 ejes.
`base_link` es el frame fijo en la base del robot. `tool0` (o **TCP**, *Tool
Center Point*) es el frame en la punta de la herramienta, donde se considera
que está el punto de trabajo.

En este paquete, **todas las coordenadas cartesianas se expresan en `base_link`**
y **todas las posiciones articulares se expresan en radianes**. La cadena
cinemática va de uno a otro:

```
base_link ──joint_1── joint_2 ──joint_3── joint_4 ──joint_5── joint_6── link_6 ──tool0
   fijo        rota    rota        rota        rota        rota        rota      fijo
                     (todo este recorrido = cinemática directa)
```

### 3.2 Cinemática directa (FK) e inversa (IK)

**FK (Forward Kinematics):** dadas las ángulos de las articulaciones, ¿dónde
queda la herramienta? Es un problema **directo y único**: una entrada, una
salida. Se calcula multiplicando transformadas homogéneas:

```
T_total = T_1 · T_2 · T_3 · T_4 · T_5 · T_6 · T_tool0
```

donde `T_i = Trans(origen_i) · Rot(eje_i, q_i)`.

**IK (Inverse Kinematics):** dada la posición y orientación deseada de la
herramienta, ¿qué ángulos hay que poner? Es un problema **no lineal y con
múltiples soluciones**.

El multiplicidad es real y muy importante aquí: un robot de 6 grados de libertad
(normalmente un brazo *redundante*, porque 6 es el mínimo para posicionar y
orientar en el espacio) puede tener **varias configuraciones distintas que
llevan el TCP al mismo punto**. Además, algunas de esas soluciones están
**fuera de los límites físicos** de la articulación.

Por eso el solver de este paquete no acepta "la primera solución que encuentre",
sino que prueba **muchas** y elige la mejor. Se detalla en la sección 6.4.

### 3.3 Límites articulares y por qué hay que comprobarlos

Cada articulación real tiene topes mecánicos. Los del TX2-60L están en
`staubli_robot_description/config/tx2_60l/joint_limits.yaml` y se pueden leer
en grados (el fichero usa el tag YAML `!degrees`):

| Articulación | Límite (grados) | Límite (radianes) |
|---|---|---|
| `joint_1` | ±180° | ±3.1416 |
| `joint_2` | ±127° | ±2.2163 |
| `joint_3` | ±150° | ±2.6180 |
| `joint_4` | ±270° | ±4.7124 |
| `joint_5` | ±120° | ±2.0944 |
| `joint_6` | ±270° | ±4.7124 |

Una solución de IK que respete la geometría pero no los topes **es
físicamente inalcanzable**: el robot no podría llegar ahí aunque la matemática
cuadre. Comprobado ejecutando el código real:

```
check_joint_limits([3.15, 0,0,0,0,0]) = (False, [False, True, True, True, True, True])
check_joint_limits([7.0,  0,0,0,0,0]) = (False, [False, True, True, True, True, True])
```

`3.15 rad = 180.4°` ya supera el tope de `joint_1` (±180° = ±3.14159 rad), y
`7.0 rad = 401°` lo excede ampliamente. La comprobación usa un margen de
`1e-6` rad (`tx2_60l_ik.py:300`) para tolerar errores de punto flotante.

### 3.4 Quaternion `(x, y, z, w)` y orden de componentes

La orientación se representa con un **quaternion**, no con ángulos de Euler.
Esto se comprueba en el convenio de todo el stack:

- `points.yaml` acepta `quaternion_xyzw` — el orden es explícito en el nombre.
- ROS 2 (`geometry_msgs/Quaternion`) usa `(x, y, z, w)`.
- PyKDL (`Frame.M.GetQuaternion()`) también devuelve `(x, y, z, w)`.
- El código lo reutiliza tal cual, sin permutar: `tx2_60l_ik.py:345` hace
  `return list(frame.M.GetQuaternion())` y lo etiqueta como `(x, y, z, w)`.

Y para **leer** un quaternion escrito por el usuario, `tx2_60l_ik.py:338` lo
pasa a rotación con el orden correcto:

```python
q = kdl.Rotation.Quaternion(*[float(v) for v in quaternion_xyzw])
```

**Este detalle importa mucho**: si se invirtiera el orden, la orientación sería
incorrecta y el robot apuntaría en una dirección absurda sin ningún error
visible. Al usar el mismo orden en entrada y salida, el viaje de ida y vuelta es
consistente.

### 3.5 Los mensajes que viajan por ROS

Tres mensajes son esenciales en este paquete:

| Mensaje | Paquete | Qué transporta |
|---|---|---|
| `JointTrajectory` | `trajectory_msgs` | **La trayectoria**: nombres de articulaciones + lista de puntos con posiciones y tiempos |
| `JointTrajectoryPoint` | `trajectory_msgs` | Un punto: `positions`, `time_from_start` |
| `FollowJointTrajectory.Goal` | `control_msgs/action` | Envuelve la trayectoria + tolerancias |

`time_from_start` es un **instante absoluto desde el inicio del movimiento**, no
un intervalo. Si los puntos están a 0 s, 5 s, 10 s y 15 s, el robot debe estar en
el primero al arrancar, en el segundo 5 segundos después, y así. El
controlador **interpola** entre ellos.

### 3.6 `FollowJointTrajectory`: la acción y su ciclo de vida

Una **acción** en ROS 2 es un patrón de comunicación con **ciclo de vida
completo**, a diferencia de un topic (que solo publica sin confirmar nada):

```
Cliente                                  Servidor (el controlador)
   │                                              │
   │── send_goal_async(goal) ────────────────────►│
   │                                              │  (acepta o rechaza)
   │◄────────────── goal_handle.aceptado ────────│
   │                                              │
   │  ┌── feedback (repetido durante la ejecución)│
   │◄─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ┤
   │                                              │
   │◄────────────── resultado final ──────────────│
```

Tres fases, y el código las maneja explícitamente
(`joint_trajectory_client.py:80-113`):

1. **Envío y aceptación** del goal. El servidor puede *rechazarlo*
   (`accepted == False`).
2. **Ejecución con feedback** opcional mientras el robot se mueve.
3. **Resultado final** con un código: `SUCCESSFUL`, `INVALID_GOAL`,
   `INVALID_JOINTS`, `PATH_TOLERANCE_VIOLATED`, `GOAL_TOLERANCE_VIOLATED`, etc.

**El resultado importa**: recibir un resultado no significa éxito. Significa
"terminó, y este es el veredicto". El código comprueba
`error_code == SUCCESSFUL` y trata todo lo demás como error
(`joint_trajectory_client.py:105-113`).

### 3.7 `JointTrajectoryController` y el `controller_manager`

El `controller_manager` es un servicio central que carga y coordina
"controladores", cada uno de los cuales maneja un conjunto de articulaciones por
un tipo de interfaz. Está configurado en
`staubli_bringup/config/controllers.yaml` con tres:

| Controlador | Tipo | Interfaz | Para qué |
|---|---|---|---|
| `joint_state_broadcaster` | `joint_state_broadcaster/JointStateBroadcaster` | estado | Publica `/joint_states` |
| `joint_trajectory_controller` | `joint_trajectory_controller/JointTrajectoryController` | **posición** | Sigue trayectorias (lo usa este paquete) |
| `forward_velocity_controller` | `forward_command_controller/ForwardCommandController` | velocidad | Mando manual por joystick (lo usa `staubli_joystick_servo`) |

Detalle crítico de la configuración (`controllers.yaml`):

```yaml
joint_trajectory_controller:
  ros__parameters:
    joints: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]
    command_interfaces: [position]
```

Dos consecuencias directas:

1. **El orden de las articulaciones importa.** El controlador de la lista es el
   que espera. `points_config.py:54-58` lo valida estrictamente y aborta si no
   coincide:

   ```python
   self.joint_names: List[str] = raw.get("joint_names", [])
   if self.joint_names != EXPECTED_JOINTS:
       raise ValueError(
           f"'joint_names' debe ser {EXPECTED_JOINTS} "
           f"(orden del joint_trajectory_controller). Recibido: {self.joint_names}"
       )
   ```

   Si el YAML dijera `[joint_2, joint_1, …]`, el cálculo se haría bien pero el
   robot se movería de forma **catastróficamente incorrecta**, porque
   `JointTrajectory` es posicional: el primer valor de `positions` va al primer
   nombre de `joint_names`. Por eso se valida en vez de confiar.

2. **Solo acepta posiciones, en radianes.** `command_interfaces: [position]`
   significa que este controlador no acepta comandos de velocidad ni de
   esfuerzo. Y como las unidades del URDF son radianes (ROS 2 lo exige), todos
   los números articulares del paquete son radianes.

Hay además dos parámetros que afectan al comportamiento real y que conviene
conocer:

```yaml
allow_partial_joints_goal: false      # un goal debe cubrir las 6 articulaciones
interpolate_from_desired_state: true  # parte del estado actual, no del último comando
```

`interpolate_from_desired_state: true` importa: al activar el controlador, la
primera interpolación arranca **desde la posición real medida**, no desde el
último valor commanded. Evita un salto al conmutar desde el control manual.

### 3.8 PyKDL, y por qué aquí no se usa MoveIt

`orocos_kdl_vendor` es la implementación de **KDL (Kinematics and Dynamics
Library)**, una biblioteca C++ clásica de cinemática. `PyKDL` es su enlace a
Python. Este paquete lo usa **directamente**, sin capas por encima, por dos
motivos:

- Es ligero: no arrastra MoveIt, ni su Grupo de planificación, ni su modelo de
  escena, ni `rviz` con plugins.
- Da control explícito sobre la elección de la rama de la IK (ver §6.4), que
  `move_group` abstrae.

Los objetos de PyKDL usados aquí:

| Objeto | Qué es |
|---|---|
| `Vector` | Vector de 3 componentes (posición o dirección) |
| `Rotation` | Matriz de rotación 3×3 (o cuaternión) |
| `Frame` | `Rotation` + `Vector`: pose completa (posición **y** orientación) |
| `Chain` | Cadena cinemática: lista ordenada de segmentos |
| `Segment` | Eslabón: un joint + una transformada fija |
| `Joint` | Articulación: nombre, origen, eje, tipo de movimiento |
| `JntArray` | Vector de valores de articulación (aquí, 6) |
| `ChainFkSolverPos_recursive` | Solver de cinemática **directa** |
| `ChainIkSolverPos_LMA` | Solver de cinemática **inversa** (Levenberg-Marquardt) |
| `ChainJntToJacSolver` | Calcula el **Jacobiano** (matriz 6×6 de velocidades) |

**`LMA`** significa *Levenberg-Marquardt Algorithm*: un método iterativo de
optimización que minimiza el error entre la pose actual y la pose deseada,
ajustando las 6 articulaciones a la vez. Se instancia en `tx2_60l_ik.py:177`:

```python
self._ik = kdl.ChainIkSolverPos_LMA(self._chain, 1.0e-6, 500)
```

Los dos parámetros son la **tolerancia** (`1e-6`) y el **máximo de iteraciones**
(`500`).

Una advertencia conceptual importante: LMA es un método **local**. Empieza en un
punto y busca un mínimo cercano. Si el punto de partida está en una "cuenca"
equivocada, converge a un mínimo local que **no** es la solución — o no converge.
Ese es exactamente el motivo del arranque multsemilla (§6.4).

---

## 4. `config/points.yaml` — la única zona que edita el usuario

Es el contrato de entrada del paquete. El propio fichero lo declara en su
cabecera (`points.yaml:2`):

```
#  CONFIGURACIÓN DE LOS PUNTOS DE LA TRAYECTORIA  (ÚNICA ZONA A EDITAR)
```

### 4.1 Contenido real y qué significa cada clave

```yaml
joint_names:
  - joint_1
  - joint_2
  - joint_3
  - joint_4
  - joint_5
  - joint_6

point_type: cartesian
planning_frame: base_link
orientation_mode: look_at_part_center
part_center_base_link: [0.0, 0.66, 0.24]

points:
  - name: inspection_fronto_superior
    position: [0.0, 0.55, 0.38]
    time_from_start: 0.0
  - name: inspection_izquierda
    position: [-0.22, 0.48, 0.36]
    time_from_start: 5.0
  - name: inspection_derecha
    position: [0.22, 0.48, 0.36]
    time_from_start: 10.0
  - name: home
    positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    time_from_start: 15.0

random_seed: 42
n_random_seeds: 30

goal_tolerance:
  position: 0.01
  velocity: 0.05
  acceleration: 0.1
goal_time_tolerance: 2.5
```

| Clave | Significado | Validada en |
|---|---|---|
| `joint_names` | Orden de las 6 articulaciones. **Debe ser exactamente** `joint_1..joint_6` | `points_config.py:54-58` (error si no) |
| `point_type` | `cartesian` (XYZ + IK) o `joint` (ángulos directos) | `points_config.py:61-64` |
| `planning_frame` | Frame de las posiciones. Se **imprime**, pero no se usa para transformar | `points_config.py:66` |
| `orientation_mode` | `manual` o `look_at_part_center` | `points_config.py:69-74` |
| `part_center_base_link` | Centro de la pieza, en `base_link` | `points_config.py:75-78` |
| `points` | Lista de puntos. **Cualquier número** (debe ser ≥1) | `points_config.py:92-95` |
| `random_seed` | Semilla del generador aleatorio (reproducibilidad) | `points_config.py:88` |
| `n_random_seeds` | Cuántas semillas aleatorias probar por punto | `points_config.py:89` |
| `goal_tolerance` | Tolerancias de posición (rad), velocidad (rad/s), aceleración (rad/s²) | `points_config.py:81-84` |
| `goal_time_tolerance` | Margen temporal del punto final (s) | `points_config.py:85` |

### 4.2 Los dos formatos de punto

Cada punto admite **dos formatos distintos**, y la validación los mantiene
mutuamente excluyentes (`points_config.py:119-122`):

```python
if pos is not None and jpos is not None:
    raise ValueError(
        f"{name}: no puede tener 'position' y 'positions' a la vez."
    )
```

- **`position: [x, y, z]`** → punto cartesiano. Se calcula la IK.
- **`positions: [q1..q6]`** → punto articular. Se usa tal cual, **sin IK**.

El fichero de ejemplo **mezcla ambos** a propósito: tres puntos de inspección
cartesianos y un `home` articular. Es un patrón muy útil —"ve a inspeccionar
estas tres poses, y luego vuelve a casa"— y funciona porque
`trajectory_builder.py:102-112` comprueba `if "positions" in point` **antes**
de intentar la cinemática inversa.

Verificado ejecutando el nodo real con este fichero: los tres puntos
cartesianos se resuelven con IK y el cuarto se marca
`articular (passthrough, sin IK)`.

### 4.3 `time_from_start`: instantes absolutos, no intervalos

La validación exige que sea **estrictamente creciente** (`points_config.py:109-113`):

```python
if previous_t is not None and t <= previous_t:
    raise ValueError(
        f"{name}: 'time_from_start' debe ser estrictamente creciente "
        f"({previous_t} -> {t})."
    )
```

Razones:

- Es un **instante absoluto** desde el inicio del movimiento, no un intervalo.
  El controlador interpola entre estos instantes.
- Si dos puntos tuvieran el mismo tiempo, la trayectoria sería degenerada (el
  robot tendría que estar en dos sitios a la vez).
- El **primer punto puede tener `time_from_start: 0.0`**, que es lo que hace
  el ejemplo: el robot debe estar en esa pose al arrancar el movimiento.

El fichero de ejemplo define una cadencia de 5 s entre puntos y 15 s hasta
`home`.

### 4.4 Los dos modos de orientación

**`manual`**: cada punto **debe** aportar `quaternion_xyzw`. Si falta, la
validación aborta (`points_config.py:133-137`):

```python
if self.orientation_mode == "manual" and q is None:
    raise ValueError(
        f"{name}: orientation_mode='manual' requiere 'quaternion_xyzw' "
        f"en cada punto."
    )
```

**`look_at_part_center`**: no hace falta escribir la orientación. El código
calcula una orientación cuyo **eje +Z de `tool0` apunta al centro de la pieza**.
Es el modo por defecto del fichero, y el que hace que "inspeccionar" tenga
sentido: la herramienta apunta a la pieza.

La configuración por defecto del proyecto es `look_at_part_center` con el
centro de la pieza en `[0.0, 0.66, 0.24]`. Ese valor **no es arbitrario**; el
fichero lo explica (`points.yaml:42-44`):

```
# Centro de la pieza en base_link: world (0.0, 0.66, 0.54) menos el offset
# world->base_link (0, 0, 0.30) del spawn de Gazebo => (0.0, 0.66, 0.24).
```

Y es correcto. Comprobado en los ficheros reales del proyecto:

- La pieza está en el mundo en `(0.00, 0.66, 0.54)`
  (`staubli_bringup/worlds/inspection_cell.sdf:245-247`).
- El robot se genera 0.30 m por encima del suelo
  (`staubli_bringup/launch/launch_gazebo.launch.py:146-151`, argumento
  `"-z", "0.30"`, para subirse a la plataforma).

Como `base_link` está 0.30 m por encima del suelo, la pieza queda en
`z = 0.54 − 0.30 = 0.24` dentro de `base_link`, y en `y = 0.66` (sin cambio).
De ahí `[0.0, 0.66, 0.24]`. Es un ejemplo perfecto de por qué las coordenadas
dependen del frame.

### 4.5 Sobre `planning_frame`: leer el código con honestidad

`planning_frame: base_link` **se lee, se valida y se imprime, pero no se usa
para transformar nada**. Verificado buscando todos sus usos en el paquete:

```
points_config.py:10   (docstring)
points_config.py:66   (asignación del atributo)
points_config.py:126  (mensaje de error)
trajectory_node.py:47 (impresión del informe)
trajectory_node.py:107(log de arranque)
```

No hay ni una sola llamada a `tf2`, ni a `buffer.transform`, ni a
`lookup_transform` en todo el paquete. **Todos los cálculos asumen `base_link`
de forma implícita**, porque la cadena de PyKDL está definida desde `base_link`.

Consecuencia práctica: cambiar `planning_frame` a otra cosa **no produciría un
error**, simplemente no transformaría las coordenadas y el cálculo sería
incorrecto. La clave está documentada y validada como excepción, pero el valor
no tiene efecto sobre el cálculo. Es algo que conviene saber antes de confiar en
ella.

### 4.6 El determinismo: `random_seed` y `n_random_seeds`

```yaml
random_seed: 42
n_random_seeds: 30
```

La IK prueba semillas aleatorias (§6.4), lo que normalmente haría el cálculo
**no determinista**. Fijando la semilla del generador, dos ejecuciones del
mismo YAML dan **exactamente el mismo resultado**. Es esencial para depurar y
para que un resultado "raro" sea reproducible.

---

## 5. `points_config.py` — capa 1: qué puntos visitar

### 5.1 Responsabilidad

```
CAPA 1 DE 3 (configuracion): define QUÉ puntos debe visitar tool0.
```

Solo dos tareas: **cargar** el YAML y **validarlo**. No calcula nada. Esto es
una separación de responsabilidades deliberada: si el YAML está mal, el error
sale antes de crear la cinemática, con un mensaje claro.

### 5.2 El patrón de "validar al cargar, no al usar"

Todo el constructor valida en cascada. Esto **convierte los errores de ejecución
en errores de arranque**: es mucho mejor que el fallo aparezca al calcular el
punto 47 de una lista de 100.

```python
def __init__(self, config_path: str) -> None:
    self.config_path: Path = Path(config_path)
    if not self.config_path.is_file():
        raise FileNotFoundError(
            f"Configuracion no encontrada: {self.config_path}"
        )

    raw: Dict[str, Any] = yaml.safe_load(self.config_path.read_text())
    if not isinstance(raw, dict):
        raise ValueError("El fichero de configuracion debe ser un YAML de diccionario.")
    ...
    self._validate()
```

Tres niveles de defensa:

1. **El fichero existe** (`is_file()`), con un error que incluye la ruta.
2. **El YAML es un diccionario**, no una lista o un escalar.
3. **El contenido es coherente** (delegado en `_validate()`).

### 5.3 El orden de `joint_names` es un contrato, no una sugerencia

Repetido de §3.7 porque es el punto donde un error sería más caro. El código
compara contra una constante y **aborta**:

```python
EXPECTED_JOINTS = [f"joint_{i}" for i in range(1, 7)]
...
if self.joint_names != EXPECTED_JOINTS:
    raise ValueError(
        f"'joint_names' debe ser {EXPECTED_JOINTS} "
        f"(orden del joint_trajectory_controller). Recibido: {self.joint_names}"
    )
```

La comparación es de **listas completas**, no de conjuntos: el orden importa.

### 5.4 `_validate()`: todas las reglas, punto por punto

El método recorre los puntos y comprueba:

**a) `time_from_start` presente y creciente**

```python
t = p.get("time_from_start")
if t is None:
    raise ValueError(f"{name}: falta 'time_from_start'.")
```

Sin valor por defecto: o está, o es error. Y recordemos que el primer punto
puede ser `0.0` pero no negativo sin que nada lo impida… salvo que el
controlador lo rechace después. Detalle fino: **la validación no comprueba que
el primer tiempo sea ≥ 0**, solo que la serie sea creciente.

**b) `position` y `positions` no pueden coexistir** (§4.2).

**c) Longitudes correctas**

- `position` → exactamente 3 valores (línea 124-127).
- `positions` → exactamente 6 valores (línea 139-142).
- `quaternion_xyzw` → exactamente 4 valores, o ausente (línea 128-132).

Los mensajes de error incluyen el **nombre del punto**, que es lo primero que
el usuario necesita ver. Además, si el punto no tiene `name`, se genera uno:

```python
name = p.get("name", f"punto_{i}")
```

**d) `orientation_mode='manual'` exige `quaternion_xyzw`** (§4.4).

**e) En modo `joint`, todos los puntos necesitan `positions`**

```python
elif self.point_type == "joint":
    positions = p.get("positions")
    if not isinstance(positions, list) or len(positions) != 6:
        raise ValueError(
            f"{name}: point_type='joint' requiere 'positions' con 6 valores."
        )
```

### 5.5 Los valores por defecto

Cada clave tiene un default sensato, de modo que un YAML mínimo funciona:

| Clave | Default en el código | Default en el fichero |
|---|---|---|
| `point_type` | `"cartesian"` | `cartesian` |
| `planning_frame` | `"base_link"` | `base_link` |
| `orientation_mode` | `"look_at_part_center"` | `look_at_part_center` |
| `part_center_base_link` | `[0.0, 0.66, 0.24]` | `[0.0, 0.66, 0.24]` |
| `goal_tolerance.position` | `0.01` | `0.01` |
| `goal_tolerance.velocity` | `0.05` | `0.05` |
| `goal_tolerance.acceleration` | `0.1` | `0.1` |
| `goal_time_tolerance` | `0.5` | `2.5` |
| `random_seed` | `42` | `42` |
| `n_random_seeds` | `30` | `30` |

**Única discrepancia**: `goal_time_tolerance`. El código usa `0.5` s como
default; el fichero fija `2.5` s. Esto es coherente con el resto del flujo
(tiempos de 5 s entre puntos), pero conviene saber que si se borra la clave del
YAML, el comportamiento cambia silenciosamente a 0.5 s.

Nótese también que `orientation_mode: manual` **no** es un valor por defecto
posible sin aportar quaternions: el modo por defecto está elegido
deliberadamente para que el caso de uso habitual (inspeccionar) funcione sin
escribir orientación.

---

## 6. `tx2_60l_ik.py` — el modelo cinemático

Es el archivo más denso (345 líneas) y el corazón matemático del paquete.

### 6.1 La fuente de la verdad: leer el robot, no reescribirlo

El principio de diseño, declarado en el docstring (`tx2_60l_ik.py:8-15`):

```
FUENTE DE LOS PARÁMETROS (sin duplicarlos):
  La geometría se lee en tiempo de ejecución de los ficheros oficiales del
  repositorio 'staubli_robot_description/config/tx2_60l/':
    - kinematics.yaml  -> origen (xyz) y eje (axis) de cada articulación
    - robot_model.yaml -> offset del TCP tool0 respecto a link_6
    - joint_limits.yaml-> límites de posición (via tag YAML !degrees)
  De este modo, cualquier cambio en el modelo del robot se refleja aquí sin
  editar este módulo.
```

**Esto es lo más importante que hay que entender del paquete.** Las
dimensiones del robot **no están escritas** en el código: se leen de los
ficheros de `staubli_robot_description` **en tiempo de ejecución**. Si mañana se
cambia el offset del TCP en el URDF del robot, este paquete **cambia
automáticamente**, sin tocar una línea de Python.

Comprobado: los valores que el código carga coinciden exactamente con los
YAML del repositorio.

```
ORIGINS [(0.0, 0.0, 0.21), (0.0, 0.11735, 0.165), (0.0, 0.00925, 0.4),
         (0.0, -0.1066, 0.088), (0.0, 0.0, 0.362), (0.0, 0.0, 0.0)]
AXES   [(0,0,1), (0,1,0), (0,1,0), (0,0,1), (0,1,0), (0,0,1)]
TOOL0  (0.0, 0.0, 0.07)
LIMITS deg: [(-180,180), (-127,127), (-150,150), (-270,270), (-120,120), (-270,270)]
VEL rad/s:   [5.236, 5.236, 5.236, 5.236, 5.236, 8.7266]
```

Ejes: `joint_1` gira sobre **Z** (eje vertical de la base), `joint_2` y
`joint_3` sobre **Y** (hombro y codo), `joint_4` sobre **Z** (muñeca),
`joint_5` sobre **Y**, `joint_6` sobre **Z**. Es la estructura típica de un
brazo de 6 ejes: base, hombro, codo, muñeca (3 rotaciones), muñeca (3 rotaciones).

### 6.2 Localizar los ficheros: dos estrategias

`find_tx2_60l_ik.py:60-86` implementa una búsqueda con **fallback** deliberado:

```python
def find_tx2_60l_config_dir() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory("staubli_robot_description"))
        if (share / "config" / "tx2_60l").is_dir():
            return share / "config" / "tx2_60l"
    except Exception:  # noqa: BLE001 - ament_index no disponible sin ROS fuenteado
        pass

    # Fallback: subimos de 'staubli_trajectory' hasta encontrar el repo.
    here = Path(__file__).resolve()
    for parent in here.parents:
        cand = parent / "staubli_robot_description" / "config" / "tx2_60l"
        if cand.is_dir():
            return cand

    raise FileNotFoundError(...)
```

1. **Share instalado**: donde `colcon` instaló `staubli_robot_description`.
2. **Árbol de fuentes**: sube desde `__file__` hasta encontrar el directorio
   hermano. Esto permite **ejecutar los tests sin compilar**, que es
   exactamente lo que hacen `staubli_gui/test/conftest.py:20` y
   `staubli_joystick_servo/test/conftest.py:19`.

Ese `except Exception` con `# noqa: BLE001` es correcto y no es descuido: si
`ament_index_python` falla porque ROS no está sourced, se quiere caer al
fallback, no propagar el error.

### 6.3 El tag YAML `!degrees`

`joint_limits.yaml` escribe los límites en grados con un tag propio:

```yaml
min_position: !degrees -180.0
max_velocity: !degrees 300.0
```

`yaml.safe_load` **rechazaría** ese tag (no es YAML estándar). El código lo
extiende (`tx2_60l_ik.py:44-52`):

```python
class _RobotYamlLoader(yaml.SafeLoader):
    """SafeLoader con soporte del tag `!degrees` usado en joint_limits.yaml."""

def _degrees_constructor(loader: yaml.Loader, node: yaml.Node) -> float:
    return math.radians(float(loader.construct_scalar(node)))

_RobotYamlLoader.add_constructor("!degrees", _degrees_constructor)
```

Extiende `SafeLoader` (no `Loader` a secas), así que **se mantienen todas las
protecciones contra YAML malicioso**. Y el constructor convierte a radianes en
el punto exacto de lectura. Verificado: `±180°` → `±3.1416 rad`,
`300°/s` → `5.236 rad/s`, `500°/s` → `8.7266 rad/s`.

### 6.4 La construcción de la cadena KDL: la parte delicada

Este es el punto más técnico del archivo, y su docstring lo explica con una
precisión notable (`tx2_60l_ik.py:17-25`):

```
NOTA de construcción de la cadena KDL:
  En URDF cada 'origin' es la transformada desde el link padre al marco del
  joint, y la rotación ocurre en ese marco: T_i = Trans(origin) · Rot(axis,q).
  La semántica nativa de PyKDL es: Segment::pose(q) = Joint::pose(q) * f_tip,
  donde el constructor de Segment reescribe f_tip = Joint::pose(0)^-1 * f_tip_dado.
  Para obtener exactamente Trans(origin) · Rot(axis,q) hay que declarar el
  origin EN EL JOINT (Joint(name, origin, axis, RotAxis)), lo que compensado
  por esa reescritura deja Segment::pose(q) = Trans(origin) · Rot(axis,q).
  Con q = todas-0 el tool0 cae en (0, 0.02, 1.295) m (verificado).
```

El problema: **PyKDL y URDF tienen semánticas distintas** para el `origin`, y
por eso el código declara el mismo `origin` **dos veces**: en el `Joint` y en
el `Frame` del `Segment` (`tx2_60l_ik.py:186-187`):

```python
joint = kdl.Joint(name, kdl.Vector(*origin), kdl.Vector(*axis), kdl.Joint.RotAxis)
chain.addSegment(kdl.Segment(joint, kdl.Frame(kdl.Vector(*origin))))
```

Parece redundante, y lo es **a propósito**: el constructor de `Segment`
reescribe `f_tip` internamente, y al poner el origin en el `Joint` se obtiene
exactamente la semántica del URDF.

**Y la validación empírica existe en el propio docstring**: "Con q = todas-0 el
tool0 cae en (0, 0.02, 1.295) m (verificado)". Ejecutado de verdad:

```
FK(q=0) p = [0.0, 0.02, 1.295]
```

Coincide. Y tiene sentido físico: con el brazo recto hacia arriba, la altura
es la suma de las alturas de los orígenes (`0.21 + 0.165 + 0.40 + 0.088 +
0.362 + 0 = 1.225`), más el offset del TCP (`0.07`) → `1.295 m`. Y la
desviación en `y` de 2 cm es la suma de los desplazamientos laterales
(`0.11735 + 0.00925 − 0.1066 = 0.02`). **La geometría está bien cargada.**

El `tool0` se añade como un segmento **fijo** (`tx2_60l_ik.py:189-191`):

```python
chain.addSegment(
    kdl.Segment(kdl.Joint("tool0", kdl.Joint.Fixed), kdl.Frame(kdl.Vector(*_TOOL0_OFFSET)))
)
```

`kdl.Joint.Fixed` significa que no aporta grados de libertad: es el desplazamiento
del centro de la herramienta respecto a `link_6`. La cadena resultante:

```
chain segments: 7 joints: 6
```

7 segmentos (6 articulaciones + 1 fijo), 6 grados de libertad.

### 6.5 La cinemática inversa: arranque multsemilla

Este es el corazón del solver (`tx2_60l_ik.py:224-281`) y lo que hace que la IK
sea fiable.

**El problema.** `ChainIkSolverPos_LMA` es un optimizador **local** (recordar
§3.3): parte de una configuración y busca un mínimo cercano. Para un robot
redundante, existen **muchas** soluciones válidas y algunos mínimos locales
"trampa". Con una sola semilla, el resultado es **arbitrario e impredecible**.

**La solución: probar muchas semillas y elegir bien.** El código genera
(`tx2_60l_ik.py:239-250`):

```python
seeds: List[List[float]] = [[0.0] * 6]
if q_state is not None:
    seeds.append([float(v) for v in q_state])
if q_prev is not None:
    seeds.append([float(v) for v in q_prev])
    for _ in range(10):
        seeds.append(
            [float(q_prev[i]) + self._rng.uniform(-0.05, 0.05) for i in range(6)]
        )
for _ in range(self._n_random_seeds):
    lo, hi = zip(*_JOINT_LIMITS_RAD)
    seeds.append([self._rng.uniform(lo[i], hi[i]) for i in range(6)])
```

Cuatro familias de semillas:

| Familia | Cuántas | Para qué |
|---|---|---|
| Ceros | 1 | Configuración neutra, punto de partida clásico |
| `q_state` | 1 (si no es `None`) | Estado real medido del robot: arrancar desde donde está |
| `q_prev` | 1 | Solución del punto anterior: **continuidad** |
| `q_prev` perturbado | 10 | Escapar del mínimo local en el que cayó `q_prev` (±0.05 rad) |
| Aleatorias | `n_random_seeds` (30) | Cobertura global del espacio de configuraciones |

Contadas para el uso real del paquete (`q_state=None` siempre, ver §8.2):
**31 semillas en el primer punto** y **42 en los siguientes**. Verificado
ejecutando el código.

**El filtro: validar cada candidato por FK inversa.** Cada semilla se resuelve
y luego **se comprueba con la cinemática directa** (`tx2_60l_ik.py:284-293`):

```python
def _check_solution(self, q: List[float], goal: kdl.Frame) -> IkResult:
    frame = self.forward_kinematics(q)
    res = IkResult(q=q)
    res.pos_error = (frame.p - goal.p).Norm()
    res.rot_error = abs((goal.M.Inverse() * frame.M).GetRotAngle()[0])
    res.solved = res.pos_error < _IK_POS_EPS and res.rot_error < _IK_ROT_EPS
    if res.solved:
        res.inside_limits, res.joints_inside = self.check_joint_limits(q)
    return res
```

Los errores de posición y rotación se calculan **independientemente del
solver**: se aplica FK a la solución y se compara con el objetivo. Es una
**verificación cruzada**: si el optimizador dice "resuelto" pero la FK no lo
confirma, se descarta.

Los umbrales (`tx2_60l_ik.py:142-143`):

```python
_IK_POS_EPS = 1.0e-4   # m     (0.1 mm)
_IK_ROT_EPS = 1.0e-3   # rad   (0.057°)
```

Son **muy estrictos** —0.1 mm—. Verificado con un round-trip FK→IK sobre 30
configuraciones aleatorias dentro de límites:

```
resueltas: 30 /30
max pos_error: 0.000e+00 m
dentro de limites: 30 /30
```

**El criterio de selección** (`tx2_60l_ik.py:279-281`):

```python
candidatos.sort(key=lambda r: (0 if r.inside_limits else 1, r.continuity))
return candidatos[0]
```

Ordena por **dos claves, en este orden**:

1. **`inside_limits`**: primero las soluciones dentro de los topes. Una
   solución fuera de límites es inútil aunque sea matemáticamente perfecta.
2. **`continuity`**: a igualdad de lo anterior, la **más cercana** al punto
   anterior. Esto evita que el robot "salte" de rama entre puntos
   consecutivos y haga un movimiento brusco, o incluso que se autochoque.

`continuity` es la distancia euclídea al cuadrado (`tx2_60l_ik.py:271`):

```python
res.continuity = sum((q[i] - float(q_prev[i])) ** 2 for i in range(6))
```

**La deduplicación de ramas** (`tx2_60l_ik.py:261-265`):

```python
clave = tuple(round(v, 3) for v in q)
if clave in vistas:
    continue
vistas.add(clave)
```

Redondea a 3 decimales para eliminar soluciones **numéricamente idénticas**:
distintas semillas que convergen a la misma configuración no cuentan como
candidatos diferentes. Sin esto, 42 semillas convergiendo a la misma solución
inflarían artificialmente el conjunto de candidatos.

**Cuando no hay solución**, se devuelve un `IkResult()` vacío (`tx2_60l_ik.py:276-277`),
que por defecto tiene `solved=False` y `pos_error=math.inf`. Verificado con un
punto inalcanzable en `(5, 5, 5)`: `solved=False pos_err=inf`. El `inf` que
aparece en el informe (`IK SIN SOLUCIÓN (pos inf m, rot inf rad)`) es
precisamente ese valor por defecto, no un error de cálculo.

### 6.6 El Jacobiano

```python
def jacobian(self, q: Sequence[float]) -> np.ndarray:
```

Devuelve la matriz **6×6** que relaciona velocidades articulares con velocidades
lineales y angulares del `tool0`, en `base_link`. Las filas son
`[vx, vy, vz, ωx, ωy, ωz]`; las columnas, `joint_1..joint_6`.

Para qué sirve: el Jacobiano indica **hacia dónde puede moverse la herramienta
para un movimiento dado de las articulaciones**. Si una fila fuera cero, ese
sentido de movimiento sería imposible en la posición actual (configuración
singular). Se resuelve sobre la **misma cadena** que FK/IK, incluyendo el offset
del TCP, y `ChainJntToJacSolver` se instancia una sola vez en el constructor
(`tx2_60l_ik.py:176`) para reutilizarla.

**Uso real en el proyecto**: `staubli_joystick_servo` importa esta clase para
su propio cálculo, aunque este paquete en sí no lo usa internamente para la
trayectoria.

### 6.7 Apuntar la herramienta: `orientation_frame_looking_at`

El cálculo de orientación del modo `look_at_part_center` está aquí
(`tx2_60l_ik.py:304-330`) y merece explicación aparte, porque construir un frame
"que mire a un punto" tiene un detalle sutil.

El objetivo: construir una rotación `R` tal que el **eje +Z de `tool0` apunte
desde la posición del punto hacia el centro de la pieza**.

```python
z = kdl.Vector(
    target[0] - position[0],
    target[1] - position[1],
    target[2] - position[2],
)
z = z * (1.0 / z.Norm())
aux = kdl.Vector(1, 0, 0) if abs(z[2]) > 0.99 else kdl.Vector(0, 0, 1)
y = aux * z                # cross(aux, z)
y = y * (1.0 / y.Norm())
x = y * z                  # cross(y, z)
return kdl.Frame(kdl.Rotation(x, y, z), kdl.Vector(*position))
```

Paso a paso:

1. **`z` = dirección de apuntado**: el vector del punto al objetivo,
   normalizado. Esto **define por completo el requisito**: `R · [0,0,1] = z`.
2. **`aux` = referente estable**: se elige `[1,0,0]` **solo si `z` es casi
   vertical** (`|z[2]| > 0.99`), si no `[0,0,1]`. Esto evita el
   **producto vectorial nulo**: si `aux` fuera paralelo a `z`, `y = aux × z`
   sería el vector cero y la normalización dividing por cero explotaría.
3. **`y` = `aux × z`**, normalizado.
4. **`x` = `y × z`**. Con esta elección, `(x, y, z)` es **directo** (no
   zurdo): `x × y = z`.
5. Se monta la `Frame` con esas columnas.

**El sub-producto de`.KDL` es `cross()`**: en PyKDL, `a * b` entre dos `Vector`
es el producto vectorial, no el escalar. Eso puede sorprender a quien lea el
código esperando multiplicación numérica. Aquí es intencionado.

**El giro alrededor de Z queda libre**: apuntar define **una** restricción
(la dirección de Z), no las tres. Queda un grado de libertad —el *roll* de la
herramienta alrededor de su propio eje de apuntado— que este código **fija de
forma determinista** eligiendo `aux` según el caso. Determinista es la palabra
clave: el mismo punto da **siempre** la misma orientación, lo que hace el
resultado reproducible.

Por eso el docstring lo dice explícitamente: "El giro alrededor del eje de
apuntado queda libre (se fija de forma determinista)."

**Un detalle que merece mención**: el docstring añade que *no* se usa
`tf_transformations` porque no es importable en este entorno, y que el
quaternion se extrae con `PyKDL.GetQuaternion()`. Así se evita una dependencia
extra y se garantiza la coherencia de orden de componentes con PyKDL.

### 6.8 `IkResult`: el dataclass que transporta el diagnóstico

```python
@dataclass
class IkResult:
    q: List[float] = field(default_factory=lambda: [0.0] * 6)
    solved: bool = False
    inside_limits: bool = False
    joints_inside: List[bool] = field(default_factory=lambda: [True] * 6)
    pos_error: float = math.inf
    rot_error: float = math.inf
    continuity: float = 0.0
```

Cada campo tiene un propósito diagnóstico:

| Campo | Para qué |
|---|---|
| `q` | La solución, en radianes |
| `solved` | ¿Convergió dentro de tolerancia? |
| `inside_limits` | ¿Respeta todos los topes? |
| `joints_inside` | **Qué** articulación concreta se sale (para el informe) |
| `pos_error` | Distancia al objetivo (m) |
| `rot_error` | Desajuste angular (rad) |
| `continuity` | Distancia al punto anterior (para elegir rama) |

`joints_inside` es lo que permite al informe decir *"FUERA de límites en
joint_3"* en vez de un genérico "fuera de límites".

**Todos los defaults son "fallo seguro"**: `solved=False`, `pos_error=inf`. Un
`IkResult()` vacío es un resultado negativo, nunca una solución válida por
accidente.

---

## 7. `trajectory_builder.py` — capa 2: cómo viajar

### 7.1 Responsabilidad

```
CAPA 2 DE 3 (calculo): convierte los puntos de config/points.yaml en una
trayectoria articulada para el joint_trajectory_controller.
```

Y la separación de responsabilidades, declarada en el docstring:

```
1. points_config    -> QUÉ puntos visitar
2. TrajectoryBuilder-> CÓMO viajar (cartesianos -> articulaciones, con IK)
3. joint_trajectory_client -> ENVÍO del goal al controller
```

### 7.2 Los dos dataclasses de resultado

`WaypointRecord` es el **registro por punto** que se imprime en terminal y
reutiliza el cliente al enviar. Contiene el diagnóstico completo:

```python
@dataclass
class WaypointRecord:
    name: str = "waypoint"
    cartesian_position: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    quaternion_xyzw: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 1.0])
    q: List[float] = field(default_factory=lambda: [0.0] * 6)
    joint_ok: List[bool] = field(default_factory=lambda: [True] * 6)
    inside_limits: bool = True
    solved: bool = False
    info: str = ""
```

`quaternion_xyzw` tiene como default la **identidad** `[0, 0, 0, 1]`. Esto es
correcto para el modo articular (punto `home`), donde no hay orientación
calculada: el quaternion identidad significa "sin rotación". Un detalle de
honestidad del diseño: para un punto articular, `cartesian_position` queda en
`[0,0,0]`, que **no es la posición real** del TCP, sino un valor por defecto.
El informe lo muestra, y conviene saber que ahí no hay información real.

`info` es una **cadena de estado legible**, con tres valores reales
verificados en ejecución:

| `info` | Significado |
|---|---|
| `+Z -> centro pieza` | Orientación automática, IK resuelta |
| `articular (passthrough, sin IK)` | Punto articular directo |
| `modo joint (passthrough)` | Modo `point_type: joint` completo |
| `orientación manual` | Quaternion del usuario |
| `IK SIN SOLUCIÓN (pos X m, rot Y rad)` | Fallo de IK, con el error concreto |
| `IK ok pero FUERA DE LÍMITES` | Resuelta pero físicamente inalcanzable |

`TrajectoryPlan` es el resultado global:

```python
@dataclass
class TrajectoryPlan:
    records: List[WaypointRecord] = field(default_factory=list)
    trajectory: Optional[JointTrajectory] = None
    all_solved: bool = False
    all_inside_limits: bool = False
```

`trajectory` es `Optional`: **solo se rellena si todo se resolvió**. Esta es la
garantía de seguridad central del paquete.

### 7.3 `build()`: el orquestador del cálculo

```python
def build(self) -> TrajectoryPlan:
    if self._config.point_type == "joint":
        return self._build_joint_mode()

    records: List[WaypointRecord] = []
    q_prev: Optional[Sequence[float]] = None

    for point in self._config.points:
        record = self._solve_point(point, q_prev)
        records.append(record)
        if record.solved:
            q_prev = record.q
        else:
            q_prev = None  # no encadenar soluciones fallidas

    plan = TrajectoryPlan(records=records)
    plan.all_solved = all(r.solved for r in records)

    if plan.all_solved:
        plan.trajectory = self._to_joint_trajectory(records)
        plan.all_inside_limits = all(r.inside_limits for r in records)
    return plan
```

Cuatro decisiones importantes aquí:

**a) `q_prev` se propaga entre puntos.** Es lo que da la **continuidad**: el
punto N se calcula<'_partiendo del resultado del punto N−1``, lo que hace
mucho más probable que la solución sea larama suave. Y reduce las semillas
necesarias, porque la buena está ya en la lista.

**b) `q_prev = None` tras un fallo.** Si un punto no se resuelve, **no se
encadena** la solución: el siguiente punto no puede usar un `q_prev`
inexistente. Está comentado en el propio código: `# no encadenar soluciones
fallidas`.

**c) `all_solved` es una conjunción estricta.** Un solo fallo invalida todo.

**d) La trayectoria solo se construye si `all_solved`.** Este es el **gate de
seguridad** del paquete. Si algo falla, `plan.trajectory` queda en `None` y el
envío es imposible (ver §8.3).

### 7.4 `_solve_point()`: cómo se resuelve un punto

**Primero el atajo articular** (`trajectory_builder.py:102-112`):

```python
if "positions" in point:
    q = [float(v) for v in point["positions"]]
    ok, per = self._kinematics.check_joint_limits(q)
    return WaypointRecord(
        name=name, q=q, joint_ok=per, inside_limits=ok,
        solved=True, info="articular (passthrough, sin IK)",
    )
```

Se comprueba `positions` **antes** de la cinemática, y el comentario explica
por qué: "Se evalúa primero para no romper el flujo cartesiano de los demás
puntos". Es lo que permite mezclar puntos articulares y cartesianos en el
mismo fichero, como hace el `home` del ejemplo.

Nótese que un punto articular **nunca puede fallar la IK** (`solved=True`
siempre): solo puede quedarse fuera de límites, que se comprueba aparte.

**Después, la orientación** (`trajectory_builder.py:117-135`), con prioridad
explícita:

1. Si el punto trae `quaternion_xyzw` → **gana siempre**, se usa tal cual
   (`info = "orientación manual"`). Esto permite **sobrescribir** el modo
   global en un punto concreto, que es justo lo que promete el comentario del
   YAML (`points.yaml:35-36`).
2. Si no, y el modo es `look_at_part_center` → orientación calculada
   (`info = "+Z -> centro pieza"`).
3. Si no → error, ya validado en `PointsConfig`.

**Finalmente la IK**:

```python
result = self._kinematics.inverse_kinematics(
    frame, q_prev=q_prev, q_state=None
)
```

y el relleno del registro con `result.solved`, `result.q`,
`result.joints_inside`, `result.inside_limits`, más el `info` descriptivo del
fallo si lo hay.

### 7.5 El modo `joint` completo

Si `point_type: joint`, **toda** la lista usa `positions` y la IK se salta
(`trajectory_builder.py:154-174`). Es un **modo de diagnóstico**: permite
comprobar el envío de una trayectoria conocida sin depender de la IK, útil
para separar problemas ("¿es la cinemática o es el envío?").

### 7.6 `_to_joint_trajectory()`: el mensaje final

```python
def _to_joint_trajectory(self, records: List[WaypointRecord]) -> JointTrajectory:
    trajectory = JointTrajectory()
    trajectory.joint_names = self._config.joint_names
    trajectory.points = []
    for (point, record) in zip(self._config.points, records):
        msg_point = JointTrajectoryPoint()
        msg_point.positions = [float(v) for v in record.q]
        msg_point.time_from_start = Duration(
            seconds=float(point["time_from_start"])
        ).to_msg()
        trajectory.points.append(msg_point)
    return trajectory
```

Puntos clave:

- `joint_names` se copia **de la configuración ya validada**, en el orden
  correcto (§5.3).
- `positions` = los ángulos en **radianes**, en el orden de `joint_names`.
- `time_from_start` se convierte de float a `builtin_interfaces/msg/Duration`
  con `rclpy.duration.Duration`, que es la forma correcta de hacerlo en ROS 2
  (la estructura de ROS 2 es `sec` + `nanosec`, no un float).

**Sobre las velocidades y aceleraciones**: `JointTrajectoryPoint` tiene campos
`velocities` y `accelerations`, y aquí **no se rellenan**. Se dejan vacíos. El
controlador los calcula por su cuenta, y las tolerancias de velocidad y
aceleración (`goal_tolerance.velocity` / `.acceleration`) se aplican igualmente
como criterio de éxito.

---

## 8. `trajectory_node.py` — el nodo ROS 2

### 8.1 La clase `TrajectoryNode`

```python
class TrajectoryNode(Node):
    """Orquesta: configuración -> cálculo -> [envío opcional]."""

    def __init__(self, config_path: str, send: bool = False) -> None:
        super().__init__("staubli_trajectory_node")
        self._config_path = config_path
        self._send = send
```

Es un `rclpy.node.Node` con **dos parámetros inyectados por el constructor**,
no por parámetros ROS. No usa `declare_parameter`, ni lee `/joint_states`, ni
se suscribe a ningún topic. **Es un nodo de un solo uso**: se crea, calcula,
opcionalmente envía, y termina. No es un servicio continuo.

Esto es coherente con su propósito: es una **herramienta de generación y
ejecución de una trayectoria puntual**, no un controlador de movimiento
permanente.

El nombre del nodo es `staubli_trajectory_node`, que es el mismo que se fija
en el launch file (`trajectory.launch.py:62,72`).

### 8.2 `run()`: la secuencia completa

```python
def run(self) -> bool:
    try:
        config = PointsConfig(self._config_path)
    except Exception as exc:
        self.get_logger().error(f"Error de configuración: {exc}")
        return False
    ...
    try:
        builder = TrajectoryBuilder(config)
        plan = builder.build()
    except Exception as exc:
        self.get_logger().error(f"Error construyendo la trayectoria: {exc}")
        return False

    _print_plan(plan, config)          # SIEMPRE, sea o no envío

    if not self._send:
        self.get_logger().info("Modo simulación (sin envío). Usa '--send' para mover el robot.")
        return plan.all_solved
    if plan.trajectory is None:
        self.get_logger().error("No se puede enviar: la IK no produjo solución para todos los puntos.")
        return False
    if not plan.all_inside_limits:
        self.get_logger().error("No se envía: hay puntos fuera de límites articulares.")
        return False

    client = JointTrajectoryClient(self)
    if not client.wait_for_server():
        return False
    return client.send(plan.trajectory, config)
```

**Las tres compuertas de seguridad**, en este orden exacto:

1. **`plan.trajectory is None`** → no se envía. La trayectoria solo existe si
   todas las IK se resolvieron (§7.3).
2. **`not plan.all_inside_limits`** → no se envía. Resuelta pero inalcanzable
   no es ejecutable.
3. **`wait_for_server()`** → no se envía si el controlador no está disponible.

Las tres deben pasar. Es un diseño deliberadamente conservador: es preferible
**no mover el robot** a mandarle una trayectoria que no puede cumplir.

**El informe se imprime siempre**, antes de cualquier compuerta
(`trajectory_node.py:119`), incluso con `--send`. Es decir: con `--send` **se
calcula, se imprime y luego se mueve**. El usuario siempre ve el plan antes de
que el robot se mueva, lo cual es valioso en una aplicación real.

Los dos `try/except` anchos con `# noqa: BLE001` son intencionados: el
comentario del segundo lo dice ("reporte claro al usuario"). Una excepción
inesperada de la cinemática se convierte en un mensaje de error legible en vez
de un traceback.

### 8.3 `main()`: el arranque desde línea de órdenes

```python
def main(args: Optional[List[str]] = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=None, help="Ruta alternativa a config/points.yaml.")
    parser.add_argument("--send", action="store_true", help="...")
    cli_args, desconocidos = parser.parse_known_args(args)

    rclpy.init(args=desconocidos or None)

    config_path = cli_args.config or _default_config_path()
    node = TrajectoryNode(config_path, send=cli_args.send)
    try:
        ok = node.run()
    except KeyboardInterrupt:
        ok = False
    finally:
        node.destroy_node()
        rclpy.shutdown()

    raise SystemExit(0 if ok else 1)
```

Cinco detalles no evidentes:

**a) `parse_known_args` en vez de `parse_args`.** Esto deja pasar a rclpy los
argumentos que no son del programa. El comentario lo explica:
`# rclpy solo consume sus propios argumentos (los '--ros-args ...')`. Sin esto,
`ros2 run staubli_trajectory trajectory_node --ros-args -p foo:=1` fallaría con
"unrecognized arguments". Es un detalle de integración con el ecosistema ROS
que cuesta descubrir.

**b) `argparse`, no parámetros ROS.** El nodo se configura por línea de órdenes,
no por `ros2 param`. Coherente con ser una herramienta de un solo uso.

**c) `action="store_true"`** hace que `--send` sea un interruptor: no requiere
valor. `--send` = True, su ausencia = False.

**d) `_default_config_path()`** (§2.2) resuelve el YAML del share cuando no se
pasa `--config`.

**e) `SystemExit(0 if ok else 1)`**: el **código de salida** refleja el
resultado. `0` = todo correcto, `1` = fallo. Verificado: con el
`points.yaml` del repo sale `EXIT=0`; con un punto inalcanzable sale `EXIT=1`.
Esto permite encadenar el nodo en scripts y comprobar su resultado.

**f) `finally: node.destroy_node(); rclpy.shutdown()`**: limpieza correcta
incluso con `KeyboardInterrupt`. Es lo que hace que `Ctrl+C` no deje procesos
colgados ni fugas de recursos.

### 8.4 El informe por terminal

`_print_plan()` y `_print_point()` producen la salida real del nodo. Ejecutado
con el `points.yaml` del repositorio, la salida **exacta** es:

```
[INFO] [staubli_trajectory_node]: Config: 'config/points.yaml' | frame=base_link | 4 puntos | orientation_mode=look_at_part_center
[INFO] [staubli_trajectory_node]: Modo simulación (sin envío). Usa '--send' para mover el robot.
==============================================================================
PLAN DE TRAYECTORIA CARTESIANA  |  frame=base_link  |  mode=look_at_part_center
==============================================================================
Punto 1 (inspection_fronto_superior)  [+Z -> centro pieza]
  posición cartesiana : (+0.0000, +0.5500, +0.3800)  m
  quaternion (xyzw)   : (-0.6683, +0.6683, +0.2311, +0.2311)
  q1..q6 (rad)        : [+1.5313, +0.4443, +1.8595, +3.2829, -0.1739, +2.9714]
  límites articulares : DENTRO  [ok]
------------------------------------------------------------------------------
Punto 2 (inspection_izquierda)  [+Z -> centro pieza]
  posición cartesiana : (-0.2200, +0.4800, +0.3600)  m
  quaternion (xyzw)   : (-0.2802, +0.7848, +0.1858, +0.5206)
  q1..q6 (rad)        : [+2.0831, +0.5478, +1.8437, +1.7333, -1.1670, -0.8186]
  límites articulares : DENTRO  [ok]
------------------------------------------------------------------------------
Punto 3 (inspection_derecha)  [+Z -> centro pieza]
  posición cartesiana : (+0.2200, +0.4800, +0.3600)  m
  quaternion (xyzw)   : (+0.7848, -0.2802, -0.5206, -0.1858)
  q1..q6 (rad)        : [+0.9809, +0.5478, +1.8437, +1.3554, +1.2189, -2.3337]
  límites articulares : DENTRO  [ok]
------------------------------------------------------------------------------
Punto 4 (home)  [articular (passthrough, sin IK)]
  posición cartesiana : (+0.0000, +0.0000, +0.0000)  m
  quaternion (xyzw)   : (+0.0000, +0.0000, +0.0000, +1.0000)
  q1..q6 (rad)        : [+0.0000, +0.0000, +0.0000, +0.0000, +0.0000, +0.0000]
  límites articulares : DENTRO  [ok]
------------------------------------------------------------------------------

Resumen: 4 puntos | IK resuelta: SÍ | dentro de límites: SÍ
```

Cuatro observaciones leyendo esta salida real:

**1. Los puntos 1, 2 y 3 son simétricos en intención** (frontal, izquierda,
derecha de la pieza) y comparten `q2` (`0.5478`) y `q3` (`1.8437`). No es
casualidad: los tres mantienen la misma distancia y altura aproximadas al
centro de la pieza, y la misma orientación "mirando a la pieza", lo que fija
parcialmente la configuración de hombro y codo. `q1` varía (`1.5313`, `2.0831`,
`0.9809`) porque es la rotación de la base hacia la izquierda, el frente y la
derecha. `q4` y `q6` varían mucho más porque la muñeca es la parte más libre.

**2. El punto 4 (`home`) muestra `posición cartesiana: (0,0,0)`.** Como se
explicó en §7.2, es el valor por defecto, **no la posición real del TCP**. La
posición real de `home` (todos los ángulos a cero) es `(0, 0.02, 1.295)` m,
verificada con la FK en §6.4. El informe no la calcula porque en modo articular
no hay frame de objetivo. **No es un error**: es una limitación del informe en
ese modo, y conviene conocerla para no interpretar mal la salida.

**3. `joint_4 = 3.2829 rad = 188°` en el punto 1** está dentro de los límites
(`±270°`), pero es un valor "envuelto": la misma orientación se podría expresar
como `3.2829 − 2π = -2.99 rad`. El solver no normaliza los ángulos a
`[-π, π]`, lo cual es correcto para un robot con ejes de ±270° reales (donde
el rango completo es físicamente alcanzable) pero puede sorprender si se
compara con un robot estandarizado. Los límites se leen del YAML, así que el
comportamiento se adapta al robot real.

**4. La última línea del `Resumen` cambia según el resultado**, y es el
diagnóstico de alto nivel:

```
> No se generará JointTrajectory hasta resolver correctamente las IK de todos los puntos.
```

cuando falla la IK, o:

```
> La trayectoria es válida pero hay puntos fuera de límites: revisa coordenadas en config/points.yaml.
```

cuando se resolvió pero hay topes. Son dos Diagnósticos **accionables** que
señalan exactamente qué revisar.

Con un punto inalcanzable, la salida real muestra el `info` con los errores
concretos:

```
Punto 1 (demasiado_lejos)  [IK SIN SOLUCIÓN (pos inf m, rot inf rad)]
  posición cartesiana : (+5.0000, +5.0000, +5.0000)  m
  quaternion (xyzw)   : (+0.8336, +0.3113, -0.4274, +0.1596)
  q1..q6 (rad)        : ---  (IK SIN SOLUCIÓN)
```

Nótese que la orientación **sí se calcula** (el quaternion aparece) aunque la IK
falle: son dos etapas independientes. Y que `---` sustituye a los ángulos,
porque no los hay.

---

## 9. `joint_trajectory_client.py` — capa 3: el envío

### 9.1 Responsabilidad y contrato

```
CAPA 3 DE 3 (envio): ActionClient del control_msgs/action/FollowJointTrajectory.

Action server (verificado en el repositorio):
    /joint_trajectory_controller/follow_joint_trajectory

Contrato del controlador (staubli_bringup/config/controllers.yaml):
    - joints: joint_1 .. joint_6   (orden exacto)
    - command_interfaces: position (todas las posiciones van en radianes)
```

El nombre de la acción está **como constante de clase**, no hardcodeado en
varios sitios:

```python
ACTION_NAME = "/joint_trajectory_controller/follow_joint_trajectory"
```

### 9.2 `wait_for_server()`: comprobar antes de enviar

```python
def wait_for_server(self, timeout_s: float = 10.0) -> bool:
    if not self._client.wait_for_server(timeout_sec=timeout_s):
        self._node.get_logger().error(
            f"Action server '{self.ACTION_NAME}' no disponible tras {timeout_s} s. "
            "¿Está lanzado el Gazebo/robot y activado el joint_trajectory_controller?"
        )
        return False
    self._node.get_logger().info(f"Action server '{self.ACTION_NAME}' disponible.")
    return True
```

El mensaje de error es **accionable**: no dice "fallo", sino "¿está lanzado
Gazebo y activado el controlador?". Es un ejemplo de buen mensaje de error en
robótica, donde la causa más común es simplemente "el robot no está
arrancado".

El **timeout de 10 s** es un valor por defecto parametrizable, y evita el
bloqueo infinito si el robot no está en marcha.

### 9.3 `send()`: el ciclo completo de la acción

```python
goal = self._build_goal(trajectory, config)

self._node.get_logger().info(
    f"Enviando goal con {len(goal.trajectory.points)} waypoints ..."
)

goal_future = self._client.send_goal_async(goal, feedback_callback=self._on_feedback)
rclpy.spin_until_future_complete(self._node, goal_future)
if not goal_future.done():
    ...
    return False

intento_goal = goal_future.result()
if not intento_goal.accepted:
    self._node.get_logger().error("Goal RECHAZADO por el controller.")
    return False

result_future = intento_goal.get_result_async()
rclpy.spin_until_future_complete(self._node, result_future)
...
```

**Cinco pasos explícitos**, cada uno con su comprobación:

1. Construir el goal.
2. Enviarlo **asíncronamente** y esperar.
3. Verificar que se envió (`goal_future.done()`).
4. Verificar que fue **aceptado** (`accepted`). El servidor puede rechazar un
   goal: no todo goal enviado se ejecuta.
5. Esperar el **resultado final** y comprobar su código.

**`spin_until_future_complete` es la clave de la implementación.** En ROS 2, un
nodo solo procesa mensajes cuando su executor gira. Como este nodo no usa
`spin()`, las futures nunca se resolverían: el código debe **girar el executor
manualmente** mientras espera. El comentario del código lo dice:
`# Envío asíncrono; se espera con spin_until_future_complete (un solo ejecutor)`.

Esto implica que el nodo es de **un solo hilo** y su `main()` es
**bloqueante** desde el punto de vista de la shell: hasta que la trayectoria
termine, el proceso no sale.

**El log antes de enviar** (`f"Enviando goal con N waypoints ... {posiciones}"`)
es una salvaguarda importante en un robot real: **imprime exactamente qué se va
a mandar antes de mandarlo**, para poder abortar si no es lo esperado.

### 9.4 `_build_goal()`: la trayectoria más las tolerancias

```python
goal = FollowJointTrajectory.Goal()
goal.trajectory = trajectory

for name in config.joint_names:
    tol = JointTolerance()
    tol.name = name
    tol.position = config.goal_position_tolerance
    tol.velocity = config.goal_velocity_tolerance
    tol.acceleration = config.goal_acceleration_tolerance
    goal.goal_tolerance.append(tol)

goal.goal_time_tolerance = Duration(
    seconds=config.goal_time_tolerance
).to_msg()
return goal
```

**El goal es la trayectoria más un contrato de precisión.** Por cada
articulación se crea un `JointTolerance` con posición, velocidad y aceleración
(los tres valores del `goal_tolerance` del YAML), y se añade un
`goal_time_tolerance` como margen temporal del punto final.

**Detalle importante y a menudo pasado por alto**: solo se rellenan
`goal_tolerance` y `goal_time_tolerance`, **no** `path_tolerance`. El
`path_tolerance` controla el error máximo **durante** el recorrido; si está
vacío, el controlador no verifica la trayectoria intermedia. Solo comprueba
el **punto final**. Para este caso de uso (puntos de inspección) es razonable
—los puntos intermedios son referencias de paso, no poses que deban respetarse
con exactitud—, pero conviene saberlo.

Un `JointTolerance` **por articulación** es obligatorio porque las tolerancias
pueden diferir entre ejes (una junta con más recorrido puede tolerar más
error), y el controlador las asocia por nombre.

### 9.5 El resultado: éxito no es "terminó"

```python
result = result_future.result()
resultado = getattr(result, "result", result)
error_code = getattr(resultado, "error_code", None)
nombre = _RESULT_CODES.get(error_code, f"desconocido ({error_code})")

if error_code == FollowJointTrajectory.Result.SUCCESSFUL:
    self._node.get_logger().info(f"Trayectoria completada con éxito ({nombre}).")
    return True

self._node.get_logger().error(
    f"Trayectoria finalizada con código {nombre}. "
    f"Detalle: {getattr(resultado, 'error_string', '')}"
)
return False
```

**Solo `SUCCESSFUL` devuelve `True`.** Cualquier otro código es un fallo, y se
traduce a un **nombre legible** mediante `_RESULT_CODES`
(`joint_trajectory_client.py:26-33`):

| Código | Significado |
|---|---|
| `SUCCESSFUL` | Completada con éxito |
| `INVALID_GOAL` | El goal es inválido |
| `INVALID_JOINTS` | Nombres de articulaciones no válidos |
| `OLD_HEADER_TIMESTAMP` | La marca de tiempo es demasiado antigua |
| `PATH_TOLERANCE_VIOLATED` | Se superó la tolerancia durante el recorrido |
| `GOAL_TOLERANCE_VIOLATED` | No se alcanzó la posición final dentro de tolerancia |

**`GOAL_TOLERANCE_VIOLATED` merece atención**: significa que el robot **no llegó
exactamente** al punto final dentro del margen (aquí 0.01 rad de posición y 2.5
s de tiempo). Puede ocurrir por fricción, o porque el path difiere
del plan. El código lo trata como fallo, que es lo prudente.

Los `getattr` con default son **defensivos**: protegen contra variations en la
estructura de la respuesta sin romper la ejecución.

### 9.6 El feedback durante la ejecución

```python
@staticmethod
def _on_feedback(feedback_msg) -> None:
    fb = getattr(feedback_msg, "feedback", feedback_msg)
    idx = getattr(fb, "index", -1)
    actual = getattr(getattr(fb, "actual", None), "positions", [])
    if actual:
        mostra = ", ".join(f"{v:.3f}" for v in actual)
        print(f"[feedback] waypoint {idx}: actual=[{mostra}]")
```

El `FollowJointTrajectory` devuelve feedback con las **posiciones reales
medidas** mientras se ejecuta. Este callback solo las imprime.

**Nota de coherencia menor**: usa `print()` en vez de
`self._node.get_logger()`, a diferencia del resto del archivo. En un nodo ROS
esto es ligeramente inconsistente (los `print` no llevan marca de tiempo ni
nivel), pero es funcional y no afecta al comportamiento.

El doble `getattr` (`feedback_msg.feedback` o el propio mensaje) y los defaults
(`-1`, `[]`) hacen el callback **tolerante a variaciones** en la estructura del
mensaje.

---

## 10. `launch/trajectory.launch.py` — el arranque con `ros2 launch`

### 10.1 Los dos argumentos

```python
config_arg = DeclareLaunchArgument(
    "config",
    default_value=default_config,
    description="Ruta a config/points.yaml (posiciones cartesianas de tool0, centro de la pieza, tiempos y tolerancias).",
)

send_arg = DeclareLaunchArgument(
    "send",
    default_value="false",
    description="true => además de calcular, ENVÍA la trayectoria al joint_trajectory_controller (requiere Gazebo + controllers).",
)
```

| Argumento | Default | Para qué |
|---|---|---|
| `config` | `<share>/config/points.yaml` | Ruta alternativa al YAML |
| `send` | `"false"` | **`true` = además de calcular, envía** |

El default de `config` se resuelve con `get_package_share_directory` + `join`,
que es la forma **correcta** de referenciar ficheros de un paquete instalado
(`trajectory.launch.py:34-38`).

### 10.2 El truco de los dos nodos condicionados

La parte más interesante del launch file. En vez de **un** nodo con un
parámetro condicional, declara **dos declaraciones del mismo `Node`**, cada una
con una `condition` mutuamente excluyente:

```python
node_calculate = Node(
    package="staubli_trajectory", executable="trajectory_node",
    name="staubli_trajectory_node", output="screen",
    arguments=["--config", LaunchConfiguration("config")],
    condition=UnlessCondition(LaunchConfiguration("send")),
)

node_send = Node(
    package="staubli_trajectory", executable="trajectory_node",
    name="staubli_trajectory_node", output="screen",
    arguments=["--config", LaunchConfiguration("config"), "--send"],
    condition=IfCondition(LaunchConfiguration("send")),
)

return LaunchDescription([config_arg, send_arg, node_calculate, node_send])
```

`IfCondition` evalúa `"true"` como verdadero y cualquier otra cosa como falso;
`UnlessCondition` es su negación. Por eso ambos tienen el **mismo nombre**: el
nombre del nodo no cambia según el modo, solo los argumentos.

**¿Por qué dos en vez de uno con un argumento condicional?** Porque
`--send` es un `store_true` de `argparse`: se pasa **o no se pasa**. No se puede
escribir "pasa `--send` solo si la condición es verdadera" con una sustitución
de launch sin hacks. Declarar los dos casos y condicionarlos es **limpio,
explícito y verificable**: leyendo el launch file se ve exactamente qué pasa en
cada modo.

Solo uno de los dos llega a instanciarse, así que **no hay doble movimiento**.

### 10.3 Comandos de uso

```bash
# 1. Solo calcular e imprimir (seguro por defecto)
ros2 launch staubli_trajectory trajectory.launch.py

# 2. Calcular Y enviar (requiere Gazebo + controlador activo)
ros2 launch staubli_trajectory trajectory.launch.py send:=true

# 3. Con un YAML alternativo
ros2 launch staubli_trajectory trajectory.launch.py config:=/ruta/otro.yaml

# 4. Equivalente con ros2 run (control directo del ejecutable)
ros2 run staubli_trajectory trajectory_node --send
```

`output="screen"` en ambos nodos hace que la salida (los `print` y los logs)
salgan directamente en la terminal, que es lo apropiado para una herramienta
de diagnóstico.

### 10.4 `package.xml`: dependencias declaradas

```xml
<buildtool_depend>buildtool_ament_python</buildtool_depend>

<exec_depend>rclpy</exec_depend>
<exec_depend>control_msgs</exec_depend>
<exec_depend>trajectory_msgs</exec_depend>
<exec_depend>builtin_interfaces</exec_depend>
<exec_depend>ament_index_python</exec_depend>
<exec_depend>python3-yaml</exec_depend>
<exec_depend>orocos_kdl_vendor</exec_depend>
<exec_depend>launch_ros</exec_depend>
```

| Dependencia | Para qué se usa |
|---|---|
| `rclpy` | El framework de nodos ROS 2 |
| `control_msgs` | `FollowJointTrajectory` (la acción) |
| `trajectory_msgs` | `JointTrajectory` y `JointTrajectoryPoint` |
| `builtin_interfaces` | `Duration` (tiempos del mensaje) |
| `ament_index_python` | `get_package_share_directory` |
| `python3-yaml` | Leer los ficheros YAML |
| `orocos_kdl_vendor` | **PyKDL**, la cinemática |
| `launch_ros` | El launch file |

**Dos dependencias usadas pero no declaradas** (verificado):

1. **`staubli_robot_description`** — el código **lee sus ficheros de
   configuración en tiempo de ejecución** (§6.1), pero no hay ningún
   `<exec_depend>staubli_robot_description</exec_depend>` en el `package.xml`.
   Es una dependencia real. El código lo salva con el fallback al árbol de
   fuentes (§6.2), así que no rompe en un workspace normal donde ambos
   paquetes están juntos, pero es una omisión real: si se instala sólo
   `staubli_trajectory` en otro sistema, la búsqueda por el share fallaría y
   sólo funcionaría si el repo de fuentes estuviera presente.

2. **`python3-numpy`** — `tx2_60l_ik.py:35` hace `import numpy as np` y lo
   usa en `jacobian()` y en las semillas aleatorias, pero no hay
   `<exec_depend>python3-numpy</exec_depend>`. En la práctica `numpy` suele
   estar instalado como dependencia de otras cosas de ROS, pero técnicamente
   falta declararlo.

Ninguna de las dos omisiones rompe el proyecto tal como está, pero son
desviaciones reales entre lo que el código importa y lo que el manifiesto
declara. "Esto se comprueba en: `staubli_trajectory/package.xml` (no aparecen)
frente a `tx2_60l_ik.py:35` y `tx2_60l_ik.py:60-70` (sí se usan)."

---

## 11. `setup.py` y `setup.cfg` — construcción e instalación

Ya comentados en §2.1 y §2.2. Aquí el detalle que importa para entender el
paquete como unidad construible.

**Tipo: `ament_python`**, no `ament_cmake`. Implicaciones:

- No hay `CMakeLists.txt` (reiterado: **no existe** en este paquete).
- No hay C++ que compilar, ni `find_package`, ni `ament_target_dependencies`.
- La "compilación" es esencialmente una copia/install de ficheros Python y la
  generación del ejecutable desde el `entry_point`.
- `ros2 build staubli_trajectory` produce el share con `config/` y `launch/`,
  y el ejecutable `trajectory_node` en `lib/staubli_trajectory/`.

**El `entry_point` es el ejecutable:**

```python
entry_points={
    "console_scripts": [
        "trajectory_node = staubli_trajectory.trajectory_node:main",
    ],
},
```

`console_scripts` le dice a `setuptools` que genere un pequeño script en
`bin/` que hace `from staubli_trajectory.trajectory_node import main; main()`.
`setup.cfg` lo redirige a `lib/staubli_trajectory/`, que es donde `ros2 run`
lo busca. De ahí que en el paquete **no exista** un fichero con
`#!/usr/bin/env python3` y `if __name__ == "__main__"` **como script
independiente**: el entry point lo genera `setuptools` automáticamente.

Sí existe, sin embargo, el bloque `if __name__ == "__main__": main()` al final
de `trajectory_node.py:175-176`, que sirve para ejecutar el módulo directamente
con `python3 -m staubli_trajectory.trajectory_node` durante el desarrollo, sin
instalar. Es una comodidad de desarrollo.

**`zip_safe=True`** (`setup.py:32`) indica que el paquete puede instalarse como
un zip. Para un paquete de ROS 2 es habitual y no tiene efecto práctico
negativo aquí.

**`version="0.0.1"`** y `maintainer="rocio"` / `rocio@example.com`: los
metadatos son marcadores de posición, como en el resto de paquetes del
proyecto.

---

## 12. Cómo se conecta con los demás paquetes

`staubli_trajectory` es una pieza más de un sistema de paquetes. Esta sección
mapea las dependencias en ambas direcciones.

### 12.1 De este paquete hacia fuera

**`staubli_robot_description`** — dependencia de **datos**, no de código
(§6.1). Lee en ejecución:

| Fichero | Qué extrae |
|---|---|
| `config/tx2_60l/kinematics.yaml` | Origen (`xyz`) y eje (`axis`) de cada articulación |
| `config/tx2_60l/robot_model.yaml` | Offset del `end_effector` (`tool0` respecto a `link_6`) y flags del sensor |
| `config/tx2_60l/joint_limits.yaml` | Límites de posición y velocidad (vía `!degrees`) |

Esta es la relación más importante del paquete: **la geometría del robot tiene
una única fuente de verdad**, y este paquete la consume sin duplicarla.

**`staubli_bringup`** — dependencia de **infraestructura en ejecución**. El
paquete asume que existe un `controller_manager` con el
`joint_trajectory_controller` cargado, y que la acción
`/joint_trajectory_controller/follow_joint_trajectory` está disponible. La
configuración de ese controlador está en `staubli_bringup/config/controllers.yaml`
(§3.7), y el mundo de Gazebo con la pieza en
`staubli_bringup/worlds/inspection_cell.sdf` (la pieza en `(0, 0.66, 0.54)` que
origina el `part_center_base_link` del YAML, §4.4). **Esto se comprueba en:**
`staubli_bringup/config/controllers.yaml`, `staubli_bringup/worlds/inspection_cell.sdf`,
`staubli_bringup/launch/launch_gazebo.launch.py`.

**`staubli_gui`** y **`staubli_joystick_servo`** — dependencia de **código**,
pero en la dirección contraria (§12.2).

### 12.2 De otros paquetes hacia este (reutilización)

Ambos declaran `<exec_depend>staubli_trajectory</exec_depend>` y **reutilizan
su cinemática** en vez de duplicarla. Este paquete es, de hecho, la biblioteca
cinemática compartida del proyecto.

**`staubli_gui`** (`staubli_gui/staubli_gui/gui.py`):

- Línea 1238, 1534: `from staubli_trajectory.tx2_60l_ik import Tx260lKinematics` —
  para resolver IK de waypoints individuales (HOME, P1/P2/P3) **de forma
  asíncrona** para no bloquear la GUI.
- Líneas 1919-1920: `from staubli_trajectory.points_config import PointsConfig` y
  `from staubli_trajectory.trajectory_builder import TrajectoryBuilder` — para
  **reutilizar la misma IK de `points.yaml`**.
- Línea 204: `_POINTS_PKG = "staubli_trajectory"  # paquete propietario de config/points.yaml`.

Es decir, la GUI **no reimplementa** la cinemática: la pide prestada. El
comentario de la cabecera lo dice: "Las IK de P1/P2/P3 se reutilizan de
staubli_trajectory (points.yaml + …)". También replica la misma lógica de
`FollowJointTrajectory` para el HOME (líneas 162-179) y las mismas tolerancias
del goal, citando este paquete como fuente.

**`staubli_joystick_servo`** (`staubli_joystick_servo/staubli_joystick_servo/servo_node.py:36`):

```python
from staubli_trajectory.tx2_60l_ik import JOINT_VELOCITY_LIMITS_RAD, Tx260lKinematics
```

Importa tanto la clase como la **lista de límites de velocidad**, para limitar
la velocidad del mando manual. El comentario (línea 22) es explícito: "de
`staubli_trajectory.tx2_60l_ik.Tx260lKinematics`: no se duplica geometría". Es
un buen ejemplo de por qué `JOINT_VELOCITY_LIMITS_RAD` está expuesto a nivel de
módulo (línea 139): para que otros paquetes lo consuman.

### 12.3 Los tests de otros paquetes que importan este

Aunque no tiene tests propios, este paquete es **importado por los tests** de
sus hermanos, que añaden su ruta al `sys.path` para no necesitar instalación:

- `staubli_gui/test/conftest.py:18-20`: "Basta con añadir la ruta del paquete
  hermano `staubli_trajectory`".
- `staubli_joystick_servo/test/conftest.py:6,19`: "Añade la ruta del paquete
  hermano `staubli_trajectory` desde las fuentes".
- `staubli_gui/test/test_capture_position.py:30`: importa `Tx260lKinematics`.
- `staubli_joystick_servo/test/test_velocity_limiter.py:24`: importa
  `JOINT_VELOCITY_LIMITS_RAD` y `Tx260lKinematics`.

Esto explica por qué `find_tx2_60l_config_dir()` tiene el fallback al árbol de
fuentes (§6.2): al ejecutar esos tests **sin haber compilado**, la función
tiene que poder encontrar los YAML del robot subiendo por el sistema de
ficheros. Es un detalle de diseño pensado exactamente para este caso.

### 12.4 Mapa de dependencias

```
                    staubli_robot_description
                    (kinematics/robot_model/joint_limits)
                              ▲
                              │ lee YAML en ejecución
                              │
        ┌─────────────────────┴─────────────────────┐
        │                                           │
        │                                   (hereda de staubli_trajectory)
        │                                           │
  staubli_trajectory ──────► staubli_bringup        │
        │                  (controller_manager,      │
        │                   JTC, mundo Gazebo)       │
        │                                           │
        ├──► staubli_gui ◄──────────────────────────┘
        │     (importa Kinematics, PointsConfig, TrajectoryBuilder)
        │
        └──► staubli_joystick_servo
              (importa Kinematics, JOINT_VELOCITY_LIMITS_RAD)
```

**Resumen de relaciones:**

| Relación | Tipo | Qué comparte |
|---|---|---|
| → `staubli_robot_description` | Datos | Geometría y límites del robot (fuente única) |
| → `staubli_bringup` | Infraestructura | `controller_manager`, JTC, acción, mundo |
| ← `staubli_gui` | Código | Reutiliza la cinemática y el builder |
| ← `staubli_joystick_servo` | Código | Reutiliza la cinemática y los límites de velocidad |

---

## 13. El flujo completo y cómo ejecutarlo

### 13.1 El recorrido de extremo a extremo

**Fase A — Carga (sin ROS)**
1. `main()` parsea `--config` y `--send`.
2. `PointsConfig` lee y valida el YAML (§5). Cualquier error aquí aborta con
   mensaje claro.

**Fase B — Cálculo (sin ROS)**
3. `TrajectoryBuilder` se instancia con la configuración; crea un
   `Tx260lKinematics` con `random_seed` y `n_random_seeds` del YAML (§7.1).
4. Para cada punto:
   - Si trae `positions` → passthrough articular.
   - Si no → calcula orientación (manual o mirando a la pieza) y resuelve la IK
     con multsemilla (§6.5), encadenando `q_prev`.
5. Si todas se resolvieron, construye el `JointTrajectory` (§7.6).

**Fase C — Informe (siempre)**
6. `_print_plan` vuelca el diagnóstico completo por terminal (§8.4).

**Fase D — Envío (opcional, con `--send`)**
7. Compuerta 1: `plan.trajectory is not None`.
8. Compuerta 2: `all_inside_limits`.
9. `wait_for_server()` — compuerta 3.
10. `send()`: goal → aceptación → ejecución con feedback → resultado.
11. Se devuelve `True` solo si el resultado es `SUCCESSFUL`.

**Fase E — Código de salida**
12. `SystemExit(0 if ok else 1)`.

### 13.2 El mapa de clases y responsabilidades

| Capa | Clase / módulo | Responsabilidad | Conoce ROS |
|---|---|---|---|
| 1 | `PointsConfig` | QUÉ puntos: carga y valida el YAML | No |
| 2 | `TrajectoryBuilder` | CÓMO viajar: cartesiano → articulaciones | Sí (`trajectory_msgs`) |
| 2 | `Tx260lKinematics` | La matemática: FK/IK | No |
| 3 | `JointTrajectoryClient` | ENVÍO: la acción al controlador | Sí (`control_msgs`) |
| — | `TrajectoryNode` | Orquestación e informe | Sí |

**El diseño en capas es la fortaleza del paquete.** `Tx260lKinematics` y
`PointsConfig` **no dependen de ROS 2**, se pueden testear en Python puro, y de
hecho lo hacen los tests de los paquetes hermanos (§12.3). Solo la capa 2
(parcialmente), la 3 y el nodo dependen de ROS. Eso hace el código **fácil de
verificar sin levantar un robot ni un grafo de ROS** — exactamente lo que
permitió escribir este documento ejecutando el código directamente.

### 13.3 Comandos verificados

Los comandos de la sección 10.3 son los correctos. Para **verificar el cálculo
sin ROS** (lo que hice para este documento), basta el import directo:

```python
import sys; sys.path.insert(0, '<ruta>/staubli_trajectory')
from staubli_trajectory.points_config import PointsConfig
from staubli_trajectory.trajectory_builder import TrajectoryBuilder
cfg = PointsConfig('config/points.yaml')
plan = TrajectoryBuilder(cfg).build()
```

Esto no necesita `rclpy` inicializado ni ningún nodo, y devuelve el mismo plan
que el nodo completo. Es la forma más rápida de depurar el YAML o la cinemática.

### 13.4 Ejemplo real: qué pasa si un punto es inalcanzable

Para un punto en `(5, 5, 5)` —claramente fuera del alcance del robot— la
ejecución real da (verificado):

```
Punto 1 (demasiado_lejos)  [IK SIN SOLUCIÓN (pos inf m, rot inf rad)]
  q1..q6 (rad)        : ---  (IK SIN SOLUCIÓN)
...
Resumen: 2 puntos | IK resuelta: NO | dentro de límites: NO
> No se generará JointTrajectory hasta resolver correctamente las IK de todos los puntos.
EXIT=1
```

Y con `--send`, aunque la trayectoria no se envíe, la compuerta 1 lo impide con:
`"No se puede enviar: la IK no produjo solución para todos los puntos."`. El
robot **no se mueve**, y el código de salida es `1`. Comportamiento correcto y
seguro.

### 13.5 Configuración recomendada de Gazebo para enviar

Para que `--send` funcione hacen falta tres cosas (según el flujo y el
`controllers.yaml`):

1. **Gazebo y el robot lanzados** — para que exista el `controller_manager`.
   Ver `staubli_bringup/launch/launch_gazebo.launch.py`.
2. **`joint_trajectory_controller` activo** — cargando las 6 articulaciones.
   Configurado en `staubli_bringup/config/controllers.yaml`.
3. **La acción disponible** — `/joint_trajectory_controller/follow_joint_trajectory`.

Si falta cualquiera, `wait_for_server()` falla con el mensaje accionable
(§9.2) tras 10 s. **Esto se comprueba en** `staubli_bringup/launch/launch_gazebo.launch.py`
y `staubli_bringup/config/controllers.yaml`.

---

## 14. Resumen final

### 14.1 Qué hace, en una frase

`staubli_trajectory` lee una lista de puntos cartesianos de un YAML, calcula la
cinemática inversa de cada uno con PyKDL usando la geometría oficial del
TX2-60L, comprueba límites, y —**solo si se le pide con `--send`** — entrega la
trayectoria articular al `joint_trajectory_controller` mediante la acción
`FollowJointTrajectory`.

### 14.2 Los siete hechos que hay que recordar

1. **Es seguro por defecto.** Sin `--send`, calcula e imprime y nada más. Tres
   compuertas (trayectoria válida, dentro de límites, servidor disponible) antes
   de mandar nada.

2. **La geometría no está duplicada.** Las dimensiones se leen de
   `staubli_robot_description/config/tx2_60l/` en ejecución. Cambiar el modelo
   del robot cambia este paquete automáticamente.

3. **La IK es multsemilla y validada por FK.** Prueba 31–42 semillas, deduplica
   ramas, **verifica cada solución aplicando la cinemática directa** (error <
   0.1 mm) y elige por (dentro de límites, continuidad). Es lo que hace fiable
   un solver local como LMA.

4. **El orden de las articulaciones es un contrato validado.** `joint_1..joint_6`
   exacto, comprobado contra el `joint_trajectory_controller`. Un error aquí
   sería catastrófico y silencioso; por eso se valida.

5. **La orientación puede ser automática.** `look_at_part_center` apunta el
   +Z de `tool0` a la pieza (con un giro residual fijo de forma determinista);
   `manual` exige un quaternion por punto; un `quaternion_xyzw` puntual siempre
   **gana** sobre el modo global.

6. **El resultado es un plan, no un movimiento.** `WaypointRecord` por punto con
   posición, orientación, q1..q6 y estado de límites; `TrajectoryPlan` solo
   contiene la `JointTrajectory` si **todas** las IK se resolvieron.

7. **Es la biblioteca cinemática compartida del proyecto.** `staubli_gui` y
   `staubli_joystick_servo` importan `Tx260lKinematics` (y los límites de
   velocidad) en vez de reimplementarla. También es la fuente de las
   tolerancias del goal que la GUI replica.

### 14.3 Puntos de atención y detalles finos

Contrastar esto antes de confiar ciegamente en el comportamiento:

- **`planning_frame` es declarativo.** Se lee, se valida y se imprime, pero
  **no se usa para transformar**: todos los cálculos asumen `base_link`. Cambiar
  su valor no da error, pero tampoco hace nada (§4.5).
- **No se rellena `path_tolerance`.** El goal solo lleva `goal_tolerance` y
  `goal_time_tolerance`, así que el controlador verifica **el punto final**, no
  el recorrido intermedio (§9.4).
- **No hay `CMakeLists.txt` ni `scripts/`.** Es `ament_python` puro; el
  ejecutable es un `entry_point` de `setuptools` (§2.1).
- **No hay tests propios.** Los `test_depend` están heredados de la plantilla,
  pero la carpeta `test/` no existe (§2.1).
- **Dos dependencias usadas sin declarar:** `staubli_robot_description` y
  `python3-numpy` (§10.4). El fallback al árbol de fuentes salva el caso
  habitual, pero conviene saberlo.
- **`q_state` siempre es `None`.** El builder pasa `q_state=None`, así que la
  IK **no parte del estado real del robot**; arranca de ceros y de la solución
  anterior. El primer punto no se beneficia de la posición medida (§6.5).
- **Los quaternions no se normalizan a `[-π,π]`.** El solver devuelve ángulos
  crudos, lo que es correcto para un robot de ±270° reales pero puede llamar la
  atención (§8.4, observación 3).
- **El primer `time_from_start` no se valida como ≥ 0.** Solo se exige que la
  serie sea creciente (§5.4).
- **`goal_time_tolerance`:** el código usa 0.5 s por defecto, el fichero fija
  2.5 s. Borrar la clave del YAML cambia el comportamiento en silencio (§5.5).
- **El `print()` del feedback** no usa el logger del nodo (§9.6).
- **El punto articular muestra `(0,0,0)`** como posición cartesiana: es un valor
  por defecto, no la posición real del TCP (§8.4, observación 2).

### 14.4 Verificaciones realizadas para este documento

Todas las cifras y salidas de este README se obtuvieron **ejecutando el código
real**, no leyéndolo:

| Verificación | Resultado |
|---|---|
| FK en `q = 0` | `(0.0, 0.02, 1.295)` m — coincide con el docstring |
| Estructura de la cadena | 7 segmentos, 6 articulaciones |
| Geometría cargada de los YAML | Coincide con `kinematics/robot_model/joint_limits` |
| Límites y velocidades | `±180/127/150/270/120/270°`, `5.236/…/8.7266 rad/s` |
| Número de semillas | 31 (primer punto) / 42 (siguientes) |
| Round-trip FK→IK (30 configs) | 30/30 resueltas, error de posición máximo `0.0` m |
| Nodo con el `points.yaml` del repo | 4 puntos, IK SÍ, límites OK, `EXIT=0` |
| Punto inalcanzable `(5,5,5)` | `IK SIN SOLUCIÓN (pos inf m)`, `EXIT=1` |
| Límite de `joint_1` a 3.15 rad | Rechazado (excede ±180°) |
| Las 14 secciones | Estructura confirmada con el usuario |
