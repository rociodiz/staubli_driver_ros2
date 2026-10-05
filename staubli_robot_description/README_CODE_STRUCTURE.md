# Estructura del paquete `staubli_robot_description`

Guía de estudio del paquete `staubli_robot_description`. Está escrita para alguien que está
aprendiendo ROS 2, URDF/Xacro, RViz y Gazebo, y que quiere entender **qué papel juega este
paquete dentro del proyecto `staubli_driver_ros2`**.

La idea central del paquete es muy simple: **aquí vive la descripción geométrica del robot**, es
decir, qué links tiene, cómo se unen, qué límites tienen, qué mallas se dibujan y qué interfaces
de control se declaran. Ni el control del robot, ni la comunicación de red, ni la física están en
este paquete: solo la *descripción*.

---

## 1. Estructura general del paquete

Las carpetas que existen realmente en el paquete son estas:

```
staubli_robot_description/
├── package.xml                    Metadatos y dependencias del paquete
├── CMakeLists.txt                 Instalación y registro de tests
├── config/                        Datos numéricos del robot, en YAML, por modelo
│   ├── initial_positions.yaml     Posición inicial (común a todos los modelos)
│   ├── tx2_60/                    Parámetros del modelo TX2-60
│   ├── tx2_60l/                   Parámetros del modelo TX2-60L
│   └── tx2_60l_med/               Parámetros del modelo TX2-60L-med
├── env-hooks/                     Script de variables de entorno para Gazebo
├── launch/                        Archivos de lanzamiento (visualización)
├── meshes/                        Mallas 3D (visual y colisión)
│   ├── laser_triangle.obj/.mtl    Malla auxiliar de la cámara
│   └── tx2_60/                    Mallas de los tres modelos de la serie
│       ├── collision/*.stl
│       └── visual/*.dae, *.stl
├── r2c/                           Bloque <ros2_control> (r2c = "ros2 control")
│   ├── staubli.r2c.xacro          Punto de entrada del bloque ros2_control
│   └── include/                   Macros de articulaciones y GPIOs
├── rviz/                          Configuración de RViz
├── test/                          Tests (pytest)
└── urdf/                          Descripción del robot en Xacro
    ├── staubli.urdf.xacro         Fichero raíz: qué se genera
    ├── staubli_macro.xacro        Macro que crea links y joints del brazo
    └── include/                   Helpers, cámara, colores y materiales
```

Los tres modelos soportados son `tx2_60`, `tx2_60l` y `tx2_60l_med`. Todos comparten las mismas
mallas (`meshes/tx2_60/`) y se diferencian en los números de `config/`.

---

## 2. `urdf/` — la descripción del robot

### 2.1 `urdf/staubli.urdf.xacro` — el fichero raíz

Es el **único punto de entrada** del modelo. Si ejecutas xacro sobre este fichero obtienes el URDF
completo que consumen el resto de herramientas:

```bash
xacro urdf/staubli.urdf.xacro robot_model:=tx2_60l > robot.urdf
```

Un Xacro es un XML que, antes de ser válido como URDF, se procesa con el programa `xacro`: este
evalúa `${...}`, resuelve `<xacro:include>`, `<xacro:arg>`, `<xacro:property>` y las macros, y
devuelve un XML "plano" que ya es URDF. Por eso `package.xml` declara `xacro` y `urdf` como
dependencias.

Qué contiene, en orden de lectura:

1. **Argumentos** (`urdf/staubli.urdf.xacro:12-18`): `robot_model` (por defecto `???`, es decir,
   **hay que indicar obligatoriamente un modelo**), `name` (por defecto `staubli`), `robot_prefix`
   (vacío), `robot_ip` (por defecto `0.0.0.0`) y tres parámetros de la cámara del láser
   (`laser_distance`, `laser_width`, `laser_line_width`).
2. **Rutas a la configuración** (`:24-32`): construye `config/$(arg robot_model)` y de ahí toma
   `robot_model.yaml`, `kinematics.yaml`, `dynamics.yaml`, `joint_limits.yaml` y `meshes.yaml`.
   `initial_positions.yaml` se toma siempre de la raíz de `config/`, sin depender del modelo.
3. **Parámetros de seguridad y de control** (`:35-50`): `use_soft_limits`, márgenes de seguridad,
   `use_mock_hardware`, `use_gazebo_hardware` y `gazebo_controllers_file`.
4. **Link `world`** (`:56`): el link raíz, vacío. Existe para poder mover el robot entero o tener
   un frame de referencia del mundo, como hace RViz.
5. **El brazo**: incluye `staubli_macro.xacro` y `staubli_camera.xacro` y llama a las macros
   `xacro:staubli_robot` (`:61-79`) y `xacro:staubli_camera` (`:82-88`).
6. **El bloque de control**: incluye `r2c/staubli.r2c.xacro` y llama a `xacro:staubli_ros2_control`
   (`:91-102`).
7. **El plugin de Gazebo** (`:111-120`): solo si `use_gazebo_hardware:=true` se añade el tag
   `<gazebo>` con `libgz_ros2_control-system.so`.

El resultado verificado para `robot_model:=tx2_60l` es un URDF con **10 links y 15 joints**, con
esta cadena de parenting:

```
world → base_link → link_1 → link_2 → link_3 → link_4 → link_5 → link_6
                                                                  ├── tool0
                                                                  └── camera_link
```

Los 15 joints son: `base_joint` (fijo), `joint_1`…`joint_6` (revolutos),
`end_effector_fixed_joint` (fijo, crea `tool0`), `camera_fixed_joint` (fijo, crea `camera_link`) y
los 6 `<joint>` adicionales queROS 2 Control declara dentro de `<ros2_control>` (no son joints
cinemáticos, sino declaraciones de interfaces).

### 2.2 `urdf/staubli_macro.xacro` — el generador del brazo

Contiene una única macro, `staubli_robot` (`urdf/staubli_macro.xacro:6-105`), que es la que crea
todos los links y joints del brazo a partir de los ficheros YAML.

Cómo funciona, paso a paso:

- **Carga la configuración** (`:25-30`): invoca los seis "loaders" definidos en
  `staubli_helpers.xacro`, que se charge cada YAML en propiedades de xacro disponibles en el
  ámbito superior (`scope="parent"`).
- **Crea los links** recursivamente: la macro interna `add_all_links` (`:34-43`) va sacando
  nombres de la lista `link_names` y llama a `xacro:add_link` por cada uno. La recursión acaba
  cuando la lista queda vacía.
- **Crea el joint de la base** (`:54-58`): `base_joint`, de tipo `fixed`, que une el link padre
  (por defecto `world`) con `base_link`. El `origin` de este joint se recibe como bloque
  `<xacro:insert_block name="origin"/>` desde quien llama la macro, de forma que quien invoque
  `staubli_robot` decide dónde se coloca el robot en el mundo.
- **Crea las articulaciones** recursivamente (`:62-82`): aquí está el truco más importante para
  entender el código. `add_all_joints` mantiene **dos** listas y va haciendo `pop(0)` en ambas:

  ```python
  joint_name      = joints_list.pop(0)     # p. ej. "joint_1"
  parent_link_name = link_list.pop(0)      # p. ej. "base_link"
  child_link_name  = link_list[0]          # el siguiente link de la lista
  ```

  Como la lista de links es `base_link, link_1, ..., link_6`, el resultado es que cada articulación
  une el link actual con el siguiente: `joint_1` une `base_link`→`link_1`, `joint_2` une
  `link_1`→`link_2`, etc. **El orden de las listas de `default_names.yaml` define la cadena
  cinemática.** Si se cambiara el orden, el robot se deformaría.
- **Frame del sensor de par/fuerza** (`:85-94`): si `has_ft_sensor` es verdadero, crea el link
  `ft_sensor` unido al link padre configurado. En los tres modelos entregados vale `false`, así
  que **el URDF de fábrica no incluye este frame**; el código lo soporta pero está desactivado.
- **Frame del efector final** (`:98-103`): crea `tool0` unido al link padre configurado con la
  transformación `xyz`/`rpy` del modelo. En `tx2_60l` es `link_6` con `xyz = 0 0 0.07`.

### 2.3 `urdf/include/staubli_helpers.xacro` — las utilidades que hacen el trabajo pesado

Es el fichero con más lógica del paquete. Contiene cuatro tipos de cosas:

**a) Un `fail` a medida** (`:11-16`). Xacro no tiene una función de error propia; esta macro usa
`xacro.fatal()` dentro de un `xacro:if` para abortar la generación con un mensaje.

**b) Los "loaders" de configuración** (`:21-188`). Cada YAML se lee con `xacro.load_yaml()` y se
exporta a propiedades:

| Macro | Lee | Exporta |
|---|---|---|
| `load_default_names` | `urdf/include/default_names.yaml` | `joint_names`, `link_names`, nombres de frame de F/T y de efector |
| `load_robot_model_config` | `robot_model.yaml` | capacidades, número de E/S, datos del efector y del sensor F/T |
| `load_kinematics_config` | `kinematics.yaml` | `kinematics_config` |
| `load_dynamics_config` | `dynamics.yaml` | `dynamics_config` |
| `load_joint_limits_config` | `joint_limits.yaml` | `joint_limits_config` |
| `load_initial_positions_config` | `initial_positions.yaml` | `initial_positions` |
| `load_meshes_config` | `meshes.yaml` | `meshes_config` |

`load_robot_model_config` hace además dos cosas destacadas: llama a `load_default_names`
(`:38`), que es quien realmente aporta los nombres de links y joints, y al final valida la
configuración con `check_robot_model_config` (`:157`).

**c) Las validaciones** (`:211-252`). Comprueban que las listas no estén vacías, que haya
exactamente un link más que joints (`num_joints + 1`, `:231-234`) y que el link padre del
efector final sea coherente: el último link, o el frame del sensor F/T si existe (`:236-251`).
Esto hace que un error en los YAML falle durante la generación del URDF, con un mensaje claro, en
lugar de producir un modelo roto.

**d) Las macros que construyen el URDF**:

- `add_link` (`:257-319`): por cada link genera `<visual>`, `<collision>` e `<inertial>`.
  - *Visual*: malla + `origin` opcional (si `meshes.yaml` no define `origin`, se usa `0 0 0`) y,
        si el YAML indica `material`, un `<material>` con el amarillo Staubli; si no, no añade
        material y se ve la textura propia de la malla.
  - *Colisión*: malla de `collision` con su `origin`.
  - *Inercial*: masa, centro de masas e inercias tomados de `dynamics.yaml`.
- `get_mesh` (`:190-199`): resuelve la ruta de la malla según `force_abs_paths`. Por defecto
  (`false`) genera `package://staubli_robot_description/meshes/...`; con `true` genera
  `file://$(find ...)/...`, que es lo que necesitan algunos converters que no resuelven el
  esquema `package://`.
- `add_joint` (`:324-389`): genera un joint `revolute` con `parent`/`child`, `origin` desde
  `kinematics.yaml`, `axis` y `<limit>` con posición, velocidad y esfuerzo. Si `use_soft_limits`
  es verdadero añade un `<safety_controller>` con límites blandos obtenidos restando
  `safety_position_margin` a los límites duros (`staubli_helpers.xacro:379-386`).

### 2.4 `urdf/include/staubli_camera.xacro` — la cámara en mano

Define la macro `staubli_camera` (`:23-121`), que monta una **cámara RGB sobre la muñeca**
(*eye-in-hand*):

- Un link `camera_link` con geometría primitiva (una caja para el cuerpo, otra para la carcasa y
  un cilindro para la lente) y masa declarada (`:38-90`).
- Un joint fijo `camera_fixed_joint` cuyo padre es el link que se le pase (en
  `staubli.urdf.xacro:84` es `link_6`, de ahí "eye-in-hand").
- Un tag `<gazebo reference="camera_link">` con un `sensor type="camera"` que publica en el topic
  de Gazebo `/camera/image_raw`, a 30 Hz y 640×480 (`:100-119`).

Añade además dos visuales "ayuda": un triángulo (malla `laser_triangle.obj`, escalada con
`laser_distance` y `laser_width`) y una línea roja, que representan el cono y el haz de un láser
alineado con la cámara. Son geometría decorativa: no son sensores ni colisión.

Detalle a tener en cuenta: el comentario de cabecera del fichero (`:6-11`) habla de un `rpy`
`0.7031 0 0`, mientras que el valor por defecto real del parámetro `rpy` de la macro es
`2.9391 -1.3969 -2.9714` (`:27`), que es el que se aplica porque `staubli.urdf.xacro` no lo
sobreescribe. La intención del autor respecto a cuál de los dos es el correcto es **DESCONOCIDO**
a partir de los ficheros.

### 2.5 `urdf/include/staubli_colors.xacro` y `urdf/include/staubli_materials.xacro`

- `staubli_colors.xacro` define seis colores con nombres de referencia RAL (`color_staubli_yellow`,
  `color_staubli_white`, `color_staubli_light_grey`, `color_staubli_telegrey_1`, `..._4`,
  `..._grey_aluminium`).
- `staubli_materials.xacro` (que incluye el anterior) convierte cada color en un macro
  `material_staubli_*`.

Verificado en el código: **ninguno de esos macros de material se usa**. El único color que
efectivamente llega al URDF es `color_staubli_yellow`, y solo cuando `meshes.yaml` declara una
clave `material` para el link. `staubli_materials.xacro` sí se incluye en
`staubli_helpers.xacro:5`, de modo que la cadena de includes existe; el consumo de los macros
colores, no.

### 2.6 `urdf/include/default_names.yaml`

Es la fuente de la verdad sobre **cómo se llama cada pieza**:

```yaml
joint_names:  [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]
link_names:   [base_link, link_1, link_2, link_3, link_4, link_5, link_6]
ft_sensor_frame: ft_sensor
ee_frame: tool0
```

Estos nombres son importantes más allá del URDF: son los mismos que usa el driver
(`staubli_robot_driver`) para sus mensajes y sus buffers, y los que aparecen en los topics de
`ros2_control`. `tool0` es la convención estándar de ROS 2 para el frame del efector final.

---

## 3. `meshes/` — la geometría 3D

Esta carpeta contiene los ficheros de malla en dos formatos:

| Formato | Para qué se usa en este paquete |
|---|---|
| `.dae` (Collada) | Mallas **visuales** de los links. Admiten textura y color, que es lo apropiado para "ver" el robot. |
| `.stl` (STL) | Mallas de **colisión**. Es un formato sin materiales ni color, barato de calcular, que es lo que los motores de física necesitan. También se usa como visual en dos casos concretos (ver más abajo). |
| `.obj` + `.mtl` | Solo la malla `laser_triangle`, referenciada directamente desde `staubli_camera.xacro:74`. |

Distribución:

- `meshes/tx2_60/visual/`: `base_link.dae`, `link_1.dae`, `link_2_L.dae`, `link_2.stl`,
  `link_3.dae`, `link_4_L.dae`, `link_4.stl`, `link_5.dae`, `link_6.dae`, `link_6_L_med.dae`.
- `meshes/tx2_60/collision/`: los `.stl` correspondientes, incluidos `link_2_L.stl`,
  `link_4_L.stl` y `link_6_L_med.stl`.
- `meshes/laser_triangle.obj` / `.mtl`: el triángulo del láser de la cámara.

**Los tres modelos de robot comparten el mismo directorio de mallas** y distinguen su variante con
el sufijo del fichero. `meshes/tx2_60/README.md` lo explica: para el TX2-60 estándar se usan los
ficheros sin sufijo, para el TX2-60L los `*_L.*` y para el TX2-60L-med los `*_L_med.*`.

**Visual frente a colisión, y por qué importa.** En el URDF, `<visual>` es lo que se dibuja y
`<collision>` es lo con lo que se calculan los contactos. Si se usara la malla visual como
colisión, la simulación sería más exacta visualmente pero mucho más lenta y con más riesgo de
atascos, porque una malla texturizada tiene muchísimos triángulos. Aquí la separación es clara:
cada link tiene su `.dae` para visualizar y su `.stl` para colisionar, y quien decide el par es
`meshes.yaml` (leído por `get_mesh` en `staubli_helpers.xacro:190-199`).

La única excepción, y está documentada en `meshes/tx2_60/README.md:5-7`, es que en el TX2-60
estándar los links 2 y 4 solo existen como STL, así que `config/tx2_60/meshes.yaml` los usa como
visual **y** como colisión. En esos dos casos el URDF generado lleva además un `<material>` con el
amarillo Staubli, precisamente porque un STL no tiene textura propia.

---

## 4. `launch/` — solo visualización

### `launch/view_robot.launch.py`

Es el **único** launch del paquete, y está orientado exclusivamente a **visualizar** el robot.

| Aspecto | Detalle |
|---|---|
| Qué inicia | Un `joint_state_publisher_gui`, un `robot_state_publisher` y un `rviz2`. |
| Nodos que lanza | `joint_state_publisher_gui` (`:73-76`), `robot_state_publisher` (`:78-83`), `rviz2` (`:85-91`). |
| Configuración que usa | `urdf/staubli.urdf.xacro` y, por defecto, `rviz/view_robot.rviz`. |
| Parámetros | `robot_model`, obligatorio, con `choices=["tx2_60","tx2_60l","tx2_60l_med"]` (`:27-37`). `rviz_config_file`, opcional, con valor por defecto (`:39-47`). |
| Situación en la que se usa | Ver el robot en RViz y mover las articulaciones a mano con la GUI. |

El flujo es el estándar de ROS 2 para visualizar sin hardware:

1. El launch ejecuta `xacro` como comando externo (`Command` + `FindExecutable`, `:57-66`) y
   captura la salida.
2. Esa salida se inyecta como parámetro `robot_description` del `robot_state_publisher`
   (`:67-69`), que es el nodo que **publica `/robot_description`** y, a partir de él, calcula y
   publica la árbol de transforms en `/tf`.
3. `joint_state_publisher_gui` publica `/joint_states` a partir de los sliders de su ventana, y con
   eso `robot_state_publisher` mueve los TF.
4. `rviz2` se lanza con `-d rviz_config_file` para abrir directamente con la configuración del
   paquete.

**Lo que este launch NO hace**, y es importante para no confundirlo con otras cosas:

- **No simula.** No arranca Gazebo ni ningún motor de física. El URDF que se genera sí contiene el
  bloque `<ros2_control>` con el plugin `mock_components/GenericSystem`, pero **este launch no
  lanza `ros2_control_node`**, así que ese plugin no llega a instanciarse. Es decir: aquí el robot
  se mueve porque tú arrastras los sliders, no porque exista un controlador ni un modelo dinámico.
- **No controla nada real.** El URDF se genera con `use_mock_hardware` por defecto, y el launch ni
  siquiera le pasa ese argumento, de modo que se usa el valor por defecto (`true`).
- **No usa el driver de Staubli.** El plugin real solo se selecciona con
  `use_mock_hardware:=false`, y ningún fichero de `launch/` en este paquete lo hace.

Las combinaciones reales (mock, Gazebo, hardware) se montan en el paquete hermano
`staubli_bringup`, que contiene `launch_gazebo.launch.py` y `launch_robot_control.launch.py`. En
esos ficheros se ve que el launch de Gazebo invoca este mismo Xacro con
`use_gazebo_hardware:=true` y `gazebo_controllers_file:=...`, y el de control con
`use_mock_hardware:=...` y `robot_ip:=...`. Ese otro paquete **no** forma parte del análisis de
este documento, pero los nombres son verificables en el repositorio.

---

## 5. `config/` — los números del robot

Aquí no hay lógica, solo datos. La gracia del diseño es que **cambiar de modelo no requiere tocar
nada de código**: se cambia `robot_model` y el Xacro lee otro directorio.

### `config/initial_positions.yaml`

Un único fichero para los tres modelos. Declara las seis posiciones iniciales a cero. Lo consume
`r2c/include/staubli_r2c_joints.xacro:38-40` como `initial_value` de las `state_interface` de
posición, es decir, el valor con el que `ros2_control` inicializa el estado antes de que llegue
el primer mensaje del robot.

### `config/<modelo>/robot_model.yaml`

Describe las capacidades del modelo, no su geometría:

- `has_joint_torque_sensor` y `has_ft_sensor`: ambos `false` en los tres modelos. Controlan si se
  crean la `state_interface` de esfuerzo y el link del sensor F/T.
- `end_effector`: link padre (`link_6` en todos) y la transformación `xyz`/`rpy` hacia `tool0`.
  Aquí sí difieren los modelos: `0.07` en `tx2_60` y `tx2_60l`, y `0.1344` en `tx2_60l_med`.
- `io`: número de entradas y salidas digitales y analógicas (16/16/2/2 en los tres modelos). Este
  dato viaja hasta el bloque `<gpio>` de `ros2_control` y hasta los parámetros del driver real.
- Un bloque `ft_sensor` comentado, que sirve de plantilla si se quisiera activar el sensor F/T.

### `config/<modelo>/kinematics.yaml`

Por cada joint: la posición `xyz` respecto al link padre, la orientación `rpy` y el `axis` de giro.
Es la fuente de la geometría de la cadena cinemática.

Comparación entre modelos (verificada con `diff`):

| | `tx2_60` | `tx2_60l` / `tx2_60l_med` |
|---|---|---|
| `joint_3` xyz | `0 0.00925 0.29` | `0 0.00925 0.40` |
| `joint_5` xyz | `0 0 0.222` | `0 0 0.362` |

Es decir, la versión "L" (brazo largo) tiene los mismos ejes pero tramos más largos. `joint_1` y
`joint_6` coinciden en los tres modelos.

### `config/<modelo>/dynamics.yaml`

Masa, centro de masas (en `xyz` y `rpy`) y tensor de inercia de cada link. Va directo al bloque
`<inertial>` del URDF (`staubli_helpers.xacro:303-317`). **Aviso importante para un principiante**:
los valores de este fichero son genéricos: `base_link` pesa 5.0, todos los demás links 1.0, y todas
las inercias valen `0.1` con centros de masa en el origen. Son valores de relleno coherentes
para que el URDF sea válido, no una identificación real del robot. El fichero es idéntico en los
tres modelos salvo el comentario del título. Si vas a simular en Gazebo y te importa la dinámica,
este es el fichero que habría que sustituir por datos reales.

### `config/<modelo>/joint_limits.yaml`

Límites por articulación. Usa una extensión de YAML propia de xacro: `!degrees 180.0`. Xacro
(rescritura de Python) la entiende y la convierte a radianes; un lector YAML estándar fallaría con
este fichero. En el URDF generado los límites quedan en radianes, como exige URDF
(por ejemplo, `joint_1` pasa de `±180°` a `±3.14159...`).

Cada joint trae `has_position_limits`, `has_velocity_limits`, `has_acceleration_limits`,
`has_effort_limits`, `min_position`, `max_position`, `max_velocity`, `max_acceleration` y
`max_effort`. La única diferencia entre modelos está en `tx2_60l_med`, que tiene
`has_effort_limits: false` en las seis articulaciones, por lo que `add_joint` escribe
`effort="0.0"` (`staubli_helpers.xacro:362-364`).

Dos consumidores distintos:
- `add_joint` usa `min_position`, `max_position`, `max_velocity` y `max_effort` para el `<limit>`.
- `staubli_r2c_joints.xacro:24-31` usa `max_acceleration` para los límites de la interfaz de
  comando `acceleration`.

### `config/<modelo>/meshes.yaml`

Traduce cada link a sus dos mallas:

```yaml
meshes:
  base_link:
    visual:   {package: staubli_robot_description, path: meshes/tx2_60/visual/base_link.dae}
    collision: {package: staubli_robot_description, path: meshes/tx2_60/collision/base_link.stl}
```

Y opcionalmente añade `origin` (desplazamiento de la malla respecto al link) y `material`.
Los tres ficheros están en el mismo directorio `meshes/tx2_60/`, así que **lo que cambia entre
modelos es el sufijo elegido**: `tx2_60` usa `link_2.stl`/`link_4.stl` (con `origin` y `material`
`staubli_yellow`), mientras que `tx2_60l` usa `link_2_L.dae`/`link_4_L.dae`, y `tx2_60l_med` usa
`link_6_L_med.*` para el último link.

---

## 6. `rviz/` — la configuración de la vista

### `rviz/view_robot.rviz`

Es la configuración guardada de RViz que el launch pasa con `-d`. Contiene:

- **Frame fijo `world`** (`:166`), que coincide con el link raíz creado en el Xacro.
- **Display `Grid`** (`:33-50`): rejilla de referencia en el plano XY.
- **Display `TF`** (`:51-93`): muestra los ejes de los frames. Tiene activo `base_link`, `tool0` y
  `world`, y apagados los `link_1`…`link_6` para no saturar la vista. El árbol guardado refleja
  la cadena `world → base_link → … → link_6 → tool0`; `camera_link` no figura en la lista.
- **Display `RobotModel`** (`:94-162`): es el que dibuja el robot. La clave está en
  `Description Source: Topic` y `Description Topic: /robot_description` (`:98-104`): RViz no lee
  el fichero, espera a que alguien publique el URDF, que es justo lo que hace
  `robot_state_publisher`. Tiene `Collision Enabled: false`, es decir, **solo muestra la malla
  visual**.
- **Herramientas** (`:169-201`): `Interact`, `MoveCamera`, `Select`, `FocusCamera`, `Measure`,
  `SetInitialPose`, `SetGoal` y `PublishPoint`, cada una con su topic por defecto.
- **Vista `Orbit`** con la distancia y el ángulo de cámara guardados (`:207-227`).

### Qué papel tiene RViz aquí, y qué no

Conviene separar tres conceptos que se confunden con frecuencia:

| Concepto | Qué es | ¿En este paquete? |
|---|---|---|
| **Visualizar** | Dibujar el modelo y sus transforms, sin física ni control. | Sí. Es la función de `view_robot.launch.py` + este `.rviz`. |
| **Simular** | Calcular física: gravedad, colisiones, dinámica de las articulaciones. | No directamente. La simulación la aporta Gazebo, que se lanza desde `staubli_bringup`. Aquí solo se prepara el URDF con los tags `<gazebo>` necesarios. |
| **Controlar** | Calcular y enviar órdenes de movimiento al robot. | No. Ni RViz ni este paquete mandan órdenes a las articulaciones. |

En concreto, este paquete no incluye ningún display de `ros2_control` ni de `joint_states`, ni el
plugin `rviz_ros` paraumeric; RViz aquí es puramente un visor del modelo.

---

## 7. `r2c/` — la descripción de `ros2_control`

`r2c` significa, por el nombre de los ficheros, **"ros2 control"**. Es la parte de la descripción
que declara **qué interfaces de control existen** y **qué plugin de hardware las implementa**.
`ros2_control` no adivina nada: lee este bloque del URDF.

### `r2c/staubli.r2c.xacro`

Es el punto de entrada, con una macro `staubli_ros2_control` (`:8-120`) que inserta el bloque
`<ros2_control name="..." type="system">` con dos secciones: `<hardware>` y las declaraciones de
articulaciones y GPIOs.

**La selección de hardware** (`:26-59`) es la pieza más importante de todo el paquete para
entender el proyecto, porque es el punto donde se decide contra qué se está hablando:

| Condición | Plugin seleccionado | Para qué sirve |
|---|---|---|
| `use_gazebo_hardware:=true` | `gz_ros2_control/GazeboSimSystem` | Conectar `ros2_control` con el motor de física de Gazebo Sim. |
| `use_gazebo_hardware:=false` y `use_mock_hardware:=true` | `mock_components/GenericSystem` | Simulación de `ros2_control` sin física ni hardware: sigue órdenes y devuelve estado. |
| Ambos a `false` | `staubli_robot_driver/StaubliHardwareInterface` | El robot real, hablado por UDP con el controlador CS9. |

En el último caso se pasan los parámetros que necesita el driver real (`:47-57`): `robot_prefix`,
`robot_ip`, los puertos `11000` (control) y `11001` (diagnóstico) — tanto remoto como local — y el
número de E/S, que viene del `robot_model.yaml`. Estos son los mismos puertos que configura la
aplicación VAL3 del controlador, así que la coherencia entre ambos extremos está garantizada desde
la descripción.

**Control de aceleración** (`:64-79`): la interfaz de comando `acceleration` solo se declara para el
hardware real, porque sirve para fijar límites de aceleración. Los comentarios del fichero
explican que ni `GazeboSimSystem` ni el mock la exponen.

**GPIOs** (`:90-118`): se declaran dos bloques `<gpio>`. Uno llamado `supervision`, con nueve
interfaces de solo lectura (`operation_mode`, `safety_status`, `brakes_released`, `in_motion`,
`estop_pressed`, etc.) que el driver rellena con el estado de seguridad del robot, y otro
generado con las E/S reales. **Ambos se omiten cuando `use_gazebo_hardware:=true`**, porque
`GazeboSimSystem` no soporta interfaces GPIO.

Lo que este paquete **no** contiene son los `<controller>` (`joint_state_broadcaster`,
`joint_trajectory_controller`): aquí solo se declara el hardware y las interfaces. Los ficheros de
controladores están en `staubli_moveit_config/config/moveit_controllers.yaml`, en otro paquete.

### `r2c/include/staubli_r2c_joints.xacro`

Macro `r2c_joints_control_description` (`:4-61`). Por cada articulación del robot declara, dentro
de `<ros2_control>`:

- `command_interface` de `position` y de `velocity` (esta última con valor inicial 0.0).
- `command_interface` de `acceleration`, solo si `include_acceleration_command` es verdadero
  (`:24-31`), con límites `max`/`min` tomados de `joint_limits.yaml`.
- `command_interface` de `effort`, solo si `has_joint_torque_control` es verdadero. En
  `staubli.r2c.xacro:86` se pasa siempre `false`, por lo que **esta interfaz no se genera en la
  práctica**.
- `state_interface` de `position` (con valor inicial de `initial_positions.yaml`), de `velocity`, y
  de `effort` si `has_joint_torque_sensor` es verdadero (hoy `false` en los tres modelos).

Se generan con la misma técnica recursiva que la descripción cinemática: una macro interna que
consume la lista `joint_names` de uno en uno.

### `r2c/include/staubli_r2c_gpios.xacro`

Macro `r2c_gpios_control_description` (`:4-73`). Genera el bloque `<gpio name="gpio">` con
interfaces numeradas a partir de los contadores de `robot_model.yaml`:

| Prefijo | Tipo | Cantidad |
|---|---|---|
| `digital_input_N` | solo estado | `num_digital_inputs` (16) |
| `digital_output_N` | solo comando | `num_digital_outputs` (16) |
| `analog_input_N` | solo estado | `num_analog_inputs` (2) |
| `analog_output_N` | solo comando | `num_analog_outputs` (2) |

Nótese que la convención es coherente con lo que espera el driver: en
`staubli_hardware_interface.cpp` los arrays de E/S tienen esos tamaños.

---

## 8. `env-hooks/` — variables de entorno para Gazebo

### `env-hooks/staubli_robot_description.sh.in`

Tiene tres líneas, y las tres hacen lo mismo:

```bash
ament_prepend_unique_value IGN_GAZEBO_RESOURCE_PATH "<prefix>/share"
ament_prepend_unique_value GAZEBO_MODEL_PATH              "<prefix>/share"
ament_prepend_unique_value GZ_SIM_RESOURCE_PATH           "<prefix>/share"
```

**Qué son los environment hooks.** En ROS 2, cuando haces `source install/setup.bash`, no solo se
añaden las librerías al `LD_LIBRARY_PATH`: también se sourcedan unos scripts "gancho" que cada
paquete puede registrar para modificar variables de entorno. Se registran con
`ament_environment_hooks(...)` en el `CMakeLists.txt:24`, y se instalan y ejecutan automáticamente
al hacer `source` del workspace. El `.sh.in` es una plantilla: `@CMAKE_INSTALL_PREFIX@` se
sustituye por la ruta real de instalación durante el build.

**Por qué este paquete necesita uno.** Los tres paths son rutas donde Gazebo busca modelos y
recursos. Al añadir el `share` del paquete, cualquier modelo o recurso que se deje en ese
directorio pasa a ser visible para Gazebo (y para Ignition, que usa otro nombre para lo mismo).

**Cuándo se ejecuta.** Solo al hacer `source` del `setup.bash` / `setup.zsh` del workspace, una vez
por terminal. No tiene ningún efecto en tiempo de ejecución del robot.

Es una prueba, por cierto, de que este paquete **sí contempla la integración con Gazebo**, aunque
no la ejecute: prepara el entorno para que la conversión URDF→SDF y la carga del modelo funcionen.

---

## 9. `test/`

Hay dos ficheros, pero solo uno está registrado como test.

### `test/test_default_urdf.py` — el test real

Es un test de **descripción**: verifica que el URDF se genera correctamente. Está registrado en el
`CMakeLists.txt:20` con el nombre `test_default_urdf_xacro_parsing` y usa `ament_add_pytest_test`.

Qué hace, para cada combinación de parámetros:

1. Localiza el Xacro instalado con `get_package_share_directory`.
2. Ejecuta `xacro` con esos argumentos y redirige la salida a un fichero temporal.
3. Ejecuta `check_urdf` sobre ese fichero, que es la herramienta estándar de ROS 2 para validar
   que un URDF es sintácticamente correcto y que su árbol de links es consistente.
4. Comprueba que ambos comandos terminan con código 0 y borra el fichero temporal.

Combina tres parámetros (`@pytest.mark.parametrize`): los tres modelos, dos prefijos y dos valores
de `use_mock_hardware`, es decir **12 casos**. Es una cobertura muy barata y muy valiosa: si
alguien toca un YAML y rompe la geometría, el test falla.

Detalle verificado: el test pasa el argumento `prefix:=...`, pero el Xacro declara el argumento
como `robot_prefix` (`staubli.urdf.xacro:14`). Xacro acepta el argumento sobrante sin error, pero
**no tiene ningún efecto**: al generar con `prefix:=staubli_` los links siguen llamándose
`link_1`, sin prefijo. Con `robot_prefix:=arm_` sí se obtiene `arm_link_1`. Es decir, la
parametrización del prefijo en el test no está probando nada actualmente. Si el prefijo se quiere
usar de verdad, el argumento correcto es `robot_prefix`.

### `test/test_launch_mocked.py` — un marcador de posición

Contiene únicamente la cabecera de licencia y un `TODO(tpoignonec): launch mocked hardware for tests`
(`test/test_launch_mocked.py:17`). No prueba nada, y **no está registrado** en el
`CMakeLists.txt`, así que nunca se ejecuta. Es la intención declarada de añadir en el futuro un
test de launch con hardware simulado.

### `test/__pycache__/`

Contiene los `.pyc` compilados de ambos tests. Es un artefacto de haber ejecutado pytest localmente,
no código fuente. Se menciona aquí porque `CMakeLists.txt:10-13` instala el directorio `test/`
completo, y por tanto estos ficheros también acaban en el `share` instalado.

---

## 10. `package.xml` y `CMakeLists.txt`

### `package.xml`

Es un paquete `ament_cmake` puro: **no compila una sola línea de C++**. Todas las dependencias son
`<exec_depend>`, porque lo único que se hace en tiempo de ejecución es lanzar herramientas:

| Dependencia | Para qué se necesita |
|---|---|
| `launch`, `launch_ros` | El sistema de ficheros de lanzamiento de ROS 2, usado por `launch/view_robot.launch.py`. |
| `robot_state_publisher` | Publica `/robot_description` y el árbol de TF. |
| `joint_state_publisher_gui` | GUI para mover las articulaciones a mano en el launch de visualización. |
| `rviz2` | El visualizador 3D. |
| `urdf` | El parser de URDF y la herramienta `check_urdf` que usa el test. |
| `xacro` | El preprocesador que convierte los `.xacro` en URDF. |

Como `<test_depend>` aparecen `ament_lint_auto`, `ament_lint_common`, `launch`, `urdf` y `xacro`.

Como dato revelador: no se declara `ros2_control`, `hardware_interface`, `ros_gz` ni `gz_ros2_control`
aunque el URDF los mencione. Eso es correcto en un paquete de descripción: quien **consume** esas
funcionalidad es el paquete que lanza `ros2_control_node` o Gazebo, y es ese paquete el que declara
la dependencia.

### `CMakeLists.txt`

Es un CMake de 25 líneas, muy simple:

- `install(DIRECTORY config launch meshes urdf r2c rviz test DESTINATION share/${PROJECT_NAME})`
  (`:10-13`): copia las seis carpetas al `share` del paquete instalado. **Ese `share` es el que
  usan los `$(find staubli_robot_description)` del Xacro, el `FindPackageShare` del launch y el
  `get_package_share_directory` del test.** No hay ninguna regla de compilación: no hay
  `add_library`, ni `add_executable`, ni `ament_python_install_package` (el paquete no tiene
  código Python importable, solo tests).
- `if(BUILD_TESTING)` (`:16-21`): activa `ament_cmake_pytest` y registra el único test.
- `ament_environment_hooks(...)` (`:24`): registra el script de `env-hooks/`.

Relación con las carpetas: la instalación es plana, así que en el workspace instalado queda
`share/staubli_robot_description/{config,launch,meshes,r2c,rviz,test,urdf}`, replicando la
estructura del repositorio. Y como `meshes/` se instala dentro del paquete, las rutas
`package://staubli_robot_description/meshes/...` que genera el Xacro son resolubles por RViz y por
los conversores de Gazebo.

---

# Cómo encaja todo

## 11. El flujo del paquete

```
config/<modelo>/*.yaml  ──────────────┐
urdf/include/default_names.yaml  ─────┤
                                      ▼
                          urdf/staubli.urdf.xacro   (punto de entrada)
                                      │  xacro:arg robot_model
                                      ├──▶ urdf/staubli_macro.xacro
                                      │        └──▶ urdf/include/staubli_helpers.xacro
                                      │                 └──▶ staubli_colors.xacro
                                      │                      (vía staubli_materials.xacro)
                                      ├──▶ urdf/include/staubli_camera.xacro
                                      └──▶ r2c/staubli.r2c.xacro
                                               ├──▶ r2c/include/staubli_r2c_joints.xacro
                                               └──▶ r2c/include/staubli_r2c_gpios.xacro
                                      │
meshes/**  ───────────────────────────┤ (referenciadas como package:// desde meshes.yaml)
                                      ▼
                        URDF completo  (links, joints, <ros2_control>, tags <gazebo>)
                        /            |                        \
```

## 12. Quién hace qué, según los ficheros

| Pregunta | Respuesta demostrable en los ficheros |
|---|---|
| **¿De dónde sale el modelo del robot?** | De `urdf/staubli.urdf.xacro`, expandido con `xacro` y con los datos numéricos de `config/<modelo>/`. El URDF resultante se puede volcar a fichero o pasar como parámetro. |
| **¿Quién publica los transforms?** | `robot_state_publisher`. Lo lanza `launch/view_robot.launch.py:78-83` con el URDF como parámetro `robot_description`; a partir de ahí publica `/tf`. |
| **¿Quién lo visualiza?** | `rviz2`, con `rviz/view_robot.rviz`, leyendo el URDF del topic `/robot_description` y los TF de `/tf`. |
| **¿Quién lo simula?** | Gazebo Sim, pero **no desde este paquete**. La integración está repartida: aquí están el tag `<gazebo>` del plugin `gz_ros2_control` (`staubli.urdf.xacro:111-120`), el sensor de cámara (`staubli_camera.xacro:100-119`) y el `env-hook` que registra `GZ_SIM_RESOURCE_PATH`. El proceso de Gazebo se lanza desde `staubli_bringup/launch/launch_gazebo.launch.py`, que es quien pasa `use_gazebo_hardware:=true` y el fichero de controladores. |
| **¿Quién lo controla?** | `ros2_control`, mediante el `ros2_control_node` que se lanza en otro paquete. Este paquete solo declara **qué** se puede controlar (articulaciones, GPIOs) y **con qué plugin**. |
| **¿Dónde entra `ros2_control`?** | En el bloque `<ros2_control>` que genera `r2c/staubli.r2c.xacro`, incluido desde `staubli.urdf.xacro:91-102`. |
| **¿Dónde entra Gazebo?** | En los tags `<gazebo>` (plugin de control y sensor de cámara) y en `env-hooks/`. |
| **¿Dónde entra RViz?** | En `launch/view_robot.launch.py:85-91` y el fichero `rviz/view_robot.rviz`. |
| **¿Dónde entra el driver de Staubli?** | En la rama de `r2c/staubli.r2c.xacro:46-58`: cuando `use_mock_hardware:=false`, el plugin es `staubli_robot_driver/StaubliHardwareInterface`, y este paquete le pasa la IP, los puertos y el número de E/S. El código del driver vive en el paquete hermano `staubli_robot_driver`. |

## 13. Los cinco planos, sin mezclarlos

1. **Descripción del robot** — `urdf/`, `r2c/`, `config/`, `meshes/`. Es todo el contenido del
   paquete: qué existe, cómo se llama, dónde está y qué límites tiene. No ejecuta nada.
2. **Visualización** — `launch/view_robot.launch.py` + `rviz/view_robot.rviz`. Dibuja el modelo y
   permite moverlo a mano. No calcula física ni manda órdenes.
3. **Simulación** — Gazebo Sim, lanzado desde `staubli_bringup`, alimentado por los tags `<gazebo>`
   de este paquete y por `env-hooks/`. Puede usar física real (`gz_ros2_control`) o el mock de
   `ros2_control` según el argumento.
4. **Control** — el bloque `<ros2_control>` y sus interfaces. Aquí solo se declaran; quien las
   activa es el `ros2_control_node` de otro paquete.
5. **Hardware físico** — el plugin `staubli_robot_driver/StaubliHardwareInterface`, cuyo código no
   está en este paquete. Este solo le proporciona, vía el URDF, la IP, los puertos y el número de
   entradas y salidas.

---

# RViz y Gazebo en este proyecto

| | RViz | Gazebo Sim |
|---|---|---|
| **Qué hace** | Visualiza el URDF y los transforms que publique `robot_state_publisher`. | Carga el modelo en un mundo físico, simula gravedad, contacto y dinámica, y ejecuta sensores. |
| **Quién lo lanza** | `launch/view_robot.launch.py` (este paquete). | `staubli_bringup/launch/launch_gazebo.launch.py` (otro paquete del repositorio). |
| **Qué necesita del URDF** | Los tags `<link>`, `<joint>`, `<visual>`. | Lo mismo, más `<collision>`, `<inertial>`, y los tags `<gazebo>` que se convierten en SDF. |
| **Modelo del robot** | El mismo: el mismo Xacro genera el URDF que ambos consumen. | Idem. La diferencia está en quién lo carga. |
| **Puede usarse sin el otro** | Sí, y es el caso normal: RViz funciona sin Gazebo. | Sí, Gazebo puede correr sin RViz abierto. |
| **Movimiento de las articulaciones** | Lo que tú arrastres en `joint_state_publisher_gui`. | Lo que diga el controlador; con `gz_ros2_control`, las órdenes de `ros2_control` mueven la física. |
| **Cámara** | No tiene ningún display de imagen en esta configuración. | `staubli_camera.xacro` declara el sensor; el launch de Gazebo en `staubli_bringup` añade además un puente del topic de Gazebo al de ROS (`/camera/image_raw`). |
| **Relación entre ellos** | En este paquete, ninguna: `view_robot.launch.py` no menciona Gazebo. La relación real es que **ambos consumen el mismo Xacro**, y cada uno con sus propios tags. |

Las dos cosas que no se deben confundir:

- **RViz no simula nada.** En el launch de este paquete, el URDF incluye `<ros2_control>` con el
  plugin `mock_components/GenericSystem`, pero como el launch no arranca `ros2_control_node`, ese
  bloque no se instancia. El movimiento que ves lo produces tú con los sliders.
- **Gazebo no sustituye al driver real.** Con `use_gazebo_hardware:=true`, el URDF deja de pedir
  `staubli_robot_driver/StaubliHardwareInterface` y pasa a pedir
  `gz_ros2_control/GazeboSimSystem`; no hay tráfico UDP ni controlador CS9 implicado. Con ambos
  argumentos a `false`, es el plugin del driver real el que se instancia.

---

# Archivos más importantes

En el orden en que tiene sentido estudiarlos:

1. **`urdf/include/default_names.yaml`** — el vocabulario del robot: cómo se llaman los links, los
   joints y los frames. Con 19 líneas se entiende qué partes existen. Leerlo primero evita perderse
   en el resto.
2. **`urdf/staubli.urdf.xacro`** — el fichero raíz. Muestra qué se incluye, qué argumentos existen y
   cómo se compone el resultado. Es el mapa de todo el paquete.
3. **`config/tx2_60l/robot_model.yaml`** — entender el fichero de capacidades (sensor F/T, sensor de
   par, número de E/S, efector final) explica por qué el URDF tiene o no tiene ciertas cosas.
4. **`urdf/staubli_macro.xacro`** — la macro que crea el brazo. Es donde se ve el truco de las dos
   listas que se consumen en paralelo para construir la cadena cinemática.
5. **`urdf/include/staubli_helpers.xacro`** — el fichero con la lógica real: loaders de YAML,
   validaciones, y las macros `add_link`, `add_joint` y `get_mesh`. Explica cómo los datos numéricos
   se convierten en XML.
6. **`config/tx2_60l/kinematics.yaml` y `config/tx2_60l/joint_limits.yaml`** — la geometría y los
   límites en su forma más legible. Comparar `tx2_60` y `tx2_60l` muestra físicamente qué cambia
   entre modelos.
7. **`meshes/tx2_60/README.md` y `config/tx2_60l/meshes.yaml`** — dos minutos para entender la
   separación visual/colisión y cómo una misma malla sirve a tres modelos.
8. **`launch/view_robot.launch.py`** — el pipeline estándar de visualización: xacro →
   `robot_description` → `robot_state_publisher` → `/robot_description` → RViz. Es elLaunch más
   pequeño y más completo del paquete.
9. **`rviz/view_robot.rviz`** — para ver de forma concreta qué se está visualizando: el frame fijo
   `world`, el display `RobotModel` leyendo del topic, y el árbol de TF.
10. **`r2c/staubli.r2c.xacro`** — el puente con `ros2_control`: la decisión entre mock, Gazebo y
    hardware real es el punto donde este proyecto se conecta con el resto del sistema.
11. **`r2c/include/staubli_r2c_joints.xacro` y `staubli_r2c_gpios.xacro`** — el detalle de qué
    interfaces se declaran por articulación y por E/S.
12. **`test/test_default_urdf.py`** — muestra cómo se prueba un paquete de descripción, y de paso
    documenta los valores válidos de los argumentos.
13. **`env-hooks/staubli_robot_description.sh.in`** — pequeño, pero explica la integración con
    Gazebo mejor que cualquier comentario.
14. **`CMakeLists.txt` y `package.xml`** — para entender que un paquete de descripción se instala,
    no se compila, y que el `share` es el punto de encuentro entre todos los consumidores.

---

# Resumen final

`staubli_robot_description` es el paquete que **define qué es el robot para el resto del sistema
ROS 2**. Su responsabilidad es exactamente una: mantener, en YAML y en Xacro, la descripción del
brazo Staubli (links, joints, geometría, límites, dinámica, mallas y sensor de cámara) y
declarar, en el bloque `<ros2_control>`, qué interfaces de control existen y qué plugin las
implementa.

No controla el robot, no habla con el controlador, no calcula física y no dibuja nada por sí solo.
Su valor está en ser **la fuente única de la que beben todos los demás**: `robot_state_publisher` y
RViz para verlo, Gazebo para simularlo, `ros2_control` para controlarlo y el driver
`staubli_robot_driver` para materializarlo contra el CS9 real. Por eso, elegir `robot_model` en el
launch es, en la práctica, elegir con qué robot se comportará todo lo demás.
