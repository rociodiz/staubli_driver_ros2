# Explicación del paquete `staubli_moveit_config`

Guía de estudio del paquete `staubli_moveit_config`. Está escrita para alguien que está
aprendiendo ROS 2 y MoveIt 2 y que quiere entender **qué papel juega este paquete dentro del
proyecto `staubli_driver_ros2`**, no solo qué ficheros contiene.

La idea central es sencilla: **`staubli_robot_description` dice qué es el robot;
`staubli_moveit_config` dice cómo se planifica con él**. El URDF describe la mecánica (links,
joints, geometría, límites). El SRDF y los YAML de este paquete describen la *semántica* (qué
conjunto de articulaciones forma "el brazo", qué pares de links nunca chocan, qué controlador
recibe la trayectoria) y los *parámetros de planificación* (qué solucionador de cinemática
inversa usar, con qué velocidades planificar).

Todo lo que sigue está contrastado con el contenido real de los ficheros de la carpeta, con el
código de `moveit_configs_utils` instalado en el sistema y con los ficheros de los demás
paquetes del repositorio que los consumen. Cuando algo no puede determinarse a partir de los
ficheros, se indica explícitamente.

---

## 1. Qué es `staubli_moveit_config` y qué papel tiene en el proyecto

### 1.1 Qué es MoveIt 2

**MoveIt 2** es el framework de referencia de ROS 2 para **planificación de movimiento** de
robots. Planificar movimiento es responder a una pregunta como: *"el robot está aquí, quiero que
llegue ahí, sin chocar y sin romper ningún límite"*. MoveIt se encarga del trabajo matemático
difícil que hay en medio:

- Resolver **cinemática inversa**: si me dices "quiero que `tool0` esté en esta pose", ¿qué valores
  deben tomar las seis articulaciones para llegar a ella? (Ver §4.4.)
- Buscar un **camino** en el espacio de configuración: una secuencia de valores de articulaciones
  que llegue al objetivo sin atravesar obstáculos ni salirse de los límites.
- **Temporalizar** ese camino: decidir a qué velocidad y con qué aceleración se recorre, para que
  el robot no se mueva a saltos ni de forma brusca.
- **Detectar colisiones** entre los links del propio robot (autocolisión) y entre el robot y los
  objetos de la escena.

MoveIt 2 no es un nodo de ROS 2, sino un conjunto de librerías y paquetes
(`moveit_core`, `moveit_planners`, `moveit_kinematics`, `moveit_ros_planning_interface`, ...). El
programa que las reúne y las pone a trabajar juntas se llama `move_group`, y es un **nodo** de
ROS 2. Se explica en §4.2.

### 1.2 El paquete en la arquitectura del repositorio

El repositorio está partido en paquetes con responsabilidades muy separadas. La cadena es esta:

| Paquete | Responsabilidad | Tipo de contenido |
|---|---|---|
| `staubli_robot_description` | **Qué es el robot**: geometría, mallas, límites, bloque `<ros2_control>`. | Xacro, YAML, mallas, un launch de visualización |
| **`staubli_moveit_config`** | **Cómo se planifica con el robot**: grupo de planificación, autocolisiones, cinemática inversa, límites para planear, mapeo a controladores, vista de planificación. | SRDF en Xacro, YAML, un launch, un test |
| `staubli_robot_driver` | **Cómo se habla con el robot real**: UDP, protocolo del CS9, `ros2_control` hardware interface. | C++ |
| `staubli_bringup` | **Cómo se arranca todo junto**: qué launch se incluye, en qué orden, y quién espera a quién. | Launch files y configuración de `controller_manager` |
| `staubli_trajectory` | Ejecución de trayectorias calculadas fuera de MoveIt (FK/IK propia con PyKDL). | Python |
| `staubli_joystick_servo` | Teleoperación cartesiana por velocidad **sin** MoveIt. | Python |
| `staubli_gui` | Interfaz gráfica de usuario. | Python |

`staubli_moveit_config` es, por tanto, un paquete **puramente de configuración**: no compila una
sola línea de código. Es el equivalente en MoveIt de lo que hace `staubli_robot_description` en el
mundo del URDF.

Su única dependencia interna es `staubli_robot_description` (`package.xml:15`): de ahí toma el
URDF, los nombres de los links y el nombre del frame del efector final.

### 1.3 Qué problema resuelve que el URDF no resuelve

Un URDF es puramente mecánico. Describe que existe un link `link_2` unido a `link_1` por `joint_2`.
Pero **no dice**:

- Que los seis joints forman una sola cadena que se planifica en conjunto (el "brazo").
- Que `link_1` y `link_3` nunca pueden chocar aunque el URDF sugiera lo contrario, porque están
  separados por la estructura del robot.
- Qué solver de cinemática inversa usar.
- A qué velocidad máxima se debe planificar, aunque el motor pueda ir más rápido.
- Cómo se traduce una trayectoria articular a una llamada a un controlador concreto.

Todo eso es lo que aporta este paquete. La forma canónica de expresarlo es el **SRDF**.

---

## 2. Estructura de la carpeta

```
staubli_moveit_config/
├── .setup_assistant              Memoria del MoveIt Setup Assistant (qué URDF y qué SRDF usar)
├── CMakeLists.txt                Instalación y registro del test
├── package.xml                   Dependencias
├── config/
│   ├── joint_limits.yaml         Límites de velocidad/aceleración para planificar
│   ├── kinematics.yaml           Solver de cinemática inversa del grupo "arm"
│   ├── moveit_controllers.yaml   Mapeo MoveIt → controladores ros2_control
│   ├── moveit.rviz               Vista de RViz con el panel MotionPlanning
│   ├── pilz_cartesian_limits.yaml Límites cartesianos del planificador Pilz
│   └── staubli_servo.yaml        Parámetros de MoveIt Servo (teleoperación)
├── launch/
│   └── staubli_moveit.launch.py  Arranca move_group, servo (opcional) y RViz
├── srdf/
│   ├── staubli.srdf.xacro        Fichero raíz del SRDF
│   └── staubli_macro.srdf.xacro  Macro que crea el grupo y las autocolisiones
└── test/
    └── test_default_srdf.py      Test registrado en el CMakeLists
```

Ausencias que conviene notar, porque son igual de informativas que las presencias:

- **No hay `config/staubli.urdf.xacro`.** El Setup Assistant suele generar aquí una copia
  "modificada" del URDF (para insertar los `ros2_control`). Este proyecto **no** lo hace: usa
  directamente el URDF de `staubli_robot_description`, que ya trae su propio bloque
  `<ros2_control>` en `r2c/`. La consecuencia práctica es que el URDF que ve MoveIt es
  exactamente el mismo que ve `robot_state_publisher`.
- **No hay ficheros `*_planning.yaml`.** Por eso los planificadores no se configuran aquí (ver
  §3.7 y §4.8).
- **No hay `sensors_3d.yaml`.** Es opcional: sirve para declarar sensores de distancia 3D que
  MoveIt use para actualizar la escena. Aquí no se usa ninguno.
- **No hay `moveit_cpp.yaml`.** Es para la interfaz de C++ de MoveIt, que no se usa aquí.

---

## 3. Explicación de cada archivo

### 3.1 `.setup_assistant` — el puente con el URDF

**Qué es.** Un fichero YAML oculto (empieza por punto) generado por el *MoveIt Setup Assistant*,
la herramienta gráfica que se usa normalmente para crear un paquete como este. Es, literalmente,
la memoria del asistente: qué URDF procesó y dónde está el SRDF que generó.

**Para qué sirve.** No lo lee ningún nodo. Lo lee la clase `MoveItConfigsBuilder` de
`moveit_configs_utils` en su constructor (`moveit_configs_builder.py:164-205`), que es la clase
que usa el launch. Con este fichero, el builder deduce por sí solo:

```yaml
urdf:
  package: staubli_robot_description        # de qué paquete sacar el URDF
  relative_path: urdf/staubli.urdf.xacro    # qué fichero
  xacro_args: robot_model:=tx2_60l          # con qué argumentos generarlo
```

**Qué contiene, y qué hay que saber de cada bloque:**

| Bloque | Contenido | Lectura por el builder |
|---|---|---|
| `urdf` | Paquete, ruta y argumentos del Xacro del robot. | Sí: fija `robot_description`. |
| `srdf` | `relative_path: config/tx2_60l.srdf`. | Se lee, **pero ese fichero no existe**. |
| `package_settings` | Autor y marca temporal de generación. | No se lee. |
| `control_xacro` | Qué interfaces de `ros2_control` insertar. | No se lee (queda el URDF original). |
| `modified_urdf` | Lista de Xacro a insertar. | No se lee. |

**Relación con los demás ficheros.** Es la única pieza que conecta automáticamente este paquete
con `staubli_robot_description`, sin que nadie tenga que escribir la ruta en un launch.

**Observaciones verificadas que conviene tener presentes:**

1. **La ruta del SRDF está obsoleta.** Declara `config/tx2_60l.srdf`, y ese fichero no está en la
   carpeta. No rompe nada porque el launch no deja que el builder use ese valor: llama
   explícitamente a `.robot_description_semantic(Path("srdf") / "staubli.srdf.xacro", ...)`
   (`staubli_moveit.launch.py:109-115`), que pisa el valor del `.setup_assistant`. Es decir, el
   bloque `srdf` de este fichero es un resto de la generación original.
2. **El bloque `control_xacro` también es un resto.** Describe insertar un Xacro llamado
   `control_xacro`, y hoy el paquete `staubli_robot_description` no lo tiene: esa función la
   cumplió la carpeta `r2c/`. Además, la clave `control_xacro` aparece **duplicada** en el
   fichero (líneas 12-17 y 21-25) con el mismo contenido.
3. **El URDF queda fijado a `tx2_60l`.** Como el launch no llama a `.robot_description(...)` con
   sus propios argumentos, el builder usa los `xacro_args` de aquí. Es decir: el `robot_model` que
   se pase al launch afecta al **SRDF**, pero el `robot_description` que se le da a `move_group` y
   a RViz se genera siempre con `tx2_60l`. En este robot concreto el efecto es menor de lo que
   parece (el grupo de planificación y los nombres de links son iguales en los tres modelos, y
   solo cambia la posición del efector final), pero es un comportamiento a tener en cuenta.

### 3.2 `package.xml` — dependencias

**Qué es.** El manifiesto del paquete, formato 3. Declara el nombre, la versión (`0.1.4`), el
mantenedor y las dependencias.

**Para qué sirve.** `rosdep` y el gestor de paquetes de ROS 2 lo usan para saber qué hay que
instalar. Como el paquete no compila nada, **todas** las dependencias son `<exec_depend>`: se
necesitan en tiempo de ejecución, no para construir.

**Qué contiene, agrupado por para qué se usa cada dependencia:**

| Dependencia | Para qué se necesita en este paquete |
|---|---|
| `staubli_robot_description` | El URDF, los nombres de links y el frame del efector. |
| `moveit_configs_utils` | La clase `MoveItConfigsBuilder` que lee `.setup_assistant` y los YAML. |
| `moveit_kinematics` | El plugin de cinemática KDL que se referencia en `kinematics.yaml`. |
| `moveit_planners` | Carga de los planificadores (OMPL, Pilz, CHOMP, STOMP). |
| `moveit_ros_move_group` | El nodo `move_group` que el launch arranca. |
| `moveit_ros_warehouse` | La base de datos donde MoveIt guarda estados y escenas. |
| `warehouse_ros_sqlite` | El backend SQLite concreto que se configura en el launch. |
| `moveit_simple_controller_manager` | El gestor de controladores declarado en `moveit_controllers.yaml`. |
| `moveit_ros_visualization` | El plugin de RViz `MotionPlanning` que se usa en `moveit.rviz`. |
| `rviz2`, `rviz_common`, `rviz_default_plugins` | El visualizador y sus displays estándar. |
| `tf2_ros` | Transformaciones entre frames. |
| `xacro` | Procesar el SRDF, que es un Xacro. |

**Relación con los demás paquetes.** Es la lista de "a quién necesito para que este paquete
funcione", y es la mejor forma de entender de qué piezas se compone el sistema de planning.

**Observación verificada:** el launch puede arrancar `moveit_servo/servo_node`, pero `moveit_servo`
**no está declarado** en este `package.xml`. Es una dependencia que falta.

### 3.3 `CMakeLists.txt` — instalación y test

**Qué es.** Un CMake mínimo, de 19 líneas, con `build_type: ament_cmake`.

**Qué hace.** Solo dos cosas:

1. Instalar las cuatro carpetas al `share` del paquete:
   `install(DIRECTORY config launch srdf test DESTINATION share/${PROJECT_NAME})`.
   No hay ninguna regla de compilación: ni `add_library`, ni `add_executable`. Es el equivalente
   exacto de lo que hace `staubli_robot_description`.
2. Registrar un único test cuando `BUILD_TESTING` está activo:
   `ament_add_pytest_test(test_default_srdf_xacro_parsing test/test_default_srdf.py)`.

**Por qué importa.** El `share` instalado es lo que el launch encuentra con
`FindPackageShare("staubli_moveit_config")` y lo que el `MoveItConfigsBuilder` usa como raíz para
encontrar `config/` y `srdf/`. Si algo no está en esas cuatro carpetas, MoveIt no lo ve.

### 3.4 `srdf/staubli.srdf.xacro` — el fichero raíz del SRDF

**Qué es.** La entrada al SRDF. Igual que el URDF es un Xacro, el SRDF también lo es: se procesa
con `xacro` y el resultado es un SRDF (XML) que se entrega a MoveIt como parámetro.

**Para qué sirve.** El SRDF es la descripción **semántica** del robot. Se explica en detalle en
§4.1, pero la idea es: el URDF dice "existe `joint_1`", y el SRDF dice "estas seis articulaciones
juntas forman el grupo `arm`".

**Qué contiene (28 líneas).** Es un fichero muy corto que se limita a preparatory el contexto y a
delegar:

1. Declara tres argumentos: `robot_model` (sin valor por defecto, hay que pasarlo),
   `name` (por defecto `staubli`) y `robot_prefix` (vacío).
2. Fija el nombre del robot con `<xacro:attribute name="name" value="$(arg name)"/>`.
3. Construye la ruta a la configuración del modelo en el **otro** paquete:
   `$(find staubli_robot_description)/config/$(arg robot_model)`.
4. Incluye la macro de este paquete, `staubli_macro.srdf.xacro`.
5. **Vuelve a incluir `staubli_helpers.xacro` del paquete de descripción y a cargar el YAML del
   modelo**, con `xacro:load_robot_model_config`. Ese loader es el que exporta las propiedades
   `link_names` y `end_effector_frame_name`.
6. Llama a la macro `staubli_srdf` pasándole, entre otros, los dos extremos del brazo:
   ```xml
   arm_base_link="$(arg robot_prefix)${link_names[0]}"          → base_link
   arm_tip_link="$(arg robot_prefix)${end_effector_frame_name}" → tool0
   ```

**El truco de la línea 25.** `end_effector_frame_name` no está escrito a mano en el SRDF: viene de
`urdf/include/default_names.yaml` del paquete de descripción, que declara `ee_frame: tool0`. Y
`${link_names[0]}` es `base_link`. Es decir, **el grupo de planificación se define a partir de los
mismos datos que el URDF**, lo que garantiza que no se puedan desincronizar los nombres: si
alguien renombra `link_1` en la descripción, el SRDF se actualiza solo.

**Resultado verificado.** Procesando el fichero con `robot_model:=tx2_60l` se obtiene exactamente:

```xml
<robot name="staubli">
  <group name="arm">
    <chain base_link="base_link" tip_link="tool0"/>
  </group>
  <disable_collisions link1="base_link" link2="link_1" reason="Adjacent"/>
  ... (11 líneas en total)
</robot>
```

Con `robot_prefix:=staubli_` el resultado pasa a ser `<group name="staubli_arm">` con
`base_link="staubli_base_link" tip_link="staubli_tool0"`, que es **exactamente** lo que produce el
URDF con el mismo prefijo (verificado: el URDF genera `staubli_base_link`, `staubli_link_1` ...
`staubli_tool0`). El mecanismo de prefijos es coherente entre los dos paquetes.

**Lo que este SRDF NO contiene**, y que sí existe en otros robots: no hay *virtual joint* (joint
flotante que une el robot al mundo), ni grupo de efector final (`<end_effectors>`), ni joints
pasivos, ni estados nombrados, ni poses nombradas. Para un brazo de 6 grados de libertad sujeto a
una base, esa es una configuración mínima razonable: el grupo es simplemente toda la cadena.

### 3.5 `srdf/staubli_macro.srdf.xacro` — grupo de planificación y autocolisiones

**Qué es.** La macro `staubli_srdf`, que es donde está el contenido semántico real del SRDF.

**Qué contiene.** Tres cosas.

**a) El grupo de planificación** (`:21-23`):

```xml
<group name="${robot_prefix}arm">
    <chain base_link="${arm_base_link}" tip_link="${arm_tip_link}" />
</group>
```

La etiqueta `<chain>` significa: "toma todos los links y todos los joints que hay en el camino
que va de `base_link` a `tool0`". Es la forma más natural de definir un brazo: no hace falta
enumerar los seis joints a mano. El nombre del grupo (`arm`) es el que después se busca en
`kinematics.yaml` (ver §3.6).

**b) Las autocolisiones deshabilitadas** (`:26-36`), once pares:

```xml
<disable_collisions link1="${link_names[1]}" link2="${link_names[3]}" reason="Never"/>
```

**Qué significa exactamente.** Por defecto, MoveIt asume que **cualquier** link del robot puede
chocar con **cualquier** otro. Esto es conservador y correcto, pero tiene un coste enorme: la
mayoría de esos pares no pueden chocarse nunca, así que el detector descarta sistemáticamente
millones de muestras-space de configuración que son válidas. El resultado es que la planificación
se ralentiza muchísimo o directamente no encuentra solución. La etiqueta `<disable_collisions>`
declara: "no compruebes colisiones entre estos dos links".

El atributo `reason` es solo documentación para quien lea el SRDF:

- `Adjacent`: los dos links están unidos por un joint. Es físicamente imposible que choquen
  porque el robot es rígido. Hay 5 de estos pares.
- `Never`: no son adyacentes, pero la cinemática del robot hace imposible que se toquen.

**Relación con el URDF.** Los links se nombran con `${link_names[n]}`, es decir, se toman de
`default_names.yaml`. La numeración es: 0 = `base_link`, 1..6 = `link_1`..`link_6`.

**c) Detalle interesante: la matriz de colisiones está incompleta a propósito.** Hay 7 links, es
decir, 21 pares posibles, y solo se deshabilitan 11. Los 10 pares que **sí** se comprueban incluyen
todos los de `base_link` contra `link_2`..`link_6`, más `link_1`-`link_4`, `link_1`-`link_5`,
`link_1`-`link_6`, `link_2`-`link_6` y `link_4`-`link_6`. Tiene sentido: el brazo plegándose
puede golpear su propia base, y los tramos largos del TX2-60L pueden acercarse entre sí. Lo que
significa exactamente cada uno de esos diez pares a partir de los ficheros es **DESCONOCIDO**:
hacerlo bien requiere medir con las mallas de colisión o usar la herramienta
*Self-Collision Report* del Setup Assistant, y no hay aquí ningún registro de ese análisis.

**Relación con los demás ficheros.** Este fichero vuelve a incluir `staubli_helpers.xacro` y a
cargar el YAML del modelo por su cuenta (`:13-14`), aunque quien lo llama ya lo había hecho. Es
redundante pero inofensivo: en Xacro, cargar dos veces el mismo YAML simply reasigna las mismas
propiedades.

### 3.6 `config/kinematics.yaml` — cómo se resuelve la cinemática inversa

**Qué es.** Cuatro líneas que asocian un solver al grupo `arm`.

```yaml
arm:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.0050000000000000001
  kinematics_solver_timeout: 0.5
```

**Para qué sirve.** Cuando MoveIt recibe un objetivo en coordenadas cartesianas ("quiero `tool0`
en x=0.3, y=0.1, z=0.5"), necesita saber **qué valores de las seis articulaciones** producen esa
pose. Eso es la cinemática inversa (IK). Cada robot resuelve la IK de forma distinta, así que
MoveIt carga un *plugin* que lo haga por él.

**Qué significa cada parámetro:**

| Parámetro | Significado |
|---|---|
| `arm` | Debe coincidir **exactamente** con el nombre del `<group>` del SRDF. Es la clave de unión entre los dos ficheros. |
| `kinematics_solver` | Qué plugin resuelve la IK. `KDLKinematicsPlugin` usa la librería KDL y hace una aproximación numérica por Newton-Raphson. |
| `kinematics_solver_search_resolution` | Resolución de la búsqueda numérica, en radianes. Valores más pequeños = más precisión y más CPU. |
| `kinematics_solver_timeout` | Tiempo máximo que se le concede al solver, en segundos. |

**Relación con los demás ficheros.** La cadena es: el SRDF crea el grupo `arm` → este YAML le
asigna un solver → el solver necesita el URDF para conocer la geometría. Los tres hacen falta.

**Verificado:** el nombre `arm` coincide entre el `<group name="arm">` del SRDF generado y la
clave de este YAML. Si no coincidieran, MoveIt no podría planificar.

### 3.7 `config/joint_limits.yaml` — con qué ritmo se planifica

**Qué es.** Cuarenta líneas que **modifican** los límites articulares del URDF para la planificación.
El propio fichero lo dice en la primera línea: "allows the dynamics properties specified in the
URDF to be overwritten or augmented".

**Para qué sirve.** Hay que distinguir dos juegos de límites, y confundirlos es el error más común
al empezar con MoveIt:

- **Límites del URDF** (en `staubli_robot_description/config/tx2_60l/joint_limits.yaml`): lo que
  el robot **puede** hacer. Para el TX2-60L son 300 °/s y 1000 °/s² en `joint_1`..`joint_5`, y
  500 °/s y 3000 °/s² en `joint_6`. Son propiedades físicas del robot.
- **Límites de MoveIt** (este fichero): con qué velocidad y aceleración se **permite que el
  planificador mueva** al robot. Casi siempre son más restrictivos que los del URDF, por
  seguridad.

**Comparación concreta, calculada a partir de los dos ficheros:**

| | URDF (descripción) | MoveIt (este fichero) | Proporción |
|---|---|---|---|
| `joint_1`..`joint_5` velocidad | 5.24 rad/s (300 °/s) | 1.5 rad/s | ~29 % |
| `joint_6` velocidad | 8.73 rad/s (500 °/s) | 2.0 rad/s | ~23 % |
| `joint_1`..`joint_5` aceleración | 17.45 rad/s² | 5.0 rad/s² | ~29 % |
| `joint_6` aceleración | 52.36 rad/s² | 10.0 rad/s² | ~19 % |

Es decir, el planificador está configurado para moverse a **algo menos de un tercio** de la
velocidad máxima teórica. Es una decisión de seguridad deliberada, y el comentario del fichero
lo dice: "For beginners, we downscale velocity and acceleration limits".

**Los dos factores de escalado globales** (`:5-6`):

```yaml
default_velocity_scaling_factor: 0.1
default_acceleration_scaling_factor: 0.1
```

Son multiplicadores que se aplican **por defecto a cada petición de movimiento**: si pides un
movimiento sin especificar nada, MoveIt usará el 10 % de la velocidad y el 10 % de la aceleración
máximas. En RViz puedes subirlos con los sliders de la pestaña *Planning Request*; los valores
pueden ir hasta 1.0. Esto es lo que hace que las demostraciones en RViz parezcan lentas, y es
intencional: MoveIt prioriza movimientos seguros y visibles sobre movimientos rápidos.

**Restricciones por joint.** `has_velocity_limits: true` y `has_acceleration_limits: true` le
dicen a MoveIt que esos límites existen y son válidos. Si se pusieran a `false`, MoveIt ignoraría
el límite correspondiente.

**Relación con los demás ficheros.** Este YAML **no** sustituye al del URDF: lo *augmenta*. El
URDF aporta los límites de posición (`min_position`/`max_position`, que MoveIt usa para no
planificar fuera del rango físico de la articulación) y los limites de velocidad que aquí se
redefinen. Ambos se combinan.

**Detalle técnico:** `MoveItConfigsBuilder.joint_limits()` carga este fichero bajo el nombre de
parámetro **`robot_description_planning`**, no bajo `joint_limits`. Es un nombre heredado que
resulta confuso, pero conviene saberlo si alguna vez se depura mirando los parámetros del nodo
`move_group` con `ros2 param list`.

### 3.8 `config/moveit_controllers.yaml` — el puente hacia `ros2_control`

**Qué es.** La configuración que le dice a MoveIt **a quién entrega las trayectorias que calcula**.

```yaml
moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager

moveit_simple_controller_manager:
  controller_names:
    - joint_trajectory_controller
  joint_trajectory_controller:
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: false
    joints: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]
```

**Para qué sirve.** Cuando `move_group` termina de calcular una trayectoria, tiene que enviarla a
alguien que la ejecute sobre el robot. Para saber a quién, necesita saber:

1. **Qué gestor de controladores usar**: `MoveItSimpleControllerManager` (la versión
   simplificada que solo sabe activar y desactivar controladores).
2. **Qué controladores existen**: aquí, solo `joint_trajectory_controller`.
3. **Dónde está su action server**: `action_ns: follow_joint_trajectory` más el nombre del
   controlador da el topic de la action
   `/joint_trajectory_controller/follow_joint_trajectory`.
4. **De qué tipo es la action**: `FollowJointTrajectory`, que es el tipo de mensaje estándar de
   `control_msgs` con puntos, velocidades y aceleraciones.
5. **Qué articulaciones gestiona**: las seis. Esto es importante, porque MoveIt comprueba que el
   controlador cubra **todas** las articulaciones del grupo que quiere mover. Si el grupo `arm`
   necesita 6 joints y el controlador solo tiene 5, la ejecución falla.

**Qué significa `default: false`.** Indica que MoveIt **no** va a activar ni desactivar este
controlador por su cuenta; asumirá que alguien lo ha puesto en `active` por otra vía. Y eso es
justo lo que hace `staubli_bringup/launch/launch_robot_control.launch.py`: carga el
`joint_trajectory_controller` con `--inactive` y luego lo activa con
`ros2 control set_controller_state joint_trajectory_controller active` si el argumento
`start_controller` lo pide.

**Relación con los demás paquetes.** El otro extremo de este mapa está en
`staubli_bringup/config/controllers.yaml`, que declara el mismo controlador con los mismos seis
joints. Verificado: los nombres y la lista de joints **coinciden exactamente**. Ese fichero
además declara `joint_state_broadcaster` (de donde sale `/joint_states`) y
`forward_velocity_controller` (que usa la teleoperación por joystick).

**Consecuencia práctica:** este paquete **no** crea ningún controlador. Solo dice cómo llamarlos.
Quien los crea es `staubli_bringup`. Y si ejecutas solo `staubli_moveit.launch.py` sin el launch de
control, MoveIt podrá planificar pero **no podrá ejecutar**: dará un error del tipo "unable to
identify any set of controllers that can actuate the requested joints".

### 3.9 `config/pilz_cartesian_limits.yaml` — límites del planificador Pilz

```yaml
cartesian_limits:
  max_trans_vel: 1.0      # m/s
  max_trans_acc: 2.25     # m/s²
  max_trans_dec: -5.0     # m/s² (negativo: es deceleración)
  max_rot_vel: 1.57       # rad/s
```

**Qué es.** Un planificador **secuencial y determinista**, a diferencia de OMPL (que es de
muestreo). Con Pilz se piden trayectorias de tipo PTP (punto a punto), LIN (línea recta) o CIRC
(arco), y el planificador las ejecuta como un perfil de velocidad trapezoidal o sinusoidal. No
busca "el camino más rápido", construye el camino que le has pedido. Por eso necesita conocer
cuál es la velocidad máxima *en el espacio cartesiano*, y no solo en las articulaciones.

**Qué significa cada límite:** `max_trans_vel` es la velocidad lineal máxima en m/s,
`max_trans_acc` la aceleración, `max_trans_dec` la deceleración (negativa por convención) y
`max_rot_vel` la velocidad angular máxima en rad/s. 1.57 rad/s es aproximadamente π/2, es decir
un cuarto de vuelta por segundo: un valor deliberadamente conservador.

**Relación con los demás ficheros, y un detalle no evidente.** Este fichero solo se carga si el
*pipeline* `pilz_industrial_motion_planner` está declarado. Como el paquete **no** tiene ningún
`*_planning.yaml`, el `MoveItConfigsBuilder` recurre a los ficheros por defecto de
`moveit_configs_utils`, que incluyen `pilz_industrial_motion_planner_planning.yaml`. Por eso el
nombre del fichero (`pilz_cartesian_limits.yaml`) coincide con lo que el builder busca, y por eso
**sí se carga** en la práctica. El contenido final se fusiona dentro de `robot_description_planning`,
junto a los límites articulares de §3.7.

**Sobre si está disponible:** el plugin `pilz_industrial_motion_planner` está instalado en el entorno
analizado, así que el pipeline es utilizable aunque no haya ninguna configuración propia de
planificación en el repositorio.

### 3.10 `config/staubli_servo.yaml` — teleoperación con MoveIt Servo

**Qué es.** Un fichero de 85 líneas, commented en inglés, con la configuración de **MoveIt Servo**.

**Qué es MoveIt Servo, y en qué se diferencia de MoveIt.** Es un concepto que confunde mucho.
Son dos cosas distintas del mismo framework:

- **MoveIt (`move_group`)** **planifica**: piensa la trayectoria completa de antemano, la verifica
  contra colisiones y límites, y la ejecuta de principio a fin. Es robusto, pero tiene latencia.
- **MoveIt Servo** **no planifica**: es un bucle de control de muy alta frecuencia que traduce
  comandos de velocidad en pequeñas correcciones articulares. Es la alternativa a MoveIt para
  teleoperación con joystick o para control cartesianos continuos. En la práctica hace IK en
  bucle cerrado y aplica los comandos articulación a articulación, sin construir trayectoria.

Por eso los dos se lanzan por separado y el servo está **desactivado por defecto**
(`launch_servo`, por defecto `false`).

**Qué contienen los bloques, agrupados por función:**

*Entrada de comandos:*
- `command_in_type: "speed_units"`: los comandos son velocidades reales (m/s y rad/s), no valores
  en el rango `[-1, 1]` como los de un joystick. Con `"unitless"` se usarían los factores de
  `scale` (0.6 m/s y 0.3 rad/s).
- `scale`: velocidades máximas para comandos cartesianos y articulares. Con `speed_units` no se
  usan, pero están ahí por si se cambia el tipo.
- `system_latency_compensation: 0.04`: corrige el retardo de red. El servo calcula "dónde quiero
  estar" sumando un pequeño margen hacia delante para compensar el retraso de comunicación.

*Salida de comandos:*
- `publish_period: 0.004`: publica a 250 Hz (1/0.004). Coincide exactamente con el
  `update_rate: 250` de `controller_manager` en `staubli_bringup/config/controllers.yaml`.
- `low_latency_mode: false`: publica cada `publish_period` en vez de publicar en cuanto llega un
  comando.
- `command_out_type: std_msgs/Float64MultiArray` y `publish_joint_positions: true`: publica solo
  posiciones articulares, en un array plano. Es lo que espera un `forward_command_controller`.
- `smoothing_filter_plugin_name: "online_signal_smoothing::ButterworthFilterPlugin"`: un filtro de
  Butterworth que suaviza el comando antes de mandarlo, para que el robot no dé tirones.

*Frames, y aquí está lo importante:*
```yaml
move_group_name: staubli_arm
planning_frame: base_link
ee_frame: tool0
robot_link_command_frame: tool0
```
`planning_frame` es el frame en el que se expresan los objetivos; aquí la base del robot, que es lo
natural para un brazo fijo. `ee_frame` es el link del efector final (coincide con el `tool0` del
SRDF). Y `move_group_name` le dice al servo **cómo se llama el nodo `move_group`** en el grafo de
ROS, para poder subscriptarse a su escena de planificación y hacer comprobaciones de colisión.

**Verificación importante:** en `staubli_moveit.launch.py:130-141`, el nodo `move_group` se declara
**sin argumento `name`**, así que su nombre real es `move_group`, no `staubli_arm`. Y
`command_out_topic: /forward_position_controller/commands` apunta a un
`forward_position_controller` que **no aparece en ningún sitio** del repositorio (lo que sí existe
es un `forward_velocity_controller`). Los dos valores parecen desalineados con el resto del
proyecto. Si la intención del autor era otra, no se puede determinar a partir de los ficheros.

*Seguridad, y esto es lo más importante del fichero:*
- `incoming_command_timeout: 0.1`: si pasan 0.1 s sin recibir comando, para el servo.
- `num_outgoing_halt_msgs_to_publish: 4`: publica 4 mensajes de parada. El comentario explica por
  qué: "ROS may drop some messages and we need the robot to halt reliably". Los mensajes se pueden
  perder, así que se repiten.
- `is_primary_planning_scene_monitor: false`: correcto, porque en este sistema el nodo `move_group`
  es el que tiene la escena de planificación principal; el servo debe ser un cliente.
- `lower_singularity_threshold: 100.0` y `hard_stop_singularity_threshold: 200.0`: el servo mide
  el **número de condición** de la matriz Jacobiana. Cuando el robot está cerca de una
  singularidad (una configuración en la que un movimiento no se puede lograr en todas las
  direcciones), el Jacobiano se hace casi singular y la IK produce valores enormes. Estos umbrales
  hacen que el servo **desacelere** y luego **pare** antes de que eso ocurra.
- `joint_limit_margins: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]`: un margen de 0.1 radianes (~5.7°) que se
  añade a los límites articulares del URDF, para no acercarse nunca del todo al tope.
- `check_collisions: true` con `collision_check_type: threshold_distance`: cada
  `collision_check_rate` (5 Hz) comprueba la distancia de los links y empieza a frenar al estar a
  menos de 1 cm de una autocolisión o 2 cm de una colisión con la escena. Es una comprobación
  *reactiva*, a diferencia de la *predictiva* que hace el planificador.

**Relación con el resto del proyecto.** Existe otro camino de teleoperación en el repositorio,
`staubli_joystick_servo`, que es un nodo propio (no MoveIt) que resuelve la cinemática con la
librería KDL y usa damped least squares. Los dos ficheros de configuración son independientes: el
de este paquete es
para el servo oficial de MoveIt, el otro es una reimplementación sin MoveIt.

### 3.11 `config/moveit.rviz` — la vista de planificación

**Qué es.** La configuración guardada de RViz, con 51 líneas, en formato YAML.

**Para qué sirve.** RViz por sí solo puede mostrar un URDF. Para **planificar** hace falta el panel
`MotionPlanning`, que es un plugin (`moveit_rviz_plugin`) que viene en el paquete
`moveit_ros_visualization`. Este fichero configura ese panel.

**Qué contiene, y por qué cada línea importa:**

| Clave | Significado |
|---|---|
| `Class: moveit_rviz_plugin/MotionPlanning` | El display que hace posible planificar desde la interfaz. |
| `Robot Description: robot_description` | El URDF, leído como **parámetro**, no como topic. |
| `Planning Scene Topic: monitored_planning_scene` | Topic donde `move_group` publica la escena (objetos, robots, colisiones). |
| `Trajectory Topic: display_planned_path` | Topic donde aparece la trayectoria planificada. |
| `Loop Animation: true`, `State Display Time: 0.05 s` | Reproduce la trayectoria en bucle, pasando 50 ms por punto, para verla "en movimiento". |
| `Robot Alpha: 0.5` | El robot se dibuja al 50 % de opacidad, para distinguirlo de los obstáculos de la escena. |
| `Fixed Frame: world` | Coincide con el link raíz del URDF. |
| `Focal Point`, `Distance`, `Pitch`, `Yaw` | La cámara guardada, ya encuadrada sobre el robot. |

**Relación con el launch.** El launch no pasa `-d config/moveit.rviz` a un `rviz2` genérico como
en el paquete de descripción: le pasa **solo** los parámetros que el panel necesita
(`robot_description`, `robot_description_semantic`, `robot_description_kinematics`,
`planning_pipelines`, `joint_limits` y la config del warehouse, en `staubli_moveit.launch.py:166-176`).
El display se connects así a la misma configuración que usa `move_group`, y por eso el panel
muestra exactamente el mismo robot y los mismos grupos que el nodo de planificación.

**Detalle importante:** el display **necesita** el parámetro `robot_description_semantic`, y sin él
el panel no sabe qué grupos de planificación existen. Por eso el launch se lo pasa
explícitamente.

### 3.12 `launch/staubli_moveit.launch.py` — el arranque

Es el fichero que ata todo lo anterior. Un launch file de ROS 2 es simplemente un programa de
Python que devuelve un `LaunchDescription`, una lista de acciones. Se ejecuta cuando se lanza.

**Los argumentos que expone** (`:53-98`):

| Argumento | Por defecto | Para qué |
|---|---|---|
| `robot_model` | (obligatorio) | Qué modelo pasar a los Xacro. |
| `launch_rviz` | `true` | Si abrir la interfaz gráfica. |
| `warehouse_sqlite_path` | `~/.ros/warehouse_ros.sqlite` | Dónde guardar la base de datos de MoveIt. |
| `launch_servo` | `false` | Si arrancar MoveIt Servo. |
| `use_sim_time` | `false` | Si usar el reloj de la simulación en vez del reloj de ROS. |
| `publish_robot_description_semantic` | `true` | Si `move_group` publica el SRDF. |

**El bloque central: la construcción de la configuración** (`:107-120`):

```python
moveit_config = (
    MoveItConfigsBuilder(robot_name="staubli", package_name="staubli_moveit_config")
    .robot_description_semantic(
        Path("srdf") / "staubli.srdf.xacro",
        {"prefix": "", "robot_model": robot_model},
    )
    .planning_scene_monitor(
        publish_robot_description_semantic=publish_robot_description_semantic
    )
    .to_moveit_configs()
)
```

Qué pasa aquí, paso a paso:

1. `MoveItConfigsBuilder(...)` lee `.setup_assistant` y se queda con la ruta del URDF y los
   `xacro_args` (`robot_model:=tx2_60l`).
2. `.robot_description_semantic(...)` procesa el SRDF y lo deja en el parámetro
   `robot_description_semantic`. Los `mappings` son los argumentos que se le pasan a `xacro`. Como
   uno de ellos es una `LaunchConfiguration` y no una cadena literal, el builder **no** procesa
   el Xacro en ese momento, sino que instala una *substitution* que lo procesará en el momento de
   lanzar, cuando el valor del argumento ya sea conocido. Es un detalle de diseño importante: los
   Xacro que dependen de argumentos de launch se resuelven **en tiempo de launch**, no antes.
3. `.planning_scene_monitor(...)` genera un pequeño diccionario de parámetros que le dice a
   `move_group` quétopics debe publicar.
4. `.to_moveit_configs()` completa todo lo que falta. Y aquí es donde conviene saber qué se carga
   solo, porque es lo que hace que `config/` "funcione solo":

| Fichero | ¿Se carga? | Cómo |
|---|---|---|
| `srdf/staubli.srdf.xacro` | Sí | Explícitamente, en el paso 2. |
| URDF de `staubli_robot_description` | Sí | Automático, desde `.setup_assistant`. |
| `config/kinematics.yaml` | Sí | Automático, nombre fijo. |
| `config/joint_limits.yaml` | Sí | Automático, nombre fijo, como `robot_description_planning`. |
| `config/moveit_controllers.yaml` | Sí | Automático, buscando cualquier `*_controllers.yaml`. |
| `config/pilz_cartesian_limits.yaml` | Sí | Automático, porque el pipeline de Pilz viene por defecto. |
| Pipelines de planificación | Sí | Automático, desde los ficheros por defecto de `moveit_configs_utils` (OMPL, Pilz, CHOMP, STOMP). |
| `config/sensors_3d.yaml` | No existe | Se salta sin error si no está. |
| `config/staubli_servo.yaml` | Manual | Lo carga el propio launch con `load_yaml` (`:143`). |
| `config/moveit.rviz` | Manual | Lo pasa como argumento `-d` a RViz (`:156-158`). |

**Los tres nodos que arranca:**

1. **`move_group`** (`:130-141`), siempre. Es el nodo principal. Recibe `moveit_config.to_dict()`
   (todos los parámetros de una vez), la configuración del warehouse y `use_sim_time`.
2. **`servo_node`** (`:145-154`), solo si `launch_servo:=true`. Nodo `moveit_servo`, con
   `servo_params = {"moveit_servo": servo_yaml}`: el `{"moveit_servo": ...}` es el **namespace**
   bajo el que el nodo espera sus parámetros, por eso el YAML no necesita esa clave raíz.
3. **`rviz2`** (`:159-177`), por defecto, con nombre `rviz2_moveit` y `-d config/moveit.rviz`.

**Qué NO hace este launch**, y es su limitación más importante:

- **No arranca `robot_state_publisher`.** Hay un comentario explícito en `:179-180`: "Make sure
  that robot state_publisher starts before this launch file. See bringup package launch file". Sin
  `robot_state_publisher` no existe el topic `/robot_description` ni el árbol de TF, y MoveIt no
  tiene con qué trabajar.
- **No arranca `controller_manager` ni el `joint_trajectory_controller`.** Solo los nombra.
- **No arranca Gazebo ni el driver.**

Por eso, **en la práctica, casi nadie lanza este fichero directamente**: se lanza
`staubli_bringup/launch/launch_moveit.launch.py`, que lo incluye con
`IncludeLaunchDescription`. Ese launch, además, lanza antes un script
(`staubli_bringup/scripts/wait_for_robot_ready.py`) que se queda esperando a recibir un mensaje en
`/robot_description` **y** otro en `/joint_states`, y solo cuando ambos han llegado lanza MoveIt.
Ese pequeño script resuelve un problema real muy común: si `move_group` arranca antes de que
exista la descripción del robot, se queda esperando indefinidamente y hay que reiniciarlo.

### 3.13 `test/test_default_srdf.py` — el test registrado

**Qué es.** Un test de pytest de 72 líneas, registrado en el `CMakeLists.txt` con el nombre
`test_default_srdf_xacro_parsing`.

**Qué hace, en realidad.** Y aquí hay que ser cuidadoso, porque el nombre del test miente:

1. Localiza `urdf/staubli.urdf.xacro` **del paquete `staubli_robot_description`** (`:33-35`).
   No toca ningún fichero de este paquete.
2. Ejecuta `xacro` con `robot_model:=tx2_60l` y `prefix:=<valor>`.
3. Ejecuta `check_urdf` sobre el URDF resultante y comprueba que termina con código 0.
4. Parametriza con dos prefijos (`""` y `"staubli_"`), lo que da **2 casos** (solo un modelo).

**Por qué el nombre engaña.** El test se llama `test_default_srdf` y el CMake lo registra como
`test_default_srdf_xacro_parsing`, pero **no genera ni valida ningún SRDF**. Se verificó que
`check_urdf` no puede validar un SRDF: al pasarle el SRDF de este paquete falla con
`Error: No link elements found in urdf file`, porque un SRDF a propósito no contiene links. Es
lógico: `check_urdf` valida un URDF. Para validar un SRDF haría falta otra herramienta.

**Segunda observación verificada:** el argumento que se pasa es `prefix:=`, pero el Xacro de la
descripción declara el argumento como `robot_prefix`. Se comprobó que `xacro` acepta el argumento
sobrante **sin error** (código de salida 0) pero **no hace nada**: el URDF generado sigue
teniendo `base_link`, `link_1`, etc., sin prefijo. Es decir, la mitad de la parametrización del
test no prueba nada. El SRDF de este paquete, en cambio, sí gestiona correctamente `robot_prefix`.

**Relación con los demás ficheros.** Cubre la integración con `staubli_robot_description` (que el
URDF es válido para el modelo indicado), pero deja sin cobertura el SRDF, el launch y los YAML.

---

## 4. Conceptos de MoveIt 2 que aparecen en estos archivos

### 4.1 URDF frente a SRDF

Es la distinción más importante de todo el paquete, así que conviene explicarla con precisión:

| | URDF | SRDF |
|---|---|---|
| Significado | **U**nified **R**obot **D**escription **F**ormat | **S**emantic **R**obot **D**escription **F**ormat |
| Responde a | ¿De qué piezas consta el robot y cómo se mueven? | ¿Qué significa cada parte y qué puede hacer el robot? |
| Contiene | Links, joints, geometría, masas, inercias, límites, mallas, tags de Gazebo | Grupos de planificación, enlaces del grupo, colisiones deshabilitadas, joints virtuales, estados nombrados, efector final |
| Lo genera | `staubli_robot_description` | `staubli_moveit_config/srdf/` |
| Lo consume | `robot_state_publisher`, RViz, Gazebo, `ros2_control` | Solo MoveIt |
| Formato | XML con `<robot>` | XML con `<robot>` (¡el mismo nombre de raíz!) |
| Validación | `check_urdf` | Otra herramienta |

Dato práctico: el URDF **no puede** expresar "estas seis articulaciones forman un brazo". El SRDF
tampoco puede expresar la geometría. Son documentos **complementarios**, y MoveIt necesita los
dos. El URDF le dice *cómo es* el robot; el SRDF le dice *con qué chunks* se planifica.

### 4.2 El Move Group

**`move_group` es el nodo que ejecuta `MoveItConfigsBuilder` carga los parámetros y hace todo el
trabajo de planificación.** En concreto:

- Carga el URDF y el SRDF y construye en memoria el `RobotModel`.
- Mantiene la **escena de planificación**: los objetos del mundo, los robots y sus colisiones. La
  publica en `monitored_planning_scene` para que otros nodos puedan verla.
- Escucha peticiones de planificación (topics de `moveit_msgs` y actions) y devuelve trayectorias.
- Gestiona el estado de los controladores a través del `moveit_controller_manager` que se le
  indica en `moveit_controllers.yaml`.
- Atiende servicios y acciones estándar, como `GetMotionPlan`, `GetCartesianPath` y
  `GetStateValidity`.

En este repositorio, el nombre del nodo es `move_group` (por defecto, porque el launch no le pone
`name`), y el SRDF **no** le pone un nombre propio.

### 4.3 Grupos de planificación

Un **grupo de planificación** es un subconjunto de links y articulaciones que MoveIt trata como
una unidad: un brazo, una mano, un torso, o incluso "todo el robot". Los grupos son la unidad
básica de trabajo de MoveIt, y por eso existen dos ficheros que deben hablar el mismo idioma:

- El **SRDF** define el grupo: qué contiene (`<chain>`) y con qué nombre.
- El **kinematics.yaml** le asigna el solver de cinemática, usando ese nombre como clave.

Aquí solo hay un grupo, `arm`, definido como la cadena de `base_link` a `tool0`.

Cuando declaras un grupo con `<chain>`, MoveIt añade automáticamente todos los links y joints del
camino, incluido el joint de la base. Por eso el grupo `arm` contiene los seis joints revolutos.

### 4.4 Cinemática inversa y el solver

**IK** es el problema: "dada esta pose del efector final, ¿qué valores de articulaciones la
alcanzan?". **FK** es el inverso: "dados estos valores de articulaciones, ¿dónde está el
efector?". MoveIt necesita las dos:

- **FK** para dibujar el robot, detectar colisiones y verificar el estado inicial.
- **IK** para convertir un objetivo en comando de articulaciones.

La IK de un brazo de 6 grados de libertad **no tiene solución única**: la misma pose del efector
final se puede alcanzar de varias maneras (codo arriba o codo abajo). El solver de KDL elige una.
De ahí los dos parámetros de `kinematics.yaml`: la resolución (con qué precisión se busca) y el
tiempo máximo (cuánto se insiste antes de rendirse).

### 4.5 Límites articulares: los dos juegos

Ya se ha explicado en §3.7, pero se repite aquí porque es el concepto que más se confunde:

- **Límites del URDF**: lo que el robot **puede** hacer. Properties físicas. Vienen de
  `staubli_robot_description/config/<modelo>/joint_limits.yaml`.
- **Límites de planificación de MoveIt**: con qué ritmo se **permite** moverse. Vienen de
  `config/joint_limits.yaml` de este paquete y se aplican **por encima** de los del URDF.

Y un tercero, relacionado con los anteriores: los **factores de escalado**
(`default_velocity_scaling_factor`), que
modulan los límites de planificación para cada petición concreta.

### 4.6 Joints virtuales y joints pasivos

Aunque **no aparecen en este SRDF**, es útil saber qué son, porque son preguntas naturales al
estudiar un SRDF:

- **Joint virtual** (`<virtual_joint>`): un joint *artificial* de tipo `fixed`, `floating` o
  `planar` que une el robot al mundo, o que une un submóvil a una base. No existe en el URDF y no
  se mueve por sí mismo; sirve para que MoveIt sepa **en qué frame** se expresan los objetivos.
  En un robot anclado al suelo como este brazo, no hace falta: el `world` del URDF ya hace ese
  papel, y el launch usa `base_link` como `planning_frame` (en el servo).
- **Joint pasivo** (`<passive_joint>`): una articulación **real** que el robot tiene pero que no
  debe moverse por sí sola, porque su posición la impone otro mecanismo (un tirante, un muelle, un
  freno). MoveIt la incluye en el modelo para colisiones pero la excluye de la planificación. En
  este robot, los seis joints son activos, así que no hay ninguno.

### 4.7 El `MoveItSimpleControllerManager`

Es el componente que traduce "quiero mover estos joints" en "llamo a este action server". Se
configura en `config/moveit_controllers.yaml` y se implementa en el paquete
`moveit_simple_controller_manager`.

Alternativas: el `ros_control_ros2_controllers/Ros2ControlManager` (más nuevo, y el habitual
hoy) o `moveit_controller_manager/MoveitControllerManager` (el clásico). El proyecto usa el
simple, que es suficiente cuando solo hay un controlador y no hace falta cambiar de
controladores según la tarea.

### 4.8 Pipelines de planificación

MoveIt 2 usa el concepto de *pipeline* para abstraer el algoritmo de planificación. Un pipeline
combina:

- **Planning plugins**: el algoritmo (OMPL, Pilz, CHOMP, STOMP).
- **Request adapters**: pasos que preparan la petición (resolver frames, comprobar que el estado
  inicial no está en colisión, validar los límites del workspace).
- **Response adapters**: pasos que post-procesan la solución (añadir la parametrización temporal,
  validar, y mostrarla en RViz).

Aquí **no hay configuración propia de pipeline**: se usan los ficheros por defecto de
`moveit_configs_utils`, que traen OMPL (muestreo, con `RRTConnect` como planificador típico) y
Pilz (secuencial). El pipeline por defecto será OMPL, que es el que aparece en la lista de
planificadores de RViz.

### 4.9 La escena de planificación y el *Planning Scene Monitor*

La **escena de planificación** es el "estado del mundo" que MoveIt conoce: qué objetos hay, dónde
está el robot, qué colisiones hay. El **Planning Scene Monitor** es el que la mantiene y la
publica en el topic `monitored_planning_scene`, que es lo que consume el panel MotionPlanning de
RViz.

El launch controla uno de sus parámetros con el argumento
`publish_robot_description_semantic`: si vale `true`, `move_group` publica también el SRDF, y sin
él el panel de RViz no puede mostrar los grupos de planificación.

### 4.10 MoveIt Servo

Ya se ha explicado en §3.10. La idea para memorizar: **Servo no es MoveIt**. Servo es un bucle
rápido de teleoperación que vive en el mismo framework pero no planifica. Por eso son dos nodos,
dos configuraciones y dos argumentos de launch independientes.

---

## 5. Cómo se conectan todos los archivos entre sí

### 5.1 Cómo se carga la configuración

```
  staubli_robot_description              staubli_moveit_config
  ------------------------               ---------------------
  urdf/staubli.urdf.xacro        ------>  .setup_assistant
  + config/<modelo>/*.yaml                 (dice de donde leer el URDF
  + meshes/ + r2c/                           y con que argumentos generarlo)
          |
          |  xacro
          v
     robot_description --------------------+
                                          |
  srdf/staubli.srdf.xacro  --------------+
          |  xacro                         |
          v                                |
  srdf/staubli_macro.srdf.xacro            |
          |                                |
          v                                |
     <group name="arm">                    |
     <chain base_link="base_link"          |
            tip_link="tool0"/>             |
     <disable_collisions/>  (11 pares)     |
                                          |
  config/kinematics.yaml  (arm -> KDL) ----+
  config/joint_limits.yaml (vel. y acel.) -+
  config/moveit_controllers.yaml ---------+
  config/pilz_cartesian_limits.yaml ------+
          |                                |
          +--------> MoveItConfigsBuilder.to_dict()
                                   |
                                   v
                         parametros de ROS 2
                                   |
                                   v
                           +----------------+
                           |   move_group   |
                           +----------------+
```

La flecha que viene de `staubli_robot_description` es importante: **el URDF entra en MoveIt por
el mismo doorway que el SRDF**. Los dos documents llegan al mismo nodo, pero como parámetros
distintos.

### 5.2 Los dos caminos de los topics

```
  ROS 2                                  MoveIt
  =====                                  =====

  /robot_description  (de robot_state_publisher)
        |
        +--> RViz: display RobotModel
        |
        +--> (tambien como parametro, no topic, en el panel MotionPlanning)


  /joint_states  (de joint_state_broadcaster o del driver real)
        |
        +--> robot_state_publisher --> /tf
        |
        +--> move_group        (sabe donde esta el robot ahora mismo)
        +--> moveit_servo      (idem, para cerrar el lazo de control)

  move_group
        |
        +--> monitored_planning_scene --> RViz: panel MotionPlanning
        |     (objetos, robots, colisiones)
        |
        +--> /get_planning_scene <--- moveit_servo lo consulta para
        |                            comprobar colisiones antes de mover
        |
        +--> action follow_joint_trajectory
              --> joint_trajectory_controller --> ros2_control --> robot
```

**Por qué el panel MotionPlanning necesita el SRDF.** RViz no lee el SRDF de un topic, lo lee de
un **parámetro**. Por eso el launch le pasa explícitamente `robot_description_semantic` al nodo
`rviz2` (`staubli_moveit.launch.py:166-176`): sin ese parámetro, el panel no sabría qué grupos de
planificación existen y aparecería sin la lista de grupos.

**Las uniones que conviene memorizar**, porque son las que hacen que el conjunto funcione:

| Union | Quién la conecta | Qué pasa si falla |
|---|---|---|
| URDF ↔ MoveIt | `.setup_assistant` | MoveIt no encuentra el robot. |
| Nombres de links ↔ SRDF | `default_names.yaml` (compartido) | El grupo de planificación no existe. |
| Nombre del grupo ↔ solver | `arm` en ambos ficheros | No hay cinemática inversa. |
| Joints del grupo ↔ controlador | `joint_1..6` en `moveit_controllers.yaml` | La ejecución falla. |
| `move_group` ↔ RViz | topic `monitored_planning_scene` | El panel MotionPlanning sale vacío. |
| `move_group` ↔ Servo | nombre del nodo y `planning_frame` | El servo no encuentra la escena. |

---

## 6. Flujo general: del modelo del robot a la planificación y la ejecución

Recorrido completo, siguiendo lo que hace realmente el código.

**Paso 1. Se lanza el punto de entrada.** En la práctica hay dos formas equivalentes:

- `ros2 launch staubli_bringup launch_demo.launch.py`, que es el atajo: ese fichero **incluye los
  dos** (`launch_demo.launch.py:55-83`), con `use_mock_hardware=true`, `start_controller=
  joint_trajectory_controller` y `gui=false` para el launch de control, de forma que solo se abre
  **un** RViz, el de MoveIt.
- O bien lanzar por separado `launch_robot_control.launch.py` y `launch_moveit.launch.py` (en dos
  terminales, o con el primero ya en marcha).

En los dos casos el resultado es el mismo grafo de ROS.

**Paso 2. `launch_robot_control.launch.py` pone en marcha la descripción y el control:**

- Ejecuta `xacro` sobre `staubli_robot_description/urdf/staubli.urdf.xacro` con el modelo pedido y
  con `use_mock_hardware` o `robot_ip` según se quiera hardware real o simulado.
- Lanza `robot_state_publisher` con ese URDF, que lo publica en el topic `/robot_description` y
  calcula el árbol de transforms a partir de `/joint_states`.
- Lanza `ros2_control_node` con `staubli_bringup/config/controllers.yaml`.
- Carga `joint_state_broadcaster` (que empieza a publicar `/joint_states`) y
  `joint_trajectory_controller`, este último con `--inactive` y activado después si
  `start_controller` lo indica.

**Paso 3. `wait_for_robot_ready.py` espera.** Se queda escuchando `/robot_description` y
`/joint_states` con las QoS correctas. Solo cuando ha recibido uno de cada uno, lanza el
`IncludeLaunchDescription` de MoveIt. Esto evita el clásico "move_group arranca antes de que exista
el robot y se queda colgado".

**Paso 4. `staubli_moveit.launch.py` construye la configuración.** El `MoveItConfigsBuilder` lee
`.setup_assistant` y carga el URDF (con `tx2_60l` fijo), procesa el SRDF de `srdf/` con el modelo
del argumento, y carga `kinematics.yaml`, `joint_limits.yaml`, `moveit_controllers.yaml` y
`pilz_cartesian_limits.yaml`. Todo eso se convierte en un diccionario de parámetros de ROS 2.

**Paso 5. Arranca `move_group`.** Con todos esos parámetros, construye su `RobotModel`:

- Del URDF saca la geometría, los links, los joints, los límites de posición y las inercias.
- Del SRDF saca el grupo `arm`, sus seis articulaciones, y qué pares de links no hay que comprobar.
- De `kinematics.yaml` saca que `arm` se resuelve con KDL.
- De `joint_limits.yaml` saca las velocidades y aceleraciones con las que planifica.
- De `moveit_controllers.yaml` sabe que, cuando tenga una trayectoria, debe mandarla al action
  server de `joint_trajectory_controller`.

**Paso 6. Arranca RViz con el panel MotionPlanning.** El panel recibe el URDF, el SRDF y los
parámetros de planificación, y se suscribe a `monitored_planning_scene`. En la interfaz aparecen
los grupos de planificación del SRDF: en este caso, `arm`.

**Paso 7. El usuario planifica.** Toca *Plan* en RViz. La petición sale por un action de
`moveit_msgs` hacia `move_group`, que:

1. Coge el estado actual del robot de `/joint_states`.
2. Resuelve la cinemática inversa del objetivo → valores de las seis articulaciones.
3. Ejecuta el planificador (OMPL) para encontrar un camino sin colisiones.
4. Lo parametriza en el tiempo respetando los límites de `joint_limits.yaml`.
5. Devuelve la trayectoria, que aparece animada en RViz sobre `display_planned_path`.

**Paso 8. La ejecución.** Si además se pulsa *Execute*, `move_group` envía la trayectoria al
action server `/joint_trajectory_controller/follow_joint_trajectory`, que la `controller_manager`
delega en el `joint_trajectory_controller`, que a su vez escribe en las interfaces de posición de
`ros2_control`, que llegan al robot (real, simulado con mock o con Gazebo).

**Resumen del flujo en una línea:**

```
config/*.yaml + urdf/*.xacro ──xacro──► URDF ──► robot_state_publisher ──► /robot_description, /tf
                                                            │
srdf/*.xacro ──xacro──► SRDF ─┐                           ├──► move_group ──► moveit_controllers.yaml
kinematics.yaml ──────────────┤                           │        │
joint_limits.yaml ────────────┼──► parámetros de ROS 2 ────┘        └──► joint_trajectory_controller
pilz_cartesian_limits.yaml ───┘                                            │
                                                                            ▼
                                                          ros2_control ──► robot real / mock / Gazebo
```

**Y el paso alternativo, el teleoperado**, que evita los pasos 6-8 de planificación:
`moveit_servo` recibe comandos de velocidad, consulta la escena de planificación a `move_group`
para comprobar colisiones, y publica posiciones articulares a un `forward_command_controller`.

---

## 7. Resumen final

`staubli_moveit_config` es la capa que convierte "un robot con seis articulaciones" en "un brazo
que se puede planificar". Su trabajo se reparten en tres cosas muy distintas:

1. **La semántica**, en `srdf/`: un único grupo de planificación llamado `arm` (la cadena de
   `base_link` a `tool0`) y once pares de links con la comprobación de colisión deshabilitada, todo
   ello construido a partir de los mismos nombres que usa el URDF para que no puedan
   desincronizarse.
2. **Los parámetros de planificación**, en `config/`: qué solver de cinemática inversa usar (KDL
   para el grupo `arm`), con qué velocidades y aceleraciones planificar (unos valores tres veces
   más conservadores que los límites físicos del robot), y qué límites cartesianos aplicar si se
   usa el planificador Pilz.
3. **El pegamento con el resto del sistema**: `moveit_controllers.yaml` (que relaciona cada
   joint con el action server de `ros2_control`), `moveit.rviz` (el panel `MotionPlanning`, que
   es donde el usuario ve los grupos y lanza la planificación) y `staubli_moveit.launch.py` (que
   arranca `move_group`, opcionalmente Servo, y RViz).

Lo más importante que hay que entender de este paquete es que **no hace nada por sí solo**. No
genera el URDF (lo pide prestado a `staubli_robot_description`), no crea los controladores (los
crea `staubli_bringup`), y ni siquiera arranca el nodo de descripción del robot. Por eso su
`launch` lleva un comentario explícito recordando que `robot_state_publisher` debe estar ya en
marcha, y por eso el bringup espera a recibir `/robot_description` y `/joint_states` antes de
lanzarlo. Todo el valor del paquete está en la **configuración**, y es una configuración que hay
que leer con el URDF abierto al lado, porque cada fichero es una capa de significado encima de los
mismos seis articulations.

Si hubiera que señalar los puntos que conviene revisar en el proyecto, con lo que hay en los
ficheros hoy, serían cuatro, y ninguno impide que el sistema funcione:

- `.setup_assistant` apunta a un SRDF (`config/tx2_60l.srdf`) que no existe, y declara un Xacro
  `control_xacro` que ya no está en el paquete de descripción. Son restos de la generación
  original, enmascarados porque el launch sobreescribe la ruta del SRDF.
- `robot_description` queda fijado a `tx2_60l` por el `.setup_assistant`, mientras que el SRDF sí
  sigue el argumento `robot_model` del launch.
- `staubli_servo.yaml` apunta a `/forward_position_controller/commands` (un controlador que no
  existe en el repositorio) y a un nodo llamado `staubli_arm` (que en realidad se llama
  `move_group`).
- El test registrado se llama `test_default_srdf` pero valida el URDF, no el SRDF, y su parámetro
  `prefix` no tiene efecto porque el Xacro espera `robot_prefix`.
