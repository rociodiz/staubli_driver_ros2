# Explicación del paquete `staubli_driver_ros2`

Guía de estudio del paquete `staubli_driver_ros2`. Está escrita para alguien que está aprendiendo
ROS 2 y que quiere entender **qué papel juega este paquete dentro del proyecto
`staubli_driver_ros2`** y, sobre todo, **por qué existe un paquete que no contiene ni una línea de
código del robot**.

La idea central es una sola, y conviene enunciarla antes de entrar en detalles:

> `staubli_driver_ros2` **no programa el robot**. Es el *meta-paquete* del repositorio: no
> compila ejecutables, no configura controladores y no habla con el controlador CS9. Su trabajo es
> (a) declarar que el stack completo debe instalarse junto, y (b) contener **toda la documentación**
> del proyecto: la guía de instalación, los procedimientos de puesta en marcha, los ejemplos de uso
> y la referencia de API generada automáticamente a partir del driver de C++.

Todo lo que sigue está contrastado con el contenido real de los ficheros de la carpeta, con la
configuración de los demás paquetes del repositorio y con los ficheros de GitHub Actions que
publican la documentación. Cuando algo no puede determinarse a partir de los ficheros, se indica
explícitamente.

---

## 1. Qué es `staubli_driver_ros2` y qué papel tiene en el proyecto

### 1.1 El concepto de "meta-paquete"

En ROS 2, un **paquete** (`package`) es la unidad mínima de distribución de código: una carpeta
con un `package.xml` y un sistema de build (`CMakeLists.txt` o `setup.py`). Lo habitual es que cada
paquete resuelva **una** cosa.

Un **meta-paquete** (`metapackage`) rompe esa regla a propósito. No implementa funcionalidad propia;
su único trabajo es **depender de otros paquetes** para que se instalen todos a la vez. Es
exactamente el mismo concepto que en otros lenguajes se llama *paquete agregador*, *paquete fachada*
o *paquete de nivel superior*, y en Python se parece a un `__init__.py` que no hace nada más que
importar.

La gracia de esto aparece en el momento de instalar. Con un meta-paquete, el usuario hace:

```bash
sudo apt install ros-<distro>-staubli-driver-ros2
```

y se le instalan de golpe el driver, la descripción del robot, la configuración de MoveIt y los
ficheros de arranque. Sin el meta-paquete, tendría que saber él mismo qué siete paquetes forman el
proyecto, y bastaría con que se le olvidara uno para que la instalación pareciera rota.

En este repositorio el meta-paquete hace además una segunda cosa, que no es típica: **contiene la
documentación**. Esa es la razón por la que su `CMakeLists.txt` tiene un `option(BUILD_DOCS)` y un
`add_custom_target(Sphinx ...)`, algo que no se vería en un meta-paquete normal.

### 1.2 ⚠️ La homonimia: dos cosas que se llaman `staubli_driver_ros2`

Este es el punto que más confunde a quien empieza, y conviene resolverlo de entrada. En el árbol de
ficheros hay **dos directorios distintos** con el mismo nombre:

```
staubli_ws/src/staubli_driver_ros2/            <- (A) EL REPOSITORIO GIT
│
├── staubli_robot_description/                 <- paquete ROS 2
├── staubli_robot_driver/                      <- paquete ROS 2
├── staubli_moveit_config/                     <- paquete ROS 2
├── staubli_bringup/                           <- paquete ROS 2
├── staubli_gui/                               <- paquete ROS 2
├── staubli_joystick_servo/                    <- paquete ROS 2
├── staubli_trajectory/                        <- paquete ROS 2
│
└── staubli_driver_ros2/                       <- (B) EL PAQUETE ROS 2 META-PACKAGE
    ├── package.xml
    ├── CMakeLists.txt
    ├── README.md
    ├── requirements.txt
    ├── doxygen/
    └── sphinx/
```

- **(A) `staubli_driver_ros2/` (el repositorio)** es lo que se clona con `git clone`. Contiene el
  código, los `.github/workflows/`, el `.docker/`, la licencia, el `CHANGELOG.md` y **dentro** los
  ocho paquetes de ROS 2. **No es un paquete**: no tiene `package.xml` en la raíz y por tanto
  `colcon` lo ignora.
- **(B) `staubli_driver_ros2/staubli_driver_ros2/` (el paquete)** es un paquete de ROS 2 de
  verdad, con su `package.xml` y su `CMakeLists.txt`. Es el que se documenta en este fichero.

Es la misma convención que usan muchos proyectos de ROS 2: el repositorio se llama como el
proyecto y el paquete meta lleva el mismo nombre, un nivel más abajo. La regla práctica para
distinguirlos es fácil: **si la carpeta tiene `package.xml`, es un paquete; si no, es el
repositorio**.

### 1.3 El paquete en la arquitectura del repositorio

El repositorio está partido en piezas muy especializadas. La tabla completa:

| Paquete | Responsabilidad | Tipo de contenido | ¿Depende de él este meta-paquete? |
|---|---|---|---|
| `staubli_robot_description` | **Qué es el robot**: URDF en Xacro, mallas, límites, bloque `<ros2_control>`. | Xacro, YAML, mallas, un launch | **Sí** |
| `staubli_moveit_config` | **Cómo se planifica con el robot**: grupo de planificación, autocolisiones, cinemática inversa, mapeo a controladores. | SRDF en Xacro, YAML, un launch | **Sí** |
| `staubli_robot_driver` | **Cómo se habla con el robot real**: UDP, protocolo del CS9, *hardware interface*, aplicación VAL3. | C++ y Python | **Sí** |
| `staubli_bringup` | **Cómo se arranca todo junto**: qué launch se incluye, en qué orden, y quién espera a quién. | Launch files y configuración de `controller_manager` | **Sí** |
| `staubli_gui` | Interfaz gráfica de escritorio (PySide6) para control manual, cámara y trayectorias. | Python | **No** |
| `staubli_joystick_servo` | Teleoperación cartesiana por velocidad **sin** MoveIt. | Python | **No** |
| `staubli_trajectory` | Cinemática inversa y construcción de trayectorias propias con PyKDL. | Python | **No** |
| **`staubli_driver_ros2`** | **Meta-paquete + documentación.** | `package.xml`, `CMakeLists.txt`, `sphinx/`, `doxygen/` | Es este |

**La observación importante sobre la última columna.** `package.xml:17-20` declara exactamente
cuatro dependencias:

```xml
<exec_depend>staubli_bringup</exec_depend>
<exec_depend>staubli_moveit_config</exec_depend>
<exec_depend>staubli_robot_description</exec_depend>
<exec_depend>staubli_robot_driver</exec_depend>
```

Es decir: la lista de "lo que hay que instalar para tener un driver funcional" está **completa y
correcta**. Los tres paquetes que faltan (`staubli_gui`, `staubli_joystick_servo`,
`staubli_trajectory`) son funcionalidad **experimental y accesoria** —una interfaz gráfica de
escritorio y dos utilidades de teleoperación—, y se describen en el `CHANGELOG.md` del proyecto
como añadidos posteriores. Excluirlos del meta-paquete es, por tanto, una decisión coherente
(depender de ellos forzaría a instalar PySide6 a quien solo quiere un driver headless), aunque
también es cierto que la documentación de este paquete los menciona (§4.7 y §5.3), lo que crea una
pequeña asimetría entre lo que se documenta y lo que se instala.

### 1.4 Lo que contiene, y lo que no contiene

**Contiene, en términos de "cosas que hacen algo":**

- Un manifiesto de dependencias (`package.xml`).
- Un guion de CMake que, opcionalmente, compila la documentación (`CMakeLists.txt`).
- La lista de herramientas de Python que necesita esa compilación (`requirements.txt`).
- Una configuración de Doxygen que extrae la API del driver C++ (`doxygen/`).
- Un sitio Sphinx completo con la guía de instalación, los procedimientos de arranque, las demos y
  los Troubleshooting (`sphinx/`).

**No contiene, y conviene tenerlo presente:**

- Ni una línea de C++, ni de Python, ni de VAL3.
- Ningún *launch file*. El punto de entrada de ejecución es `staubli_bringup`, no este paquete.
- Ningún mensaje, servicio ni *topic*. No publica niRos suscribe nada.
- Ninguna configuración de controladores. Vive en `staubli_bringup/config/controllers.yaml`.
- Ninguna geometría. Las mallas están en `staubli_robot_description/meshes/`.

Y una verificación que reinforce ese punto: **ningún paquete del repositorio declara
`staubli_driver_ros2` como dependencia**, ni siquiera él mismo. Búsqueda literal de
`staubli_driver_ros2` en todos los `package.xml` del árbol: solo aparecen las dos líneas de su
propio manifiesto (`package.xml:4` y `package.xml:6`). Es decir, nada en el robot depende de este
paquete para funcionar, y nadie lo busca con `FindPackageShare("staubli_driver_ros2")` ni con
`get_package_share_directory("staubli_driver_ros2")` (verificado: cero coincidencias en todo el
repositorio). **Es una hoja de entrada al proyecto, no una pieza del robot.**

---

## 2. Estructura de la carpeta

```
staubli_driver_ros2/staubli_driver_ros2/
├── package.xml                   Metadatos y las 4 dependencias del stack
├── CMakeLists.txt                33 líneas. No compila nada; opcionalmente compila la doc
├── README.md                     Instrucciones para generar la documentación en local
├── requirements.txt              Dependencias pip de la documentación (Sphinx y extensiones)
│
├── doxygen/                      Generación de la referencia de API del driver C++
│   ├── COLCON_IGNORE             Fichero vacío: colcon no entra en esta carpeta
│   ├── Doxyfile                  32 líneas. Apunta a staubli_robot_driver/include
│   ├── doxygen-awesome.css       Hoja de estilo externa (63 KB, de la comunidad Doxygen)
│   └── mainpage.md               Página principal de la referencia, en Markdown
│
└── sphinx/                       El sitio de documentación
    ├── COLCON_IGNORE             Fichero vacío
    ├── conf.py                   Configuración de Sphinx. Lanza Doxygen como efecto secundario
    ├── index.rst                 Portada de la versión HTML
    ├── index_pdf.rst             Portada de la versión PDF (con front-matter propio)
    ├── toc.rst                   Índice maestro; se usa desde los dos portadas
    ├── changelog.rst             Incluye el CHANGELOG.md de la raíz del repositorio
    ├── Makefile / make.bat       Atajos de línea de comandos de Sphinx (Linux / Windows)
    ├── _static/css/custom.css    CSS propio: permite que las tablas se ajusten al ancho
    ├── images/                   Logotipos, cabecera, capturas y el diagrama del stack
    ├── frontmatter_pdf/          Introducción y descargo de responsabilidad (solo PDF)
    ├── quickstart/               5 páginas: qué es el stack, instalación, arranque, modelos
    ├── demo_applications/        6 páginas: MoveIt, Docker, logs, diagnóstico, GPIO, problemas
    └── developer_guide/API.rst   Enlace a la referencia Doxygen generada
```

**Ausencias que son igual de informativas que las presencias:**

- **No hay `test/`.** El paquete no registra ningún test, ni en `CMakeLists.txt` ni en
  `package.xml`. Es coherente con no tener código.
- **No hay `setup.py` ni `setup.cfg`.** El build es `ament_cmake`, no `ament_python`.
- **No hay `launch/`.** Ningún proceso se arranca desde aquí.
- **No hay `config/`.** No hay ni un solo parámetro de robot en esta carpeta.
- **No hay `sphinx/_templates/`**, aunque `conf.py:93` declara `templates_path = ["_templates"]`.
  Sphinx tolerará la carpeta ausente, pero la línea es un resto.
- **No hay `sphinx/_build/`.** Es la carpeta de salida y se crea al compilar; no está versionada.
- **No hay `sphinx/_static/` con más que `css/custom.css`.** Las imágenes viven en `images/`, no en
  `_static/`, que es la convención clásica de Sphinx pero no es obligatoria.

---

## 3. Los conceptos que hay que entender antes de seguir

Esta sección explica el vocabulario en lenguaje sencillo. Los conceptos aparecen por todas partes
en este paquete (especialmente en `sphinx/`), y sin ellos el resto del documento es ruido.

### 3.1 `ros2_control` y el *hardware interface*

**`ros2_control`** es el framework de ROS 2 que hace que un robot sea **programable con
herramientas genéricas**. La idea es separar por completo *qué se quiere hacer* (planificar una
trayectoria, leer el estado) de *con qué máquina se hace*.

- `hardware_interface` es la biblioteca que define **cómo se habla con un robot real o simulado**.
- `controller_manager` es el nodo que carga la interfaz de hardware y los *controladores* y hace
  funcionar el robot a 250 Hz.

**El punto clave para entender este paquete:** `hardware_interface` no sabe nada de robots
concretos. Espera a que alguien le *enchufe* una clase que sepa hablar con tu máquina. Ese
"alguien" es un **plugin**.

- **`pluginlib`** es el mecanismo de ROS 2 para cargar piezas intercambiables en tiempo de
  ejecución. Un paquete declara sus plugins en un XML y `ros2_control` los busca al arrancar.
- **`SystemInterface`** es la clase base que debe heredar el plugin. Es la abstracción que
  `ros2_control` exige: una serie de métodos (`on_init`, `read`, `write`,
  `export_state_interfaces`, `export_command_interfaces`, ...) que describen cómo leer el estado
  del robot y cómo enviarle órdenes.
- **`state_interface`** es un valor que se **lee** del robot (posición, velocidad, par, estado de la
  seguridad...). **`command_interface`** es un valor que se **escribe** hacia el robot (posición
  objetivo, velocidad objetivo...).

El nombre formal de `<plugin name="..."/>` en el URDF es `<hardware>`. Es lo que decide si el
`controller_manager` habla con el robot real, con un simulador o con un modelo ficticio. En este
proyecto la elección está en el bloque `<ros2_control>` que genera
`staubli_robot_description/r2c/staubli.r2c.xacro:36-59`.

### 3.2 Topics, servicios y acciones

ROS 2 tiene tres formas de falar entre procesos. La diferencia es el **modelo de comunicación**:

| Mecanismo | Modelo | Ejemplo en este proyecto |
|---|---|---|
| **Topic** | Difusión, sin respuesta. "Todo el que quiera saber, que escuche." | `/joint_states` (estado del robot), `/tf` (transformadas) |
| **Servicio** | Petición y respuesta puntual, uno a uno. | `/controller_manager/switch_controller` (activar un controlador) |
| **Acción** | Petición, progreso y resultado, con posibilidad de cancelar. | `/joint_trajectory_controller/follow_joint_trajectory` |

Una **acción** es la más interesante de las tres para robots, porque resuelve un problema real:
mover un brazo tarda varios segundos, y durante ese tiempo hay que poder **ver el progreso** y
**cancelar** si algo va mal. Por eso "ejecutar una trayectoria" es una acción y no un topic.

Los **puertos** son el otro concepto imprescindible para este proyecto, y no son los puertos de
TCP. En `ros2_control`, "puerto" (*port*) es cada uno de los puntos de conexión abstracted del
robot: una entrada, una salida, una esquina, una velocidad. La nomenclatura es estricta:

- `<joint_name>/position` — la posición del codo
- `<joint_name>/velocity` — su velocidad
- `supervision/estop_pressed` — un booleano de seguridad, no una.articulación

### 3.3 URDF, Xacro y el `robot_state_publisher`

- El **URDF** (*Unified Robot Description Format*) es un XML que describe la estructura mecánica de
  un robot: qué *links* (piezas rígidas) existen, qué *joints* (articulaciones) los unen, con qué
  geometría, y entre qué ángulos se mueven. Lo produce `staubli_robot_description`.
- El **Xacro** es un preprocesador: permite escribir XML con macros, variables y condicionales, en
  lugar de un XML plano enorme. El URDF de este proyecto es un Xacro parametrizado por modelo
  (`tx2_60`, `tx2_60l`, `tx2_60l_med`).
- **`robot_state_publisher`** es un nodo estándar de ROS 2 que recibe el URDF y las posiciones
  actuales de las articulaciones, y publica el árbol de transformadas (`/tf`) para que RViz pueda
  dibujar el robot en la pose correcta.

### 3.4 UDP, sockets, mensajes y serialización

Este es el lenguaje en el que hablan el PC y el robot, y es la parte más "de bajo nivel" del
proyecto.

- Un **socket** es un punto de intercambio de datos en la red. Puede ser **TCP** (conexo, fiable,
  ordenado) o **UDP** (sin conexión, sin garantías, muy rápido). Este proyecto usa **UDP**, que
  es lo habitual en control de robots: la latencia importa más que la entrega perfecta, porque un
  paquete perdido se sustituye por el siguiente 4 ms después.
- Un **mensaje** es la estructura de datos que viaja. No es texto: son **bytes**. El formato
  binario está descrito en `staubli_robot_driver/include/staubli_robot_driver/communication/protocol.hpp`.
- La **serialización** es el proceso de convertir una estructura en memoria (`struct`, `class`) en
  una secuencia de bytes, y viceversa. Sin ella no se puede cruzar un límite de proceso o de
  máquina. En este proyecto la implementa `serialization.hpp`, y hay una decisión relevante: los
  valores articulares viajan en **precisión simple** (`float`, 4 bytes) aunque ROS 2 trabaje con
  `double` (8 bytes), para ahorrar tráfico.
- Todos los números de más de un byte van en **little endian** (*byte menos significativo primero*),
  y ambos extremos del enlace deben coincidir en eso.

El mensaje tiene una **cabecera** de 8 bytes común a todos los tipos, que permite al receptor saber
qué ha recibido y descartar basura:

```
| Magic (2 B) | Versión (1 B) | Tipo (1 B) | Secuencia (2 B) | Tamaño payload (2 B) |
```

### 3.5 `ament_cmake`, `colcon` y `COLCON_IGNORE`

- **`ament_cmake`** es el sistema de build de ROS 2 basado en CMake. Un `CMakeLists.txt` de ROS 2
  termina siempre con `ament_package()`, que genera los ficheros de índice que permiten que
  `ros2` y `launch` encuentren el paquete tras instalarlo.
- **`colcon`** es la herramienta que compila todos los paquetes de un *workspace* a la vez y
  resuelve el orden correcto. Acepta `project(nombre NONE)`, que significa "no activa ningún
  lenguaje de compilación": CMake se ejecuta, pero no se compila nada.
- **`COLCON_IGNORE`** es un fichero **vacío** cuya mera existencia le dice a `colcon` "no entres en
  esta carpeta". Es la forma canónica de excluir directorios que contienen ficheros que no son
  paquetes (documentación, código del robot, datos). Este paquete usa esa técnica en `doxygen/` y
  `sphinx/`, que contienen un `conf.py` y un `Doxyfile` que, sin el `COLCON_IGNORE`, `colcon`
  intentaría interpretar como paquetes.

### 3.6 Sphinx, Breathe y Doxygen

La documentación de este proyecto es **documentación escrita a mano** (en `reStructuredText`, un
lenguaje de marcado) **más documentación generada automáticamente** del código C++ del driver.
Son dos herramientas distintas que se combinan:

- **Sphinx** es el generador del sitio. Toma los ficheros `.rst` de `sphinx/` y produce el HTML (o
  el PDF) final. Es como generar un sitio web entero desde un conjunto de ficheros de texto.
- **Doxygen** lee el **código fuente** y extrae la documentación de la API: clases, métodos,
  parámetros, Diagramas de herencia. Es la herramienta clásica de C++.
- **Breathe** es el puente entre ambos: le dice a Sphinx dónde están los XML que generó Doxygen
  para que los incruste en la página. Sin Breathe, el sitio no puede documentar clases de C++.
- **`.rst` frente a `.md`**: RST es el formato nativo de Sphinx; Markdown es más extendido, y por
  eso el proyecto instala `sphinx_mdinclude`, que permite incrustar ficheros `.md` (como el
  `CHANGELOG.md` de la raíz). Markdown **no** es el formato principal aquí.

### 3.7 Contenedores, CI y GitHub Pages

Fuera del paquete, pero es lo que hace que la documentación exista: los ficheros
`.github/workflows/documentation.yml` construyen la documentación en cada `push` a `main` y la
publican en GitHub Pages. El `.github/workflows/ci.yml` compila y testea el stack en Ubuntu Noble
con ROS 2 Jazzy. El `.docker/` construye una imagen con todo precompilado. Ninguno de estos
artefactos pertenece al paquete: están en la raíz del repositorio.

---

## 4. Fichero por fichero

### 4.1 `package.xml` — el manifiesto

**Qué es.** El documento de identidad del paquete, en formato 3.

**Qué contiene, y qué significa cada cosa:**

| Campo | Valor | Lectura |
|---|---|---|
| `<name>` | `staubli_driver_ros2` | El nombre del paquete. |
| `<version>` | `0.1.4` | Coincide con la del resto de paquetes del repositorio. |
| `<description>` | *"Meta-package aggregating the staubli_driver_ros2 packages and documentation"* | Dice en una línea qué es. Es exactamente lo que es. |
| `<maintainer>` / `<author>` | Thibault Poignonec | Persona responsable. |
| `<license>` | Apache License 2.0 | Coherente con el `LICENSE` de la raíz. |
| `<buildtool_depend>` | `ament_cmake` | El build es CMake. |
| `<build_depend>` | `python3-sphinx`, `doxygen` | **Declarados como `build_depend`, no `exec_depend`**: se necesitan para construir (la documentación), no para ejecutar. Es la única sutileza del manifiesto. |
| `<exec_depend>` ×4 | Los cuatro paquetes del stack | Lo que hay que instalar para tener el driver. |
| `<export>` | `build_type: ament_cmake` | Indica a `colcon` cómo construir. |

**Lo que hay que entender.** `<exec_depend>` y `<build_depend>` parecen intercambiables y no lo son.
Un `exec_depend` significa "necesario **cuando el sistema está en marcha**"; un `build_depend`
significa "necesario **mientras se compila**". Confundirlos hace que una máquina en la que se
compiló el paquete luego no pueda generar su documentación, o al revés.

**Lo que este manifiesto NO declara, y es correcto:** `sphinx`, `breathe`, `sphinx_rtd_theme`, etc.
Esas dependencias están en `requirements.txt`, porque son paquetes de **pip**, no paquetes de
ROS 2, y `rosdep` no los gestiona. La división correcta es: `package.xml` para lo de ROS/Debian,
`requirements.txt` para lo de pip.

### 4.2 `CMakeLists.txt` — el guion de build (33 líneas)

**Qué es.** El fichero que CMake ejecuta al construir el paquete. Es corto, y cada línea tiene un
motivo.

**Qué hace, línea a línea.**

1. **`project(staubli_driver_ros2 NONE)`** (`:2`). El `NONE` es lo más importante: le dice a CMake
   "no actives ningún lenguaje de programación". Sin eso, CMake asumiría C++ y crearía un
   ejecutable de prueba vacío. Con `NONE`, el paquete se construye sin compilar una sola línea.
2. **`find_package(ament_cmake REQUIRED)`** y **`ament_package()`** (`:3`, `:33`). El.frame de ROS
   2: `ament_package()` genera el índice que hace que el paquete sea visible tras
   `colcon build`.
3. **`option(BUILD_DOCS "Build the doc (doxygen + sphinx)" OFF)`** (`:6-9`). Una opción que el
   usuario puede activar con `colcon build --cmake-args -DBUILD_DOCS=ON`. Por defecto está
   **desactivada**, lo que significa que un `colcon build` normal del workspace **no genera nada
   de este paquete**.
4. **`if(${BUILD_DOCS})`** (`:11`). Activa todo el bloque. Nótese la forma `if(${VAR})`: se expande
   literalmente a `if(OFF)` o `if(ON)`. Funciona, aunque la forma recomendada en CMake moderno es
   `if(BUILD_DOCS)`, que evalúa la variable por su nombre en vez de por su valor.
5. **La detección de entorno virtual** (`:14-18`). Si estás dentro de un `venv` (`VIRTUAL_ENV`) o
   de un entorno Conda (`CONDA_PREFIX`), deja `pip_args` vacío; si no, lo pone a `--user`. La
   intención es no contaminar un entorno virtual con una instalación global, y no hacer `sudo pip`
   en un Python del sistema.
6. **`execute_process(... pip install -r requirements.txt)`** (`:20`). Instala Sphinx y las
   extensiones **en tiempo de compilación**.
7. **`add_custom_target(Sphinx ALL ...)`** (`:25-30`). Crea un objetivo de build llamado `Sphinx`.
   El `ALL` significa que se ejecuta siempre que se construya el paquete, siempre que
   `BUILD_DOCS` esté activo.
8. **Lo que no hay: ninguna regla `install()`.** No se instala ni un solo fichero. El paquete no
   aporta datos en tiempo de ejecución.

**Un error real en la línea 20.** La variable se define como `pip_args` (línea 15/17) pero se usa
como `${_pip_args}` (línea 20), con un guion bajo delante. En CMake el nombre de la variable es
`_pip_args`, que no existe y se expande a la cadena vacía. El efecto **no es un error de
compilación** (una variable vacía simplemente desaparece de la línea de comandos), sino que la
opción `--user` nunca se pasa nunca. Es decir, la lógica de "instalar en el usuario si no estamos
en un entorno virtual" está muerta. La instalación de Sphinx seguirá funcionando, pero se hará en
el Python del sistema en lugar de en el directorio personal.

**Consecuencia práctica.** El `CMakeLists.txt` de este paquete es, en el uso normal del proyecto,
**completamente inerte**: sin `-DBUILD_DOCS=ON` no hace absolutamente nada. El único producto
real del meta-paquete es la documentación, y se genera fuera de `colcon`.

### 4.3 `README.md` — cómo generar la doc en local

**Qué es.** 50 líneas, en inglés, con el único objetivo de explicar cómo montar la documentación
en la máquina del desarrollador.

**Qué hace.** Describe cinco pasos:

1. Instalar `doxygen`, `doxygen-gui` y `doxygen-doc` por `apt`.
2. Crear un **entorno virtual de Python** con `python3 -m venv .venv`, y —detalle fino— crear un
   `touch .venv/COLCON_IGNORE` dentro. ¿Por qué? Porque el `venv` se crea **dentro de la carpeta
   del paquete**, y si no se le pone el `COLCON_IGNORE`, `colcon` intentaría recorrerlo en cada
   `colcon build` del workspace. Es el mismo truco que se usa en `doxygen/` y `sphinx/`.
3. `pip install -r requirements.txt` dentro del entorno virtual.
4. `sphinx-build -b html sphinx sphinx/_build` (y la variante `-E -a` para forzar reconstrucción
   completa).
5. Opcionalmente, generar el PDF con `-b latex` + `pdflatex`.

**Relación con el resto del proyecto.** Es el mismo procedimiento que automatiza
`.github/workflows/documentation.yml`, paso a paso. Si entiendes el `README.md`, entiendes el CI de
documentación. La diferencia es que el CI no crea un `venv`, sino que hace `pip install` con
`break-system-packages` permitido.

**Observación.** El `README.md` del paquete y el `README.md` de la raíz del repositorio responden a
preguntas distintas y no se contradicen: la raíz explica cómo clonar, compilar y arrancar el
*robot*; el del paquete explica cómo generar la *documentación*. Es una división sana.

### 4.4 `requirements.txt` — las herramientas de documentación

**Qué es.** 22 líneas con los paquetes de pip que la documentación necesita.

**Qué contiene, y para qué sirve cada cosa:**

| Paquete | Para qué |
|---|---|
| `sphinx` | El generador del sitio. Imprescindible. |
| `breathe` | El puente entre Doxygen y Sphinx. Imprescindible para la API de C++. |
| `sphinx_rtd_theme` | El tema visual (el que usa `conf.py:112`). |
| `sphinx-book-theme` | Un tema alternativo, actualmente **no usado** (`html_theme_options` tiene un comentario que lo menciona como alternativa sin activarlo, `conf.py:112`). |
| `sphinx-autobuild` | Recarga la página del navegador automáticamente mientras se escribe. Es una comodidad de desarrollo local. |
| `sphinx_mdinclude` | Permite incrustar ficheros `.md` dentro del `.rst`. Se usa en `sphinx/changelog.rst`. |
| `sphinxcontrib.plantuml` | Diagrams en sintaxis PlantUML. |
| `sphinxcontrib-video` | Incrustar vídeos. |
| `sphinxcontrib-mermaid` | Diagramas escritos en sintaxis Mermaid, que el propio `conf.py:71` declara como extensión. |
| `sphinx-copybutton` | Añade un botón "copiar" a los bloques de código. |

**Y una cabecera que no es código:**

```
## Requisite debian pck ##
# sudo apt install doxygen doxygen-gui doxygen-doc
```

Es una lista de comprobación en forma de comentario. Es útil: distingue lo que se instala con
`apt` de lo que se instala con `pip`, que es una confusión habitual.

**Observación sobre laInflation.** De las diez entradas, cuatro (`sphinx-book-theme`,
`sphinx-autobuild`, `sphinxcontrib-video`, `sphinxcontrib-mermaid`) **no son necesarias** para el
sitio tal como está: dos no se activan en `conf.py`, y las otras son comodidades o funcionalidad
de futuro. Mantener el superset evita sorpresas si alguien empieza a escribir una página nueva, pero
significa que instalar la documentación descarga más de lo estrictamente necesario.

### 4.5 `doxygen/` — la referencia de API

**Qué es.** La carpeta que le dice a Doxygen **qué código documentar**. No contiene código: contiene
la *configuración* que hace que Doxygen mire dentro del paquete vecino.

#### 4.5.1 `doxygen/Doxyfile` — 32 líneas

**Qué hace, y lo importante de cada ajuste:**

```ini
PROJECT_NAME           = "staubli_driver_ros2"
INPUT                  = ../../staubli_robot_driver/include   # <- LA LÍNEA CLAVE
INPUT += mainpage.md
RECURSIVE              = YES
OUTPUT_DIRECTORY       = _build
GENERATE_LATEX         = NO
GENERATE_XML           = YES     # para Breathe
GENERATE_HTML          = YES     # para el link directo
FILE_PATTERNS          = *.cpp *.h *.hpp *.py *.md
```

- **`INPUT = ../../staubli_robot_driver/include`** es la conexión más importante de todo este
  paquete. Doxygen está en `staubli_driver_ros2/staubli_driver_ros2/doxygen/`, y `../..` desde ahí
  es la raíz del repositorio. Es decir: **el meta-paquete de documentación es el único sitio del
  repositorio desde el que se le pide a una herramienta que entre en el driver de C++.** Ese es
  el sentido concreto de la relación entre ambos paquetes (§5.4).
- **Solo se leen las cabeceras** (`include/`), no los `.cpp`. Es una decisión habitual y sensata:
  lo que un usuario de la API necesita ver son las declaraciones, no la implementación.
- **`RECURSIVE = YES`** hace que también se processen las subcarpetas, que es donde vive la capa
  de comunicación (`communication/protocol.hpp`, `messages.hpp`, `socket.hpp`, ...).
- **`GENERATE_XML = YES` y `GENERATE_HTML = YES` a la vez.** El XML es lo que Breathe inyecta en
  Sphinx; el HTML es lo que permite el enlace directo de `developer_guide/API.rst`.
- **`DOT_IMAGE_FORMAT = svg` con `INTERACTIVE_SVG = YES`**: los diagramas de clases se pueden
  desplazar y hacer zoom, que es lo que hace útil un diagrama de herencia grande.
- **`FILE_PATTERNS` incluye `.py` y `.md`**, aunque solo se le pasa `include/` (que no tiene ni
  uno ni otro) más `mainpage.md`. Es una configuración heredada, no un problema.
- **`EXTRACT_ALL = YES`**: documenta también lo que no tiene comentario. Útil para librerías de
  terceros, ruidoso para código propio, y es la razón de que la referencia tenga entradas sin
  explicación.

**Un detalle sobre rutas que conviene tener presente.** El `Doxyfile` escribe en
`doxygen/_build/` (relativo al `Doxyfile`), mientras que `conf.py:97-99` le dice a Breathe que
busque en `_build/xml/` (relativo a `conf.py`, es decir, `sphinx/_build/xml/`). Son **directorios
distintos** según la ruta que se tome como base. Puede que funcione porque `html_extra_path`
(`conf.py:34`) copia además el HTML de Doxygen al directorio de salida de Sphinx, pero el
resultado merece atención: si la referencia de clases aparece vacía en la documentación publicada,
este es el primer sitio donde mirar. No he podido verificarlo ejecutando la compilación, porque
ni Sphinx ni Doxygen están instalados en este workspace.

#### 4.5.2 `doxygen/mainpage.md` — la portada de la API

**Qué es.** Seis líneas en Markdown que son la portada de la referencia generada.

**Qué contiene.** Una lista de las tres clases que la referencia cubre:

```
staubli_robot_driver::StaubliHardwareInterface
staubli_robot_driver::RobotDriver
staubli_robot_driver::MockRobotServer
```

**Por qué es útil.** Las tres son, exactamente, las tres piezas del driver que un usuario necesitaría
entender: la que se enchufa a `ros2_control`, la que habla UDP, y la que simula al robot para
pruebas. El `USE_MDFILE_AS_MAINPAGE = mainpage.md` del `Doxyfile` la convierte en la portada
automáticamente.

#### 4.5.3 `doxygen/COLCON_IGNORE`

Un fichero de cero bytes. Existe **solo** para que `colcon` no entre en la carpeta. Sin él, `colcon`
miraría dentro de `doxygen/`, vería el `Doxyfile` y el `mainpage.md`, y no encontraría un
`package.xml`: simplemente lo ignoraría, pero con una advertencia. Con el fichero, ni se molesta.
Es la misma razón por la que existe `staubli_robot_driver/val3/COLCON_IGNORE` (allí para proteger
los `.pgx` de VAL3, que no son código de ROS 2 en absoluto).

#### 4.5.4 `doxygen/doxygen-awesome.css`

Una hoja de estilos de 63 KB que **no está escrita en este repositorio**: viene del proyecto
Doxygen Awesome. Se referencia con `HTML_EXTRA_STYLESHEET` en el `Doxyfile`. Su presencia aquí es
solo la copia del fichero para que el build sea reproducible. No hay nada que explicar de su
contenido; es un fichero de terceros que se versiona por comodidad.

### 4.6 `sphinx/` — el sitio de documentación

#### 4.6.1 `sphinx/conf.py` — el corazón de la configuración

**Qué es.** La configuración de Sphinx, 176 líneas. Se ejecuta **cada vez que se compila** la
documentación, y por eso puede (y de hecho lo hace) lanzar otras herramientas.

**Los cuatro bloques que importan:**

**a) Doxygen como efecto secundario del build de Sphinx** (`:32-34`):

```python
subprocess.call("cd ../doxygen; doxygen Doxyfile", shell=True)
html_extra_path = ["../doxygen/_build"]
```

Esta línea es la que hace que la API de C++ aparezca en el sitio: cada vez que se ejecuta
`sphinx-build`, **antes** de generar nada, se lanza Doxygen con el `Doxyfile` de la carpeta
hermana. Es una cadena de herramientas montada a mano, sin `Makefile` ni script intermedio. Es
elegante, aunque frágil: si Doxygen no está instalado, la llamada falla **en silencio** (porque
`subprocess.call` no comprueba el código de retorno) y la documentación se genera igual, solo que
sin la referencia de API.

**b) El número de versión se lee del `package.xml`** (`:42-53`):

```python
package_xml_path = Path(__file__).parent.parent / "package.xml"
...
release = version_element.text     # "0.1.4"
version = ".".join(release.split(".")[:2])   # "0.1"
```

Es un detalle bien hecho: la versión que aparece en la documentación no está duplicada en dos
sitios, sino que sale de la fuente de verdad. Por eso `sphinx/index.rst:29` puede escribir
`**Version:** |release|` y mostrar siempre el valor correcto. El `|` de la línea 29 lo es: en Sphinx,
`substitution` está activado por defecto.

**c) Breathe y el dominio C++** (`:77-101`):

```python
breathe_default_project = "staubli_driver_ros2"
breathe_projects = {"staubli_driver_ros2": "_build/xml/"}
breathe_default_members = ("members", "undoc-members")
primary_domain = "cpp"
highlight_language = "cpp"
```

Estas cuatro líneas hacen dos cosas: le dicen a Breathe dónde están los XML de Doxygen, y cambian
el "lenguaje por defecto" de la documentación de Python (el de Sphinx) a C++. Es el equivalente
en la documentación de lo que hace el `<language>` en un proyecto de Doxygen.

**d) Salida HTML y PDF** (`:110-176`). Tema `sphinx_rtd_theme` (el tema de Read the Docs), logo de
ICube, y una configuración de LaTeX bastante cuidada: tamaño A4, sangrados en el índice, cabeceras
personalizadas con `fancyhdr` y títulos con `titlesec`, y sustituciones de emojis por texto. El
bloque `preamble` incluye un `\DeclareUnicodeCharacter{00A9}` con el comentario *"Replace Unicode
emojis with text representations"*, que existe porque `index.rst:35-46` usa emoji en los avisos de
seguridad (`🛑`, `⚠️`) y `pdflatex` no los sabe dibujar. Es un detalle muy concreto de la vida real
de generar PDF con LaTeX.

**Una función muerta.** `conf.py:84-90` define:

```python
def get_package(package: str):
    path = Path(__file__).parent.parent.parent.joinpath(f"{package}/include/{package}")
    ...
```

No se usa en ninguna parte del fichero (verificado: `get_package` solo aparece en su `def`), ni
en ningún `.rst`. Está muerta. Probablemente fue un intento de automatizar el `INPUT` del
`Doxyfile` (que es exactamente lo que esa función haría) que se abandonó cuando se optó por
declarar la ruta a mano en el `Doxyfile`. Es una pista útil sobre la historia del paquete.

#### 4.6.2 `sphinx/index.rst` — la portada HTML

**Qué es.** La página de entrada de la versión web. Contiene:

- Una imagen de cabecera a ancho completo (`images/staubli_robot_header.jpg`).
- Una descripción del estado del proyecto: ROS 2 Jazzy / Ubuntu 24.04, controladores **CS9**
  únicamente, **solo control de posición implementado**.
- Una **nota de desarrollo** (`:15-24`) que dice, sin rodeos, que el desarrollo está pausado por
  falta de financiación, que el software se comparte "en su estado actual" y que es un **prototipo
  temprano, no un producto industrial**, con un contacto del ICube para_collaborar. Esta honestidad
  es parte de la documentación, no un descuido.
- Un bloque `line-block` con el repositorio, la versión, el copyright, la licencia y el autor.
- Un aviso `danger` con las **instrucciones de seguridad**: probar primero en MANUAL, E-stop a mano
  al pasar a AUTO, empezar con escalado de velocidad del 10 % con MoveIt, y comprobar el movimiento
  en RViz antes de ejecutarlo. Y el descargo de responsabilidad: se distribuye "as is", sin
  garantías, sin responsabilidad por daños físicos o económicos.
- `.. include:: toc.rst` al final, que inserta el índice.

**Sobre los bloques `note`, `warning`, `danger` y `important`:** son *admonitions* de RST, cajas
con color que Sphinx renderiza como recuadros. Es la forma canónica de destacar información
importante, y en un proyecto de robótica su uso es menos decorativo de lo que parece: casi todos
los avisos de esta documentación son de seguridad.

#### 4.6.3 `sphinx/index_pdf.rst` — la portada PDF

**Qué es.** La gemela de `index.rst`, pero para la salida LaTeX, con una diferencia estructural clave.

**Qué hace.** No repite la portada. En su lugar:

```rst
\chapter*{Package overview}          <- sin numerar
\include frontmatter_pdf/intro.rst

\chapter*{Note about safety}
\include frontmatter_pdf/disclaimer.rst

\include:: toc.rst
```

Usa `\chapter*` (asterisco) para crear capítulos **sin número**, y mete el resto del documento en
un `\part` de los que se definen en `toc.rst:1-5, 18-22, 34-44`. El resultado es una portada de
documento oficial, con las partes ("Quickstart", "Demos", "Extra") en páginas de separación, en
lugar de la portada herbácea de la web.

**Por qué existen dos portadas y no una:** porque la portada web lleva dentro el estado de
desarrollo, la nota de financiación y el bloque de metadatos, y eso no tiene sentido en un PDF que
se distribuye a un cliente industrial. El PDF es sobrio; el HTML es transparente. La decisión de
qué contarle a cada lector está tomada con criterio.

#### 4.6.4 `sphinx/toc.rst` — el índice maestro

**Qué es.** 52 líneas que definen el índice de la documentación, con tres `toctree` (Quickstart,
Demos, Extra) y bloques `.. only:: latex` que inyectan `\part{}` para la salida PDF.

**El `:glob:` que aparece en los tres `toctree` es importante**: le dice a Sphinx que las entradas
del índice son *patrones*, no rutas exactas. Así, añadir `quickstart/nueva_pagina.rst` al árbol de
ficheros la hace aparecer automáticamente en el índice sin tocar este fichero.

**Observación verificada: hay dos páginas que existen pero no se publican.** Los ficheros
`demo_applications/diagnostics.rst` y `demo_applications/gpios.rst` están en disco, y ambos tienen
su etiqueta (`.. _demo_diagnostics:` y `.. _demo_gpio:`), pero **ninguno de los dos aparece en
ningún `toctree` de `toc.rst`**. Como no están en el índice, Sphinx no las publica y no avisa. Los
dos ficheros dicen, íntegramente, "Coming soon :)". El resultado es el mismo que si no existieran,
pero explica por qué están ahí: son páginas reservadas.

#### 4.6.5 `sphinx/Makefile` y `sphinx/make.bat`

Los dos atajos estándar que genera `sphinx-quickstart`, para no escribir `sphinx-build` entero cada
vez. `make html` en lugar de `sphinx-build -M html . _build`.

**Observación:** el `Makefile` tiene un `clean` que borra `_doxygen/` y `api/`
(`Makefile:17-18`). **Ninguna de esas dos carpetas existe en la distribución actual**: Doxygen
escribe en `doxygen/_build` y la API se integra en el mismo `_build` de Sphinx. Son restos de una
fase anterior de la configuración. `make clean` no rompe nada, simplemente no limpia lo que
realmente se genera.

#### 4.6.6 `sphinx/_static/css/custom.css`

Ocho líneas de CSS. Lo que hace, en una frase, es forzar a las tablas a Adjustarse al ancho
disponible en lugar de desbordar. Existe porque las tablas de esta documentación tienen muchas
columnas y, con el tema por defecto, se salen de la pantalla. Es el típico detalle que solo se
descubre cuando alguien abre la documentación en un portátil estrecho.

### 4.7 `sphinx/quickstart/` — poner en marcha el stack (5 páginas)

Estas cinco páginas son, en conjunto, **el manual de instalación y arranque** del proyecto, y de
hecho son el contenido más valioso de todo el meta-paquete. Están pensadas para leerse en orden,
que es como las presenta `toc.rst`.

#### 4.7.1 `quickstart/overview.rst` — qué hay en el stack

La página "presentación del proyecto". Define primero la **pila de tres piezas** que da título al
proyecto:

> un servidor en el robot escrito en **VAL3** (el lenguaje propio de Staubli) y un cliente en
> ROS 2 con forma de *hardware interface*, unidos por UDP.

Y después define la **interfaz de hardware**, que es la documentación más útil del proyecto, porque
es el contrato exacto entre el driver y `ros2_control`. La documenta en cuatro bloques:

- **Interfaces de estado articulares** (`<prefijo><joint>/<interfaz>`): `position`, `velocity`,
  `effort` (estimado a partir de la corriente, **no medido directamente** — la distinción es
  importante para no confiar en él como si fuera un sensor de par real).
- **Interfaces de supervisión** (`<prefijo>supervision/<estado>`): diez estados, entre ellos
  `operation_mode`, `safety_status`, `control_sequence_delay` (el retardo total del ida y vuelta del
  bucle, expresado en ciclos), `brakes_released`, `motion_possible`, `estop_pressed`,
  `wait_for_ack`. Estos nombres coinciden exactamente con los que genera
  `staubli_robot_description/r2c/staubli.r2c.xacro:95-108`, es decir, esta página es la
  documentación legible de ese Xacro.
- **Interfaces GPIO** (`<prefijo>gpio/<nombre>`): `digital_input_i` (hasta 16), `analog_input_i`
  (hasta 4).
- **Interfaces de comando**: articulares `position` y `velocity` (**"not implemented yet"**), y
  GPIO `digital_output_i` / `analog_output_i`.

Documenta además los **parámetros** del driver (los que se pasan por el Xacro) y **los scripts
FTP** (§4.7.5).

**Verificación cruzada que vale la pena hacer:** la tabla de modelos (`tx2_60l`,
`tx2_60l_med`) de `quickstart/available_models.rst` coincide con los tres directorios de
`staubli_robot_description/config/`. La documentación y el código están sincronizados.

#### 4.7.2 `quickstart/installation_ros2.rst` — instalación del lado PC

Seis pasos: instalar Ubuntu 24.04 (idealmente con kernel *real-time*), instalar ROS 2 Jazzy,
`colcon`, clonar el repositorio, `rosdep install`, `colcon build --symlink-install
--cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo`, `colcon test`, y `source install/setup.bash`.

**Dos cosas que se pasan de paso y que son importantes para un principiante:**

- **`--symlink-install`** hace que los ficheros de `share/` sean enlaces al árbol de fuentes en vez
  de copias. Consecuencia práctica: si editas un YAML de configuración, **no hace falta
  recompilar**, basta con relanzar. Y a la inversa: si un cambio no se refleja, mira primero si has
  si has olvidado de rehacer `source install/setup.bash`.
- **El orden `colcon build` → `source install/setup.bash`** no es opcional. `colcon` construye,
  pero es `setup.bash` lo que añade los paquetes al entorno del shell. Sin él, `ros2 launch` no
  encuentra nada y el error es críptico.

#### 4.7.3 `quickstart/installation_val3.rst` — instalación del lado robot

La página más específica del proyecto, porque el otro extremo no es un PC sino un **controlador
Staubli CS9**.

Describe:

- **El cableado**: el puerto Ethernet **J204** del CS9 al PC. Documenta explícitamente que hay que
  usar J204 y no otro (el `troubleshooting.rst:33-34` insiste en el mismo punto).
- **Las direcciones IP**: el PC en `192.168.0.1` con máscara `255.255.255.0`, y el robot en
  `192.168.0.254`, comprobable con `ping`.
- **Los *addons* de Staubli** que hacen falta: `motion` y `velocity` se descargan del portal
  MyStaubli y se copian a `/usr/app` **por FTP**, y hay que **reiniciar el controlador**. La nota
  explica algo que explica la mitad de los fallos de campo: `alter` y `advCtrlFunctions` vienen de
  serie, pero **en modo demo**, y **dejan de funcionar a las 2 horas**. No es un fallo del driver:
  es el licenciamiento de Staubli. Y avisa de que por eso `troubleshooting.rst:40-48` tiene una
  entrada dedicada al robot que "se desconecta a los ~2 horas".
- **La subida de la aplicación VAL3** con `ros2 run staubli_robot_driver upload_val3_server.py`, o
  a mano con FileZilla copiando `val3/userapp` a `/usrapp` del robot.
- **La creación de los sockets** en el pendil: `E/S > Socket`, dos sockets UDP, `control` en el
  puerto **11000** y `diagnostics` en el **11001**, ambos con *IP remota* `192.168.0.1`.

**El detalle de "Fin de string = 10 (linux)"** merece explicación para un principiante: en la
configuración de sockets del CS9, ese campo indica qué byte marca el final de un mensaje. El valor
`10` es el salto de línea en Linux, y `-1` en *timeout* significa "sin timeout, esperar
indefinidamente". Configurar esto mal es una causa clásica de que la comunicación no funcione, y por
eso la tabla está en la documentación y no solo en el código.

#### 4.7.4 `quickstart/bringup.rst` — arrancar el driver

La página que une las dos instalaciones. Tiene una estructura de dos fases:

**Fase 1: prueba sin robot, con *mock hardware*.** Antes de tocar el controlador físico, se
arranca todo con `use_mock_hardware:=true` (que es el valor por defecto) y se comprueba con:

```bash
ros2 control list_hardware_interfaces
ros2 control list_controllers
```

Debe aparecer `joint_state_broadcaster` como `active` y las seis interfaces de posición,
velocidad y esfuerzo como `available`. La nota explica que el mock calcula el estado a partir de
la última orden recibida, sin física, y que **sirve para verificar la instalación pero no para
evaluar la dinámica real**.

**Fase 2: con el robot.** Cargar la aplicación desde el pendil (`VAL3 > Stockages > Disk`, luego
`VAL3 > Mémoire` y ejecutar), y lanzar con `use_mock_hardware:=false` y `gui:=true`.

**Y tres notas de precisión que revelan el estado real del driver:**

- La interfaz de comando `velocity` **existe** pero "la implementación interna no está terminada":
  el hardware **rechaza** la petición de cambiar a modo velocidad. Esto coincide exactamente con
  lo que hace el código (`prepare_command_mode_switch` devuelve `ERROR` para `JOINT_VELOCITY`).
- Las interfaces de comando `acceleration` son un **efecto secundario** de proporcionar límites al
  *Joint Limiter*: no son un modo de comando real, y el hardware también las rechaza.
- Si el IP del robot no es el de por defecto, hay que pasarlo con `robot_ip:=<ip>`.

**Patrón importante.** La secuencia "probar primero con mock, luego con el robot real" no es una
recomendación estética: es la forma de comprobar, en segundos y sin riesgo, que la instalación
correcta, que los controladores cargan y que las interfaces están bien declaradas. Los fallos de
configuración se detectan antes de que exista el riesgo de mover un brazo de 60 kg.

#### 4.7.5 `quickstart/available_models.rst` — modelos soportados

Una tabla con los dos modelos que funcionan out-of-the-box: `tx2_60l` (TX2-60L) y `tx2_60l_med`
(TX2-60L MR, con la nota de que el sensor de fuerza/par no está soportado). Se usa el **tag** de
cada modelo, que es el valor que hay que pasar a `robot_model:=` en los launches.

La nota final es de las más útiles del proyecto, porque explica algo que solo se descubre cuando
el robot real no se mueve como se espera: **el periodo de muestreo del controlador varía según el
robot**. El TX2-60L va a 4 ms (250 Hz) y el TX2-60L MR a 1 ms (1 kHz), pero la aplicación VAL3 de
este repositorio está ajustada a **250 Hz**, y se explica cómo cambiarla (editar `l_nCtrlPeriod` en
`start.pgx`, redeplegar, y ajustar el periodo del bucle de control en ROS 2 para que coincidan). Se
menciona que para el TX2-60L MR se ha probado con éxito a 500 Hz.

### 4.8 `sphinx/demo_applications/` — ejemplos de uso (6 páginas)

De las seis, cuatro están publicadas en el índice y dos no (§4.6.4). Las publicadas:

#### 4.8.1 `demo_applications/moveit2.rst` — planificar y ejecutar con MoveIt

El tutorial principal. Documenta primero la **lista de comprobación de seguridad** previa: modo AUTO
(o MANUAL con movimiento habilitado), E-stop no pulsado y seguridad reiniciada, frenos abiertos,
movimiento habilitado (el LED azul fijo, no parpadeando). Solo después, los dos terminales:

```bash
# Terminal 1: el control
ros2 launch staubli_bringup launch_robot_control.launch.py \
    robot_model:=tx2_60l use_mock_hardware:=false \
    start_controller:=joint_trajectory_controller gui:=false

# Terminal 2: la planificación
ros2 launch staubli_bringup launch_moveit.launch.py robot_model:=tx2_60l
```

Y los tres pasos en RViz: arrastrar el asa del efector final, pulsar **Plan**, y si tiene éxito,
**Execute**. Con la advertencia de empezar con escalado del 10 %.

**Lo interesante desde el punto de vista de la arquitectura:** esta página es la prueba de que la
cadena de paquetes funciona de extremo a extremo. Usa `staubli_bringup` (dos veces),
`staubli_robot_description` (vía el Xacro), `staubli_robot_driver` (vía el hardware) y
`staubli_moveit_config` (vía el launch incluido). Y es, de hecho, el **único recorrido documentado
de extremo a extremo** del proyecto.

#### 4.8.2 `demo_applications/moveit2_docker.rst` — lo mismo, pero en Docker

La alternativa para quien no quiere compilar. Instala `rocker` (una herramienta que ejecuta
aplicaciones ROS en contenedores con acceso a hardware y reenvío de GUI) y lanza la imagen
precompilada de GitHub Container Registry:

```bash
rocker --net=host --devices /dev/dri --x11 \
    ghcr.io/icube-robotics/staubli_driver_ros2:${IMG_TAG} \
    ros2 launch staubli_bringup launch_demo.launch.py robot_model:=tx2_60l
```

**Explicar los flags es parte de la página, y con razón:** `--net=host` es lo que permite hablar con
el robot sin configurar nada más (sin él, el contenedor tendría su propia red y no alcanzaría al
robot); `--x11` y `--devices /dev/dri` son lo que permiten ver la ventana y usar la GPU. Sin
explicar esos tres flags, el comando es un conjuro. Con ellos, se ve que cada uno resuelve un
problema concreto deDockerización.

**Detalle que conviene saber:** el `Doxyfile` y el `Dockerfile` de este repositorio están
versionados juntos, y el `Dockerfile` (en `.docker/`, fuera del paquete) hace `colcon build` de todo
el workspace, **incluido este meta-paquete**. Es decir: la imagen de Docker es una de las pocas
cosas donde el meta-paquete participa de verdad, y también allí lo único que aporta es
documentación.

#### 4.8.3 `demo_applications/get_robot_logs.rst` — los logs del robot

Un caso muy práctico: cuando algo va mal, el log **del robot** suele ser más informativo que el del
PC, porque la aplicación VAL3 registra el motivo por el que pasó a `STOP`. La página documenta:

```bash
ros2 run staubli_robot_driver download_logs.py --ros-args -p robot_ip:=<ip>   # → .staubli_robot_logs/
ros2 run staubli_robot_driver show_logs.py                                     # los muestra en el terminal
```

Y muestra un ejemplo real de salida, con colores, del tipo:

```
ERROR ros2_server::controlTask : Error, command sequence delay too high (5 ), going to STOP mode
WARN  ros2_server::controlTask : Error, transitioning from command modes requires a stop
```

Esas dos líneas describen **dos de los fallos más comunes del driver** (§6.6) y son, en la práctica,
la mejor documentación de diagnóstico que tiene el proyecto. Es también la página que mejor ilustra
por qué el meta-paquete existe: un script que está en `staubli_robot_driver/scripts/` se explica en
la documentación del meta-paquete, no en la del driver.

#### 4.8.4 `demo_applications/troubleshooting.rst` — problemas frecuentes

La página más útil en la práctica. Organizada por síntoma, con el formato *"si ves esto, prueba
esto"*, y cada entrada con la causa y la comprobación concretas.

Los cuatro grupos de problemas:

1. **Conexión.** "Failed to connect to robot": cinco comprobaciones en orden (aplicación cargada,
   `ping`, IP del PC, puerto **J204**, sockets del pendil). Y el casoaspero del **desconectado a las
   ~2 horas**, correctamente atribuido a los *addons* en modo demo.
2. **Movimiento.** "Conecta pero el robot no se mueve": cuatro comprobaciones (modo AUTO, E-stop,
   frenos, estado del controlador).
3. **Parada inesperada a mitad de trayectoria.** Explica el `command sequence delay too high (5)`:
   el robot no ha recibido una orden durante 5 ciclos seguidos (~20 ms a 250 Hz), lo que en un robot
   en movimiento es motivo suficiente para pasar a `STOP` por seguridad. Las causas que da son
   realistas: un enlace Ethernet compartido o con *switching* que introduce *buffering* (por eso
   recomienda **cable punto a punto dedicado**), un PC sobrecargado que no cumple los plazos del
   bucle (por eso recomienda kernel de tiempo real), o un driver que ha caído.
4. **Visualización.** RViz vacío (frame fijo equivocado, error al cargar el URDF) y
   `/joint_states` que no se actualiza (JSB caído, falta `robot_state_publisher`, o simplemente el
   topic no publica: `ros2 topic hz /joint_states`).

**Detalle menor pero real:** la entrada de la parada inesperada remite con
`:ref:`control_modes`` a "la secuencia correcta de transición de modos", pero **ninguna página de
la documentación define esa etiqueta** (verificado: las únicas etiquetas existentes son
`overview`, `installation_ros2`, `supported_robots`, `installation_val3`, `mystaubli`, `demo_moveit2`,
`demo_docker`, `demo_diagnostics`, `troubleshooting`, `demo_gpio`, `demo_get_logs`). Ese enlace
está roto: Sphinx lo renderizará como texto sin destino, o generará un aviso. La secuencia correcta
sí está documentada, pero en otro sitio: en el código (`resolve_mode` y
`prepare_command_mode_switch` en `staubli_hardware_interface.cpp:500-578`) y, en forma legible, en el
log de ejemplo de `get_robot_logs.rst`. **Falta la página que lo explique**; es la hueco más
evidente de la documentación actual.

### 4.9 `sphinx/developer_guide/API.rst` — el enlace a la API

Quince líneas, y casi todas son andamiaje. En la salida HTML (`.. raw:: html`) inserta un
`<a href="../html/index.html">` que apunta a la referencia generada por Doxygen. En la salida LaTeX
(`.. only:: latex`) sustituye ese enlace por un texto que remite a la versión online.

**Observación:** la ruta `../html/index.html` no corresponde a la salida estándar de Sphinx (que
sería `_build/index.html`). Es otra indicación de que el enlace se escribió para una estructura de
salida anterior. En una compilación estándar este enlace probablemente no funcione; la vía que sí
funciona es `breathe`, que ya inyecta la referencia Doxygen dentro de la propia página.

### 4.10 `sphinx/frontmatter_pdf/` — intro y descargo (solo PDF)

Dos ficheros que solo se usan desde `index_pdf.rst`:

- **`intro.rst`**: seis líneas de descripción del proyecto y metadatos, con la referencia
  `:ref:`supported_robots``. Es la versión *edificada* de lo que dice `index.rst`.
- **`disclaimer.rst`**: el descargo de responsabilidad y las instrucciones de seguridad. Contiene un
  bloque `.. important::` con el título **"SAFELY FIRST"** y los cuatro puntos de seguridad: probar
  en MANUAL, E-stop a mano, escalado del 10 % con MoveIt, comprobar en RViz.

**Sobre la duplicación del texto de seguridad:** las mismas recomendaciones aparecen en
`README.md` (raíz), `index.rst`, `disclaimer.rst`, `moveit2.rst` y `troubleshooting.rst`. Cinco
copias del mismo texto, en Markdown y en RST, con ligeras variaciones. Es una duplicación real (no
puede evitarse del todo: GitHub renderiza Markdown, Sphinx renderiza RST, y el PDF tiene su
portada), pero conviene saber que **si se cambia una instrucción de seguridad hay que cambiarla en
los cinco sitios**.

### 4.11 `sphinx/changelog.rst` — un archivo de una línea

```rst
.. mdinclude:: ../../CHANGELOG.md
```

**Qué es.** La página de historial, que no contiene nada propio: **incluye el `CHANGELOG.md` de la
raíz del repositorio**. El `../../` sube desde `sphinx/` a la raíz del repositorio, y
`sphinx_mdinclude` (la extensión del `requirements.txt`) hace la inclusión.

**Por qué está bien hecho.** El historial de cambios del proyecto está en **un solo sitio**, en
Markdown, en la raíz, que es donde GitHub lo renderiza automáticamente. La documentación no lo
duplica: lo apunta. Es el mismo criterio que el de la versión leída del `package.xml` (§4.6.1) y el
de los scripts FTP documentados desde aquí pero implementados en `staubli_robot_driver`.

**Y un riesgo que introduce.** La documentación deja de compilar si ese `../../CHANGELOG.md` no
existe. Funciona porque la documentación solo se genera desde el repositorio completo, pero es una
dependencia oculta de una carpeta que, en principio, debería bastarse sola.

### 4.12 `sphinx/images/` — los recursos gráficos

Ocho ficheros, y conviene separarlos por función porque no todos son igual de esenciales:

| Fichero | Función | ¿Se usa? |
|---|---|---|
| `staubli_robot_header.jpg` | Cabecera a ancho completo de `index.rst` | Sí |
| `staubli_driver_ros2_overview.png` | **El diagrama del stack** en `quickstart/overview.rst` | Sí |
| `logo-icube.png` | Logo del ICube (`html_logo` y `latex_logo`) | Sí |
| `logo-conectus-couleur.png` | Logo de SATT Conectus, en la página de créditos | Solo en la raíz |
| `screenshot_download_addon.png` | Captura del portal MyStaubli para descargar *addons* | Sí |
| `screenshot_transfert_addon_to_CS9.png` | Captura de FileZilla con los *addons* transferidos | Sí |
| `screenshot_log_files_dir.png` | Captura de los logs descargados | Sí |

**`staubli_driver_ros2_overview.png` merece atención especial**, porque es la imagen que explica
gráficamente lo que todo este paquete significa: muestra la pila completa, con el servidor VAL3
dentro del controlador CS9 y el cliente ROS 2 en el PC, unidos por UDP. Es, visualmente, la
respuesta a la pregunta "¿qué es el meta-paquete?".

---

## 5. `staubli_driver_ros2` frente a `staubli_robot_driver`

Esta es la relación que el enunciado pide aclarar explícitamente, así que conviene abordarla de
frente, porque es la que más malentendidos genera.

### 5.1 Los dos no se parecen en nada

| | `staubli_driver_ros2` (el meta-paquete) | `staubli_robot_driver` (el driver) |
|---|---|---|
| **Qué es** | Meta-paquete + documentación | El driver de verdad |
| **Tipo de build** | `ament_cmake`, `project(... NONE)` | `ament_cmake`, C++17 |
| **Compila código** | No | Sí: dos librerías compartidas + 4 scripts |
| **Lenguajes** | Python, RST, Markdown, CMake | C++, Python, VAL3 |
| **Ficheros de código** | 1 (`conf.py`) | ~40 |
| **Habla con el robot** | No | Sí, por UDP |
| **Se conecta a `ros2_control`** | No | Sí, es un *plugin* (`SystemInterface`) |
| **Conocen al CS9** | Solo en la documentación | Sí, es su interlocutor |
| **Líneas de `package.xml`** | 25 | 31 |
| **Dependencias de ROS** | 0 (solo 4 paquetes del repo) | 9 (`rclcpp`, `hardware_interface`, `pluginlib`, ...) |
| **Contenido** | `sphinx/`, `doxygen/` | `include/`, `src/`, `scripts/`, `val3/`, `test/` |
| **Es crítico para el robot** | **No** (nadie lo importa) | **Sí** |

**La versión de una frase:** `staubli_robot_driver` es el robot; `staubli_driver_ros2` es el manual
del robot, y además el manual de seguridad de la instalación.

### 5.2 En qué se parecen

- **Comparten versión** (`0.1.4`) y licencia, porque son paquetes del mismo repositorio.
- **Ambos son `ament_cmake`**, así que ambos terminan en `ament_package()`.
- **El `CHANGELOG.md` es del repositorio**, no de cada paquete, así que la versión que aparece en la
  documentación es la de todo el proyecto.
- **Los dos "hablan" de control de posición** (uno lo implementa, el otro lo documenta).

### 5.3 En qué se diferencian en su relación con el resto de paquetes

`staubli_driver_ros2` es el único paquete del repositorio que:

- **Menciona a los siete paquetes hermanos**, incluidos los tres experimentales que no declara
  (§1.3). Su documentación los cubre; su manifiesto no los necesita.
- **Tiene relación con `val3/`, `scripts/` y `include/` de `staubli_robot_driver`** sin
  depender de él en el sentido de build. No hay `#include`, no hay `find_package`, no hay
  `<exec_depend>` hacia `staubli_robot_driver`... salvo la que sí existe, y que es puramente
  declarativa: `<exec_depend>staubli_robot_driver</exec_depend>` en `package.xml:20`.
- **No es importado por nadie**, como ya se comprobó en §1.4.

### 5.4 La única dependencia técnica real: la ruta del `Doxyfile`

Si se busca un acoplamiento de código entre ambos, hay **exactamente uno**, y es una cadena de
texto:

```
Doxyfile:8  →  INPUT = ../../staubli_robot_driver/include
```

Todo lo que hace el meta-paquete con el driver es **leer sus cabeceras con Doxygen**. Esa línea es
el resumen de toda la relación técnica entre los dos paquetes.

**Y el riesgo que implica** es real y merece mención: si alguien renombra o mueve
`staubli_robot_driver/`, o cambia la disposición de las carpetas, el `Doxyfile` no dará ningún error
— Doxygen simplemente no encontrará nada y la referencia de API se generará vacía. Es un
acoplamiento silencioso. La función `get_package()` muerta de `conf.py:84` (§4.6.1) parece haber
sido un intento de evitar exactamente este problema, automatizando la ruta a partir del nombre del
paquete. La solución definitiva sería que `conf.py` generase el `INPUT` del `Doxyfile` antes de
invocarlo, en vez de escribirlo a mano.

### 5.5 Y cómo encajan los demás paquetes

Para completar el cuadro, esta es la posición de cada paquete respecto a la documentación:

| Paquete | ¿Lo menciona la doc del meta-paquete? | ¿Dónde? |
|---|---|---|
| `staubli_robot_description` | Sí, implicitamente | Los modelos (`available_models.rst`), los parámetros y las interfaces (`overview.rst`) |
| `staubli_robot_driver` | Sí, mucho | Toda laQuickstart, los scripts FTP, los Troubleshooting |
| `staubli_bringup` | Sí, mucho | Todos los comandos de `ros2 launch` |
| `staubli_moveit_config` | Sí | La demo de MoveIt 2 |
| `staubli_gui` | **No** | — |
| `staubli_joystick_servo` | **No** | — |
| `staubli_trajectory` | **No** | — |

Las tres ausencias son coherentes entre sí y confirman la lectura de §1.3: la documentación del
meta-paquete cubre **el stack del driver**, que es el que se distribuyó y se documentó primero
(véase el `CHANGELOG.md`: la GUI y la teleoperación son añadidos posteriores). El día que se
documenten, el sitio contendrá dos páginas más y el meta-paquete deberá decidir si las declara
como dependencia.

---

## 6. El flujo completo de comunicación

Esta sección es la que responde a *"¿por qué existe todo esto?"*. Se explica primero el flujo de
ida (una orden de ROS 2 hasta el robot) y luego el de vuelta (el estado del robot hasta ROS 2),
con los números reales del protocolo.

### 6.1 El escenario

Lo que hay en pie cuando todo funciona:

- **En el PC**: `ros2_control_node` (el `controller_manager`) con el
  `StaubliHardwareInterface` cargado como plugin, y `joint_state_broadcaster` y
  `joint_trajectory_controller` activos a **250 Hz** (`staubli_bringup/config/controllers.yaml:3`).
- **Un cable Ethernet** del PC al **J204** del CS9, con el PC en `192.168.0.1` y el robot en
  `192.168.0.254`.
- **En el robot**: la aplicación VAL3 `ros2_server` corriendo, con su tarea de control a
  **4 ms = 250 Hz** (`val3/userapp/ros2_server/start.pgx:36`).
- **Dos sockets UDP**: control (puerto 11000) y diagnóstico (puerto 11001).

Y una coincidencia que es el requisito de diseño de todo el sistema: **las dos frecuencias son
iguales**, 250 Hz a ambos lados. Por eso el protocolo lleva un campo `control_sequence_delay` que
mide el retardo **en número de ciclos**, y no en milisegundos: si cambia la frecuencia, el
diagnóstico sigue teniendo sentido.

### 6.2 Ida: de una aplicación ROS 2 al robot

```
1.  move_group (o staubli_trajectory, o staubli_gui)
        decide adónde va el robot
        |
        |  ACTION  /joint_trajectory_controller/follow_joint_trajectory
        |          (control_msgs/action/FollowJointTrajectory)
        v
2.  joint_trajectory_controller
        interpola la trayectoria en el tiempo y publica
        un valor objetivo por articulación y por ciclo
        |
        |  escribe en la command_interface  <joint_i>/position
        v
3.  controller_manager  (250 Hz)
        llama a hardware->write(...) con los valores actuales
        |
        v
4.  StaubliHardwareInterface::write()
        - comprueba que no haya NaN
        - convierte radianes -> GRADOS        (rad2deg)
        - fija command_type = JOINT_POSITION
        - pone el número de secuencia recibido del robot
        - pone controller_period = 4 ms
        |
        |  RobotDriver::send_robot_command()
        v
5.  serialización
        FrameHeader (8 B) + payload (47 B)  =  55 bytes
        double -> float32, little endian
        |
        |  socket UDP, puerto local 11000 -> 192.168.0.254:11000
        v
═══════════════════════════  CABLE ETHERNET  ═══════════════════════════
        |
        v
6.  UdpSocket\control  (controlTask, 250 Hz)
        readData -> decodeHeader -> decodeRobotCommandMsg
        |
        |  valida el modo pedido (JOINT_POSITION) y que el robot
        |  permita el cambio de modo
        v
7.  updateJointPositionControl
        llama a la rutina de movimiento de VAL3 del CS9
        (movejSync o equivalente)
        |
        v
8.  El robot se mueve
```

**Puntos que se entienden mejor con el diagrama delante:**

- **El paso 2 a 3 es el corazón de `ros2_control`.** El controlador no habla con el robot: escribe
  en una *command interface*, que no es más que una posición en memoria compartida. El
  `controller_manager` la lee 250 veces por segundo y se la pasa al hardware. Esa indirección es la
  que permite que el mismo controlador funcione con el robot real, con Gazebo o con el mock, sin
  cambiar una línea.
- **El paso 4 introduce las unidades.** ROS 2 trabaja en **radianes**; el bus de Staubli habla en
  **grados**. La conversión (`rad2deg`, `staubli_hardware_interface.cpp:452`) ocurre dentro del
  hardware interface, y por eso nadie más del sistema tiene que saber de este detalle.
- **El paso 5 es donde el `double` se convierte en `float`.** `serialization.hpp:80-87` lo hace
  explícitamente ("serialize as float to save space"). El mensaje de comando ocupa **55 bytes** en
  total: 8 de cabecera y 47 de payload.
- **Los pasos 6-8 son el otro 50 % del proyecto**, y están escritos en un lenguaje que no es
  ROS 2. La `controlTask` de VAL3 es una tarea de tiempo real de 4 ms que hace su propio bucle
  `readData` / `sendData` y aplica sus propias condiciones de seguridad.

### 6.3 Vuelta: del robot a ROS 2

```
1.  El robot, a 250 Hz
        posiciones, velocidades, par estimado, estado de seguridad
        |
        |  updateState.pgx  ->  encodeRobotStateMsg
        v
2.  RobotStateMessage
        8 B cabecera + 124 B payload  =  132 bytes
        (dentro: sequence_delay, control_state, safety_status,
         operation_mode, status_flags, valid_fields_flags,
         6 posiciones, 6 velocidades, 6 pares, 4 valores de par,
         16 entradas digitales, 4 entradas analógicas)
        |
        |  UdpSocket\control  ->  192.168.0.1:11000
        v
═══════════════════════════  CABLE ETHERNET  ═══════════════════════════
        |
        v
3.  UDPSocket (Boost.Asio) -> hilo de recepción -> RealtimeTools::RealtimeThreadSafeBox
        (el hilo de recepción nunca bloquea al hilo de control)
        |
        v
4.  RobotDriver::read_robot_state()
        - valida message_type == ROBOT_STATE
        - RECHAZA el mensaje si es antiguo  (time_since_received >= 100 ms)
        - si no, lo acepta y recuerda su número de secuencia
        |
        v
5.  StaubliHardwareInterface::read() -> update_state()
        - grados -> radianes                    (deg2rad)
        - desempaqueta los bitfields en 10 estados de supervisión
        - rellena las entradas digitales y analógicas
        |
        |  escribe en las state_interfaces
        v
6.  controller_manager
        |
        +--> joint_state_broadcaster  ->  /joint_states  ->  RViz, MoveIt, GUI, servo
        |
        +--> (todos los demás consumidores del estado)
        |
        +--> robot_state_publisher  ->  /tf  ->  RViz dibuja el robot en su pose real
```

**Tres ideas que solo se ven mirando el diagrama:**

- **El estado va por el mismo socket que el comando, en el mismo puerto (11000).** No hay un canal
  de retorno: el robot envía su estado periódicamente por el mismo socket por el que recibe. Esto
  es lo que hace que el mecanismo sea robusto: si el PC pierde un paquete de estado, el siguiente
  4 ms después lo sustituye. Es la razón de que UDP sea una elección razonable aquí, y no un atajo.
- **El rechazo de mensajes antiguos es crítico.** `RobotDriver` no acepta un mensaje si ha pasado
  más de 100 ms (`state_staleness_timeout_`). Sin esa comprobación, si se cortara el cable, el
  driver seguiría "viendo" el último estado conocido y publicaría posiciones inventadas. Con ella,
  el sistema se da cuenta de que está ciego. Es una decisión de seguridad, no de eficiencia.
- **El hilo de recepción es independiente del hilo de control.** Los sockets de `staubli_robot_driver`
  usan `realtime_tools::RealtimeThreadSafeBox` para pasar los mensajes de un hilo a otro sin
  bloqueos ni asignaciones dinámicas. Sin eso, un paquete que llegara en el instante equivocado
  podría hacer que el hilo de 4 ms perdiera su plazo.

### 6.4 El número de secuencia: el reloj del sistema

Un detalle que atraviesa las tres capas y que conviene entender por separado, porque es lo que
permite que el robot sepa si el PC está vivo.

- El robot envía su estado con un **número de secuencia** que se incrementa en cada envío.
- El PC lo **devuelve** en el siguiente comando: es la respuesta a "te he oído".
- El robot mide la diferencia entre "el número que yo envié" y "el número que tú me has devuelto".
  Esa diferencia es el `control_sequence_delay`, que ROS 2 expone como
  `supervision/control_sequence_delay`.
- **Si la diferencia crece, el robot se para.** En VAL3 (`controlTask.pgx:158-162`): si el robot se
  está moviendo y el retardo supera **5 ciclos** (~20 ms), pasa a `STOP`. Si está quieto, el umbral
  sube a 25 ciclos (~100 ms). Y un retardo negativo también es un error.

Es decir: **la seguridad del robot real no depende de que el PC le diga que pare, sino de que el PC
deje de hablar**. Ese diseño es la diferencia entre un driver de investigación y uno que se puede
usar en una célula. Y es, también, la razón por la que la documentación insiste tanto en el cable
punto a punto y en el kernel de tiempo real (§4.8.4).

### 6.5 La máquina de estados del modo de comando

Un último detalle de la cadena, que explica un error muy frecuente:

```
STOP ──► JOINT_POSITION ──► STOP ──► JOINT_VELOCITY (rechazado por el driver)
```

- `prepare_command_mode_switch` (`staubli_hardware_interface.cpp:517-578`) **rechaza** los cambios a
  velocidad (`:552-555`) y a par (`:556-559`), porque no están implementados.
- La `controlTask` de VAL3 exige **pasar siempre por `STOP`** para cambiar de un modo de movimiento
  a otro (`controlTask.pgx:194-199`), y además no permite salir de `STOP` si el robot sigue en
  movimiento (`:201-206`).

Por eso el log de ejemplo de `get_robot_logs.rst` contiene
`"transitioning from command modes requires a stop"`: es la VAL3 diciendo que alguien intentó
un salto de modo que no está permitido. Y por eso `troubleshooting.rst` remite (con un enlace roto,
§4.8.4) a una página sobre modos de control que todavía no existe.

### 6.6 Diagnóstico rápido de los dos errores más frecuentes

| Síntoma en el log del robot | Qué significa realmente | Dónde mirar |
|---|---|---|
| `command sequence delay too high (5)` | El PC dejó de mandar órdenes durante ~20 ms | Enlace Ethernet (cable dedicado), carga de CPU, kernel RT |
| `transitioning from command modes requires a stop` | Se pidió un cambio de modo saltándose `STOP` | Qué controlador está activo y en qué modo |
| Desconexión a las ~2 horas | Addons de Staubli en modo demo | Licencia de `alter` y `advCtrlFunctions` |
| `Failed to connect to robot` (en el PC) | No llega un mensaje de estado válido en 5 s | Aplicación VAL3 cargada, IP, puerto J204, sockets del pendil |

Toda esta tabla está en `troubleshooting.rst` y en `get_robot_logs.rst`. Que esté documentada en el
meta-paquete y no en el driver dice mucho sobre cuál de los dos paquetes es la fuente de verdad
para un usuario.

---

## 7. Cómo se genera esta documentación (reproducibilidad)

Para quien quiera ver la documentación en su máquina, el procedimiento completo (de `README.md` y de
`.github/workflows/documentation.yml`):

```bash
# 1. Herramientas de sistema
sudo apt install doxygen doxygen-gui doxygen-doc python3-sphinx

# 2. Extensiones de Python, en un entorno virtual para no tocar el sistema
cd staubli_driver_ros2/staubli_driver_ros2
python3 -m venv .venv
touch .venv/COLCON_IGNORE          # <- imprescindible, ver §3.5
source .venv/bin/activate
pip install -r requirements.txt

# 3. Compilar (esto lanza Doxygen automáticamente, ver §4.6.1)
sphinx-build -b html sphinx sphinx/_build

# 4. Abrir
firefox sphinx/_build/index.html
```

Y para el PDF:

```bash
sphinx-build -E -a -b latex sphinx pdf/latex
pdflatex -output-directory=pdf/latex pdf/latex/staubli_driver_ros2.tex
firefox pdf/latex/staubli_driver_ros2.pdf
```

**Lo que hace la CI**, exactamente lo mismo pero sin `venv`: instala las herramientas con `apt`,
hace `pip install -r requirements.txt`, ejecuta `sphinx-build -b html sphinx sphinx/_build`, y sube
`sphinx/_build/` como artefacto de GitHub Pages, que se publica en
`https://icube-robotics.github.io/staubli_driver_ros2/`. El workflow se dispara en cada `push` a
`main` y también manualmente.

---

## 8. Observaciones verificadas sobre el estado del paquete

Resumen de lo que se ha comprobado leyendo los ficheros. No son errores que impidan usar el
proyecto; son cosas que conviene conocer, y la mayoría son restos de la evolución.

**Sobre el estado de la documentación**

1. `demo_applications/diagnostics.rst` y `demo_applications/gpios.rst` existen, tienen etiqueta, y
   **no están en ningún `toctree`**: no se publican. Ambos dicen "Coming soon :)".
2. `troubleshooting.rst:94` remite a `:ref:`control_modes``, **etiqueta que no existe** en ninguna
   parte de la documentación. Enlace roto.
3. `developer_guide/API.rst` apunta a `../html/index.html`, que no corresponde a la salida estándar
   de Sphinx.
4. `Makefile:17-18` limpia `_doxygen/` y `api/`, carpetas que **no existen** en esta distribución.
5. `conf.py:93` declara `templates_path = ["_templates"]` y la carpeta **no existe**.
6. `conf.py:84-90` define `get_package()` y **nunca se usa**.
7. `requirements.txt` instala cuatro paquetes que el sitio actual no necesita
   (`sphinx-book-theme`, `sphinx-autobuild`, `sphinxcontrib-video`, `sphinxcontrib-mermaid`).
8. El texto de seguridad está **duplicado en cinco sitios** con ligeras variaciones.

**Sobre el comportamiento**

9. `sphinx/conf.py:33` invoca Doxygen con `subprocess.call` **sin comprobar el resultado**: si
   Doxygen no está instalado, la build continúa y solo falta la referencia de API, sin ningún
   aviso.
10. `CMakeLists.txt:20` usa `${_pip_args}` donde debería usar `${pip_args}` (§4.2): la instalación
    de las herramientas de documentación nunca usa `--user`.
11. El `Doxyfile` y `conf.py` apuntan a directorios de salida distintos para los XML
    (`doxygen/_build/xml/` frente a `sphinx/_build/xml/`, §4.5.1). No he podido verificar la
    compilación porque ni Sphinx ni Doxygen están instalados en este workspace.
12. La acoplamiento con `staubli_robot_driver` es **una línea de texto** en el `Doxyfile`
    (§5.4). Si cambia la disposición de las carpetas, la referencia de API se generará vacía **sin
    ningún error**.

**Sobre la relación con el driver**

13. El `Doxyfile` lee **solo las cabeceras** (`include/`), no los `.cpp`. Es correcto, pero
    significa que la referencia no incluye implementaciones, solo declaraciones.
14. El `package.xml` declara `doxygen` y `python3-sphinx` como `build_depend`, pero **no**
    `graphviz`, que es necesario para los diagramas `dot` y lo instala explícitamente el workflow
    de CI (`.github/workflows/documentation.yml`). Sin él, los diagramas de clases y de herencia
    no se generan, y de nuevo sin error.
15. El `package.xml` no declara `breathe` ni las extensiones de `requirements.txt`, porque no son
    paquetes de `rosdep` sino de `pip`. Es la decisión correcta, pero significa que
    `rosdep install` **no** deja la documentación lista para compilar: hay que hacer el paso de
    `pip` aparte.

---

## 9. Resumen final

`staubli_driver_ros2` es un **meta-paquete con un encargo doble**: declarar las cuatro dependencias
que hacen falta para tener un driver de Staubli funcional, y contener toda la documentación del
proyecto. No compila código, no configura nada y no habla con el robot. Su `CMakeLists.txt`, con
`project(staubli_driver_ros2 NONE)`, es la forma más literal de decir "aquí no hay nada que
compilar".

Lo que sí hace, y que es más de lo que parece al principio, es **conocer todo el proyecto** sin
pertenecer a él. Su documentación es el único sitio donde se explican a la vez las dos mitades del
sistema: el cliente en ROS 2 y el servidor VAL3 dentro del CS9. Alguien que solo lea
`staubli_robot_driver` entiende el cliente; alguien que solo lea la aplicación VAL3 entiende el
servidor; **este paquete es el que permite entender el cable que los une**.

Las cinco ideas que conviene llevarse de este paquete:

- **Un meta-paquete existe para que el usuario no tenga que saber qué falta.** Las cuatro
  dependencias de `package.xml` son exactamente la lista mínima de un driver funcional; los tres
  paquetes experimentales quedan fuera a propósito.
- **La documentación es infraestructura, y este paquete es el contenedor de esa infraestructura.**
  El mismo `Doxyfile` que genera la referencia de API es el que obliga a que el driver tenga
  cabeceras documentadas.
- **La relación con `staubli_robot_driver` es de lectura, no de build.** Una línea de ruta en el
  `Doxyfile` es todo lo que une ambos paquetes, y esa fragilidad conviene conocerla.
- **El proyecto tiene dos mitades muy distintas que hay que aprender por separado:** la de ROS 2
  (plugins, `ros2_control`, controladores, acciones) y la del CS9 (VAL3, sockets, tareas de 4 ms,
  addons con licencia). La documentación de este paquete es, casi sin querer, la introducción a las
  dos.
- **La seguridad no está en el código, está en el protocolo.** Que el robot se pare cuando el PC
  deja de hablar (§6.4), y que las transiciones de modo tengan que pasar por `STOP` (§6.5), explica
  por qué el Troubleshooting del meta-paquete es tan concreto: no es theory, es el registro de lo
  que pasa cuando algo real falla.
