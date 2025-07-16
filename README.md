# tfm-moises

Este repositorio contiene varios paquetes ROS&nbsp;2 utilizados durante un Trabajo Fin de Máster.
Los paquetes están pensados para trabajar con el robot TurtleBot3 y se pueden
compilar tanto de forma local como mediante el Dockerfile incluido.

## Contenido del repositorio

- **moi_exp_lite** – implementación del algoritmo explore-lite junto con nodos
  en Python para la detección de semáforos y un controlador de exploración.
- **nav2_abc_pso_planner** y **nav2_abc_pso_planner_exp_area** – plugins de
  planificación global para Nav2 basados en los métodos ABC y PSO.
- **docker/** – Dockerfile y script de entrada que permiten crear un entorno de
  simulación listo para usar.

A continuación se describen los pasos para instalar los paquetes en un
workspace local y las instrucciones para usar la imagen Docker.

## Instalación en local

1. **Instalar dependencias básicas**

   Se recomienda utilizar Ubuntu&nbsp;22.04 con ROS&nbsp;2 Humble ya instalado.
   Además, asegúrate de contar con `git`, `rosdep` y las extensiones de colcon:

   ```bash
   sudo apt install git python3-colcon-common-extensions python3-rosdep
   ```

   Inicializa rosdep si todavía no lo has hecho:

   ```bash
   sudo rosdep init
   rosdep update
   ```

2. **Crear un workspace y clonar el repositorio**

   ```bash
   mkdir -p ~/turtlebot3_ws/src
   cd ~/turtlebot3_ws/src
   git clone <URL_DEL_REPOSITORIO> tfm-moises
   cd ..
   ```

3. **Instalar las dependencias de los paquetes**

   Desde la raíz del workspace ejecuta:

   ```bash
   rosdep install --from-paths src --ignore-src -r -y \
       --skip-keys ament_python --skip-keys turtlebot3_autorace_camera
   ```

   El paquete `moi_exp_lite` dispone además del script `rosdep_install.sh` para
   instalar solo sus dependencias:

   ```bash
   cd src/tfm-moises/moi_exp_lite
   ./rosdep_install.sh
   cd ../../..
   ```

4. **Compilar el workspace**

   ```bash
   colcon build --symlink-install
   ```

5. **Ejecutar la demo de exploración**

   ```bash
   source install/setup.bash
   ros2 launch moi_exp_lite moi_exploration.launch.py
   ```

## Uso con Docker

La carpeta `docker/` proporciona un Dockerfile que compila todos los paquetes
y configura la simulación de TurtleBot3.

1. **Construir la imagen**

   Ejecuta desde la raíz del repositorio:

   ```bash
   docker build -t tfm-moises -f docker/Dockerfile .
   ```

   Puedes cambiar la distribución de ROS pasando el argumento `--build-arg`:

   ```bash
   docker build --build-arg ROS_DISTRO=humble -t tfm-moises .
   ```

2. **Lanzar un contenedor**

   Para visualizar Gazebo y RViz es necesario permitir el acceso a X11 y usar la
   red del host:

   ```bash
   xhost +local:root
   docker run --rm -it --net=host \
       -e DISPLAY=$DISPLAY \
       -v /tmp/.X11-unix:/tmp/.X11-unix \
       tfm-moises
   ```

   La variable `TURTLEBOT3_MODEL` puede fijarse para elegir el modelo (por
   defecto `burger_cam`):

   ```bash
   docker run --rm -it --net=host \
       -e DISPLAY=$DISPLAY \
       -e TURTLEBOT3_MODEL=waffle \
       -v /tmp/.X11-unix:/tmp/.X11-unix \
       tfm-moises
   ```

3. **Ejecutar dentro del contenedor**

   El `entrypoint` carga automáticamente ROS&nbsp;2 y el workspace en
   `/opt/turtlebot3_ws`. Ya dentro del contenedor se pueden lanzar los nodos de
   la misma forma que en una instalación local:

   ```bash
   ros2 launch moi_exp_lite moi_exploration.launch.py
   ```

Con estos pasos puedes trabajar con los paquetes de este repositorio tanto en
un entorno local como dentro de un contenedor Docker.
