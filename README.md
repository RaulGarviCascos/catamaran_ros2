# ROS2 Catamarán – Control de velocidad y motores brushless

Este repositorio contiene un *workspace* de ROS 2 con:

- **my_interfaces**: paquete de mensajes personalizados (`Velocity.msg`).
- **cpp_pkg**: nodos en C++:
  - `control_velocity`: publica velocidades (izquierda/derecha) en el tópico `velocity`.
  - `motor_brushless_node`: corre en la Raspberry Pi y controla los motores brushless leyendo el tópico `velocity`.
  - Otros nodos de prueba (listeners, ejemplos…).
- **py_pkg**: nodos de ejemplo en Python.

El objetivo principal es controlar motores brushless desde un PC (publicador) hacia una Raspberry Pi (suscriptor y control low-level).

---

## 1. Requisitos

### 1.1. ROS 2

- Distro recomendada: **ROS 2 Humble** en **PC** y **Raspberry Pi**.
- Sistema: Ubuntu 22.04 .

Instalación de ROS 2 Humble ( ver docs oficiales para detalles): [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

### 1.2 Herramientas de compilación:
```
sudo apt update
sudo apt install -y \
  build-essential \
  cmake \
  python3-colcon-common-extensions \
  git
```
Para la raspberry haría falta también:
```
sudo apt install libwiringpi-dev
sudo chmod u+s /usr/bin/gpio
```
## 2. Clonar este repo.
```
git clone https://github.com/RaulGarviCascos/catamaran_ros2
cd catamaran_ros2
```
## 3. Configuración del entorno.

En el fichero ```~/.bashrc``` añadir:
```
# ROS 2 Humble
source /opt/ros/humble/setup.bash
# Workspace del catamarán
source ~/Documents/catamaran_ros2/install/setup.bash 2>/dev/null || true
export ROS_DOMAIN_ID=13
```
## 4. Compilación del workspace.

```
cd ~/Documents/catamaran_ros2
colcon build
```

Después de compilar:
```
source install/setup.bash
```
## 5. Ejecutar nodos de prueba.

En una terminal.

```
cd ~/Documents/catamaran_ros2
source install/setup.bash
ros2 run my_cpp_pkg listener_velocidad
```
En otra terminal.

```
cd ~/Documents/catamaran_ros2
source install/setup.bash
ros2 run my_cpp_pkg control_velocidad
```
