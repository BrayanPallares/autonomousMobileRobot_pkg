#   Robot Packages

Conjunto de paquetes de ROS para ejecutar RTAB-Map en el robot omnidireccional de la Universidad Santo Tomás.

#   Configuracion

Para el control del robot se cuenta con dos computadores, uno a bordo del robot y otro remoto. Ambos computadores deben estar conectados a la misma red y se deben configurar sus variables de ROS de la siguiente forma:

Variables de entorno computador abordo:
``` bash
export ROS_MASTER_URI=http://<ROBOT_IP>:11311
export ROS_IP=<ROBOT_IP>
```

Variables de entorno computador remoto:
``` bash
export ROS_MASTER_URI=http://<ROBOT_IP>:11311
export ROS_IP=<PC_IP>
```

# Ejecución

Se necesitan que se haya configurado el *workspace* (Se puede agregar al `~/.bashrc`), tanto en el computador como en el robot:
``` bash
source ~/ros/create_ws/devel/setup.bash
```

Para iniciar con la ejecucion se debe ejecutar `roscore` en el computador abordo del robot:

``` bash
roscore
```

## Lanzar movimiento manual del robot

En el computador abordo del robot se debe lanzar el driver de movimiento:

``` bash
roslaunch robot_pkg launchRobotJoy.launch
```
Este launch lanza los nodos necesarios para leer un joystick conectado mediante bluetooth, convertir sus botones en una velocidad objetivo "cmd_vel" y enviarla por serial al hardware del robot para que este ejecute sus movimiento.

## Lanzar cámara de profundidad (*Intel Realsense D435*)

En el robot se debe ejecutar la cámara de profundidad:
``` bash
roslaunch camera_pkg camera_d435.launch
```
Es posible ejecutar *RViz* para confirmar el correcto funcionamiento del nodo:

``` bash
roslaunch rviz_pkg create_rviz_view_d435.launch
```
Una vez se confirme el correcto funcionamiento de la cámara se debe cerrar *RViz* para continuar.

## Lanzar cámara de seguimiento (*Intel Realsense T265*) y odometría visual

En el robot lanzar la cámara T265:

``` bash
roslaunch camera_pkg camera_t265.launch
```
Es posible ejecutar *RViz* para confirmar el correcto funcionamiento del nodo:

``` bash
roslaunch rviz_pkg create_rviz_view_t265.launch
```
Una vez se confirme el correcto funcionamiento de la cámara se debe cerrar *RViz* para continuar.

## Lanzar mapeo

Se debe estar ejecutando el paquete del robot y ambas cámaras.

``` bash
roslaunch rtabmap_pkg mapping.launch
```

Debe lanzar Rviz para evidenciar el proceso de mapeo, recuerde que en este modo el mapeo se ejecuta de modo manual ejecutando los movimientos del Joystick.

``` bash
roslaunch create_rviz create_rviz_map.launch
```
## Lanzar localización

Una vez se ha creado un mapa puede lanzar el paquete de localización para ubicar el robot. Recuerde que el mapa es guardado como `~/.ros/rtabmap.db`. Antes de lanzar la localización debe cerrar el mapeo.

``` bash
roslaunch rtabmap_pkg localization.launch
```

Rviz nos permite ver la localización del robot en el mapa creado.

``` bash
roslaunch create_rviz create_rviz_loc.launch
```

## Lanzar Navegación

Para lanzar la navegación debe estar ejecutando la localización en un mapa creado anteriormente.


``` bash
roslaunch navigation_pkg teb_planner.launch
```

Debe lanzar Rviz para ver la navegación y configurar el punto objetivo.

``` bash
roslaunch create_rviz create_rviz_navigation.launch
```
Debe dar click en la parte superior "2D Nav Goal" y seleccionar un punto dentro del mapa. 
