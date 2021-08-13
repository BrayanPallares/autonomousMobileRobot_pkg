#   Robot Packages

Conjunto de paquetes de ROS para ejecutar RTAB-Map en el robot omnidireccional de la Universidad Santo Tomás.

Los paquetes pueden ser ejecutados en su totalidad en el computador a bordo del robot o utilizar un segundo computador para controlarlo remotamente. Esta documentacion contempla que el software se ejecutara en un computador a bordo [Robot] y un computador remoto [PC], en caso de que se quiera ejecutar todo en el robot, correr tambien el codigo marcado como [PC].

La convencion para los computadores involucrados es de la siguiente forma:

* *Robot*: Computador a bordo
* *PC*: Computador externo

#   Requisitos

Probado con los siguientes computadores a bordo:

* Up Squared - 4GB de RAM - Intel(R) Atom(TM) Processor E3940 @ 1.60GHz

Sistema Operativo:

* Ubuntu 18.04
* Linux Mint 19.3

Versiones de ROS:

* ROS Melodic


Paquetes de ROS:

* rtabmap
* rtabmap-ros

#   Instalacion

[PC][Robot] Cree el workspace y clone el repositorio:

``` bash
mkdir -p ~/ros/autonomousMobileRobot_pkg/src/
cd ~/ros/autonomousMobileRobot_pkg/src
git clone https://github.com/BrayanPallares/autonomousMobileRobot_pkg.git
```

[PC][Robot] Compile el workspace:

``` bash
cd ~/ros/autonomousMobileRobot_pkg/
catkin_make
```



#   Configuracion

Para el control del robot se cuenta con dos computadores, uno a bordo del robot y otro remoto. Ambos computadores deben estar conectados a la misma red y se deben configurar sus variables de ROS de la siguiente forma:

[Robot]Variables de entorno computador abordo:
``` bash
export ROS_MASTER_URI=http://<ROBOT_IP>:11311
export ROS_IP=<ROBOT_IP>
```

[PC]Variables de entorno computador remoto:
``` bash
export ROS_MASTER_URI=http://<ROBOT_IP>:11311
export ROS_IP=<PC_IP>
```

# Ejecución

[Robot][PC] Se necesitan que se haya configurado el *workspace* (Se puede agregar al `~/.bashrc`), tanto en el computador como en el robot:
``` bash
source ~/ros/autonomousMobileRobot_pkg/devel/setup.bash
```

[Robot] Para iniciar con la ejecucion se debe ejecutar `roscore` en el computador abordo del robot:

``` bash
roscore
```

## Lanzar movimiento manual del robot

Se debe conectar un controlador Joystick via bluetooth al computador remoto.

[Robot] Debe ir a la ruta /dev y suministrar permisos de ejecucion al puerto de la tarjeta STM32F4, usualmente identificado como ttyACM1 - ttyACM0.

``` bash
sudo chmod +777 ttyACM1
```

[PC] Lee el contrador Joysctick y genera una velocidad cmd_vel a partir de la informacion de este

``` bash
roslaunch robot_pkg convertJoy2Cmdvel.launch
```

Debe mostrar algo como:
```
...
[ INFO] [1628811972.000221041]: Opened joystick: /dev/input/js0. deadzone_: 0.050000.
('message read = ', header: 
  seq: 1
  stamp: 
    secs: 1628811977
    nsecs: 493257854
  frame_id: "/dev/input/js0"
axes: [0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
buttons: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
('start = ', 0)
('back  = ', 0)
...
```

[Robot] Ejecutar nodo que lee la velocidad cmd_vel y la envia por serial a la tarjeta STM32f *importante: primero se deben suministrar permisos al puerto ttyACM# como se muestra mas arriba*:

``` bash
roslaunch robot_pkg moveRobot.launch
```

Debe mostar algo como:


```
...
Leyendo serial.....
(' odom = ', header: 
  seq: 0
  stamp: 
    secs: 1628811648
    nsecs: 753022909
  frame_id: "odom"
child_frame_id: "base_footprint"
pose: 
  pose: 
    position: 
      x: -0.190343585232
      y: -0.0634478617441
      z: 0.0
    orientation: 
      x: 0.0
      y: 0.0
      z: -0.083277766096
      w: 0.996526373798
  covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
twist: 
  twist: 
    linear: 
      x: -0.00559596609375
      y: -0.00186532203125
      z: 0
    angular: 
      x: 0
      y: 0
      z: -0.00490229180355
  covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
escrito correctamente
...
```


## Lanzar cámara de profundidad (*Intel Realsense D435*)

[Robot] Se lanza el nodo de la cámara de profundidad:
``` bash
roslaunch camera_pkg camera_d435_scan.launch 
```
Debe mostrar algo como:
```
...
[ INFO] [1628812894.445100413]: setupPublishers...
[ INFO] [1628812894.451393585]: Expected frequency for depth = 30.00000
[ INFO] [1628812894.519005636]: Expected frequency for color = 30.00000
[ INFO] [1628812894.557228461]: Expected frequency for aligned_depth_to_color = 30.00000
[ INFO] [1628812894.588334341]: setupStreams...
[ INFO] [1628812894.610891639]: insert Depth to Stereo Module
[ INFO] [1628812894.611013992]: insert Color to RGB Camera
[ INFO] [1628812894.727195461]: SELECTED BASE:Depth, 0
[ INFO] [1628812894.745680884]: RealSense Node Is Up!
...
```

[Robot]Es posible ejecutar *RViz* para confirmar el correcto funcionamiento del nodo:

``` bash
roslaunch rviz_pkg rviz_view_d435.launch 
```

Debe abrirse una pantalla de Rviz donde se muestra la imagen de la camara de profundidad similar a:

![d435_test_BP.png](https://github.com/BrayanPallares/autonomousMobileRobot_pkg/blob/master/img/d435_test_BP.png?raw=true)

Una vez se confirme el correcto funcionamiento de la cámara se debe cerrar *RViz* para continuar.

## Lanzar cámara de seguimiento (*Intel Realsense T265*) y odometría visual

En el robot lanzar la cámara T265:

``` bash
roslaunch camera_pkg camera_t265.launch
```

Debe mostrar algo como:

```
...
[ INFO] [1628814418.984079807]: Done Setting Dynamic reconfig parameters.
[ INFO] [1628814418.984299955]: gyro stream is enabled - fps: 200
[ INFO] [1628814418.984355624]: accel stream is enabled - fps: 62
[ INFO] [1628814418.984388307]: pose stream is enabled - fps: 200
[ INFO] [1628814418.984426386]: setupPublishers...
[ INFO] [1628814418.989096433]: setupStreams...
[ INFO] [1628814418.989206759]: insert Gyro to Tracking Module
[ INFO] [1628814418.989301910]: insert Accel to Tracking Module
[ INFO] [1628814418.989329582]: insert Pose to Tracking Module
[ INFO] [1628814419.023630065]: SELECTED BASE:Pose, 0
[ INFO] [1628814419.026308638]: RealSense Node Is Up!
[ INFO] [1628814419.027322724]: Subscribing to in_odom topic: odom_wheels
...
```


Es posible ejecutar *RViz* para confirmar el correcto funcionamiento del nodo:



``` bash
roslaunch rviz_pkg create_rviz_view_t265.launch
```
Una vez se confirme el correcto funcionamiento de la cámara se debe cerrar *RViz* para continuar.


![t265_test_BP.png](https://github.com/BrayanPallares/autonomousMobileRobot_pkg/blob/master/img/t265_test_BP.png?raw=true)

## Lanzar mapeo

Se debe estar ejecutando el paquete del robot y ambas cámaras.

``` bash
roslaunch rtabmap_pkg map_scan.launch 
```

Debe mostrar algo como:

```
...
[ INFO] [1628814775.626510696]: rtabmap 0.20.7 started...
[ INFO] [1628814776.181489063]: rtabmap (1): Rate=1.00s, Limit=0.000s, RTAB-Map=0.2471s, Maps update=0.0001s pub=0.0001s (local map=1, WM=1)
[ INFO] [1628814776.936888351]: rtabmap (2): Rate=1.00s, Limit=0.000s, RTAB-Map=0.0608s, Maps update=0.0001s pub=0.0000s (local map=1, WM=1)
[ INFO] [1628814777.945338294]: rtabmap (3): Rate=1.00s, Limit=0.000s, RTAB-Map=0.0572s, Maps update=0.0001s pub=0.0000s (local map=1, WM=1)
[ INFO] [1628814778.946412326]: rtabmap (4): Rate=1.00s, Limit=0.000s, RTAB-Map=0.0664s, Maps update=0.0001s pub=0.0000s (local map=1, WM=1)
[ INFO] [1628814779.908611567]: rtabmap (5): Rate=1.00s, Limit=0.000s, RTAB-Map=0.0565s, Maps update=0.0001s pub=0.0000s (local map=1, WM=1)
...
```

Debe lanzar Rviz para evidenciar el proceso de mapeo, recuerde que en este modo el mapeo se ejecuta de modo manual ejecutando los movimientos del Joystick.

``` bash
roslaunch rviz_pkg rviz_map.launch
```

Se empezara a crear el mapa a medida que el robot se mueva, se debe ver algo similar a:

![t265_test_BP.png](https://github.com/BrayanPallares/autonomousMobileRobot_pkg/blob/master/img/t265_test_BP.png?raw=true)

Una vez creado el mapa se puede cerrar el mapeo presionando control + c, esto guardara el mapa en un .db. El mapa completo se debe ver algo asi:

![t265_test_BP.png](https://github.com/BrayanPallares/autonomousMobileRobot_pkg/blob/master/img/t265_test_BP.png?raw=true)

## Lanzar localización

Una vez se ha creado un mapa puede lanzar el paquete de localización para ubicar el robot. Recuerde que el mapa es guardado como `~/.ros/rtabmap.db`. Antes de lanzar la localización debe cerrar el mapeo.

``` bash
roslaunch rtabmap_pkg localizationScan.launch 
```

Debe mostrar algo asi:

```
...
[ INFO] [1628815623.296693297]: rtabmap (806): Rate=1.00s, Limit=0.000s, RTAB-Map=0.1487s, Maps update=0.0001s pub=0.0000s (local map=39, WM=39)
[ INFO] [1628815624.284781279]: rtabmap (807): Rate=1.00s, Limit=0.000s, RTAB-Map=0.1445s, Maps update=0.0001s pub=0.0000s (local map=39, WM=39)
[ INFO] [1628815625.269370982]: rtabmap (808): Rate=1.00s, Limit=0.000s, RTAB-Map=0.1331s, Maps update=0.0001s pub=0.0000s (local map=39, WM=39)
[ INFO] [1628815626.286564063]: rtabmap (809): Rate=1.00s, Limit=0.000s, RTAB-Map=0.1305s, Maps update=0.0001s pub=0.0000s (local map=39, WM=39)
[ INFO] [1628815627.293476715]: rtabmap (810): Rate=1.00s, Limit=0.000s, RTAB-Map=0.1449s, Maps update=0.0001s pub=0.0000s (local map=39, WM=39)
...
```

Rviz nos permite ver la localización del robot en el mapa creado.

``` bash
roslaunch rviz_pkg rviz_loc.launch 
```

Se debe abrir una ventana con una vista similar a est:

![t265_test_BP.png](https://github.com/BrayanPallares/autonomousMobileRobot_pkg/blob/master/img/t265_test_BP.png?raw=true)

## Lanzar Navegación

Para lanzar la navegación debe estar ejecutando la localización en un mapa creado anteriormente.


``` bash
roslaunch navigation_pkg move_base_teb.launch
```

Debe mostrar algo como:

```
...
[ INFO] [1628815803.276952350]: odom received!
[ INFO] [1628815803.392717985]: Resizing costmap to 49 X 71 at 0.050000 m/pix
[ INFO] [1628815804.378840655]: Resizing costmap to 49 X 70 at 0.050000 m/pix
[ INFO] [1628815805.403610445]: Resizing costmap to 50 X 70 at 0.050000 m/pix
[ INFO] [1628815806.426767347]: Resizing costmap to 49 X 70 at 0.050000 m/pix
[ INFO] [1628815807.411396310]: Resizing costmap to 47 X 70 at 0.050000 m/pix
...
```

Debe lanzar Rviz para ver la navegación y configurar el punto objetivo.

``` bash
roslaunch rviz_pkg rviz_navigation.launch 
```
Debe dar click en la parte superior "2D Nav Goal" y seleccionar un punto dentro del mapa.

El robot en navegacion se ve en Rviz similar a:

![t265_test_BP.png](https://github.com/BrayanPallares/autonomousMobileRobot_pkg/blob/master/img/t265_test_BP.png?raw=true)
