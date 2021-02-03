# social_nav2
social_nav2 es un framework para la representación de personas en el stack de navegación de ROS2, Nav2. Está basado en la teoria de la proxemia [ref Hall], 
que estudia cómo los humanos se posicionan en es espacio respecto a los demás. Esta teoría define zonas, las zonas de proxemia, de diferente tamaño según 
el contexto, la edad, la actividad que se esté realizando, nuestra relación con el resto de humanos, la cultura, etc.
Poder respresentar estás zonas en el mapa del robot resulta muy útil si queremos solucinar problemas de navegación en entornos reales con humanos, 
ya que permite representar de forma distinta a un humano de un obstáculo. Este es un proyecto de software libre y en pleno desarrollo, 
por lo que se agradece cualquier contribución.

![demo](https://github.com/jginesclavero/social_nav2/blob/master/doc/demo.gif?raw=true)

## Cita
Si usas social_layer, construyes tus aplicaciones usando este framework como base, o coges ideas de nuestro trabajo, por favor citanos en tus publicaciones!
 - J. Clavero, F. Martin, FJ. Rodriguez, JM. Hernandez, V.Matellan 
 [**Defining Adaptive Proxemic Zones for Activity-aware Navigation**](https://arxiv.org/abs/2009.04770).Workshop of Physical Agents (WAF), 2020.
 
 ```bibtex
 @inproceedings{clavero2020defining,
  title={Defining Adaptive Proxemic Zones for Activity-Aware Navigation},
  author={Clavero, Jonatan Gin{\'e}s and Rico, Francisco Mart{\'\i}n and Rodr{\'\i}guez-Lera, Francisco J and Hern{\'a}ndez, Jos{\'e} Miguel Guerrero and Olivera,   Vicente Matell{\'a}n},
  booktitle={Workshop of Physical Agents},
  pages={3-17},
  year={2020},
  organization={Springer, Cham}
}

```
## Diseño
Este framework ha sido implementado en forma de plugins de Nav2, por lo que se puede integrar de una forma casi inmediata.
![stack](https://github.com/jginesclavero/social_nav2/blob/master/doc/ros_navigation_layers_2.png)

### Input
Para poder realizar la representación de las personas es necesario alimentar al sistema con su posición y orientación. Para una mejor usabilidad, 
se utiliza el arbol de TFs como entrada, por lo que habrá que publicar la posición y orientación de las personas en este árbol.
Estás TFs pueden ser obtenidas de un sistema de MotionCapture, de un simulador como PedSim o de un sistema de vision abordo del robot.
Normalmente se establecerá un prefijo seguido de un número para tener una TF por cada persona, por ejemplo, "agente_1", "agente_2", "human1", "human2", etc.

![agent_tf](https://github.com/jginesclavero/social_nav2/blob/master/doc/agent_w_tf.png)

### Output
Como ya hemos comentado anteriormente, este framework se ha desarrollado como un plugin de Nav2, por lo que su salida será una contribución
al costmap final que nos produce el sistema de navegación. El framework marcará como ocupadas, con diferentes valores de ocupación, 
las celdas del mapa que correspondan a las zonas de proxemia de la persona.

### People_filter
Por integrarnos en el stack de navegación de ROS2 y para no perder ninguna de sus funcionalidades es necesario limpiar la zona que ocupan las personas en el mapa
para despues establecer su zona de proxemia.

#### Parámetros
Parámetros de configuración. Se especifican en el mismo fichero de parámetros de nav2.
- tf_prefix [string]: Prefijo para identificar las TF correspondientes a las personas
- filter_radius [float]: Radio del filtro, por defecto 0.45m.

### Social_layer
Plugin central del sistema. Es el encargado de crear las zonas de proxemia. Estas zonas de proxemia son totalmente configurables gracias a que son creadas usando
una función [Gausiana Asimétrica](https://ri.cmu.edu/pub_files/2010/5/rk_thesis.pdf). Podemos configurar las zonas usando var_h, var_r y var_s para cada uno de los agentes.

<img src="https://github.com/jginesclavero/social_nav2/blob/master/doc/asymmetric_gaussian.png" width="400">
 
#### Parámetros
Parámetros de configuración. Se especifican en el mismo fichero de parámetros de nav2.
- tf_prefix [string]: Prefijo para identificar las TF correspondientes a las personas
- intimate_z_radius [float]: Radio de la zona intima. Esta zona será no transitable para el robot.
- personal_z_radius [float]: Radio de la zona personal. Radio total de la zona que se representa.
- orientation_info [bool]: Indicamos si queremos que las zonas de proxemia tengan orientación. 
- var_h [float]: 1.2 por defecto.
- var_s [float]: 1.2 por defecto.
- var_r [float]: 1.2 por defecto.

#### Zona de cooperacion
Las zonas de cooperación son opcionales y nos permiten crear zonas libres en las que poder realizar actividades cooperativas con humanos como interaccionar, seguirlos o acompañarlos andando a su lado. 
Debemos definir, por tanto, una acción y configurarla como queramos. A continuación se muestran algunos ejemplos de uso y de como se representan las personas cuando se establecen estas zonas.

HRI      |  Escort action 
:-------------------------:|:-------------------------:
![hri](https://github.com/jginesclavero/social_nav2/blob/master/doc/hri.png) | ![escort](https://github.com/jginesclavero/social_nav2/blob/master/doc/escort.png)

##### Parámetros de la zona de cooperacion. [Ver este ejemplo](https://github.com/jginesclavero/social_nav2/blob/1c1118518a01c9c47358ad0e195b25f11f6e1300/social_nav2_bringup/params/nav2_params.yaml#L202-L208)
- var_h
- var_s
- var_r
- n_activity_zones: Numero de zonas de actividad, entre 0 y 2, para cada acción. 
- activity_zone_phi: Angulo de la zona de actividad.
  
## Uso
Para hacer uso de social_layer, incluimos este repositorio en nuestro workspace y compilamos. Tras esto, añadimos a nuestro fichero de configuración de nav2 la configuración específica del social_layer. [Un ejemplo completo aquí](https://github.com/jginesclavero/social_nav2/blob/master/social_nav2_bringup/params/nav2_params.yaml)

## Demo
### Dependencias.
Para poder simular la posición de las personas es necesario descargar y compilar [PedSim](http://pedsim.silmaril.org/). En este caso, usamos su versión en ROS2. 
```console
  cd ros2_ws/src
  git clone --recursive https://github.com/jginesclavero/pedsim_ros
  cd ..
  colcon build --symlink-install
```
Finalmente, ejecutamos la demo.
```console
  ros2 launch social_nav2_bringup demo_launch.py
```
