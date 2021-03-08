# Social_nav2 (work in progress, all comments are welcome)
[Documentación disponible en Español](https://github.com/jginesclavero/social_nav2/blob/master/doc/README_es.md)

Social_nav2 is a people representation framework for ROS2 Navigation stack, [NAV2](https://navigation.ros.org/). It is based on the [Hall proxemic theory](http://200.2.15.132/handle/123456789/32281). This theory studies how humans share the space between them defining zones, the proxemic zones. Shape and size of the zone may change with the age, culture, context, activity performed, our relationship with others, etc. This information could be useful in a crowded environment where the robot has to navigate or interact with humans. With this framework, you can represent people in the navigation map differently than an obstacle. This package is an opensource project and it does a number of things to improve. Every comment is welcome!

This work was presented in WAF2020. Here you find the presentation video.

[![](https://raw.githubusercontent.com/jginesclavero/social_nav2/master/doc/social_nav2_miniature.png)](http://www.youtube.com/watch?v=HZ6HeVKQSZU "")


## Citation
If you use social_,nav2, builts your applications on top, or ideas from it please cite this work in your papers!
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

![demo](https://github.com/jginesclavero/social_nav2/blob/master/doc/demo.gif?raw=true)


## Design
Social_nav2 framework has to implemented as a NAV2 plugin, it's easy to use!

![stack](https://github.com/jginesclavero/social_nav2/blob/master/doc/ros_navigation_layers_2.png)

### Input
The framework needs the people pose (position and orientation) to work. To do an easy integration is necessary to feed the framework with a TF from each person.
These TFs could be extracted from a MotionCapture system, a simulator like PedSim, or an onboard vision system.
Usually a prefix followed by a number will be set to have a TF for each person, e.g. "agent_1", "agent_2", "human1", "human2", etc.

![agent_tf](https://github.com/jginesclavero/social_nav2/blob/master/doc/agent_w_tf.png)

### Output
As we mention before, this framework has to develop as a NAV2 plugin. Because of this, the output is a contribution in the final navigation costmap, adding the proxemic zones to the map.

### People_filter
To not lose any functionalities of the NAV2, it is necessary to clean the zone occupied by the people on the map (established by the obstacle layer) and then create their proxemic zone. People_filter uses the people pose to do this cleaning.

#### Parameters
- tf_prefix [string]: Prefix to identify the people TF.
- filter_radius [float]: 0.45m by default.

### Social_layer
Framewrok core. It creates the proxemic zones. These zones are fully customizables because we use an [Assymetric Gaussian funtion](https://ri.cmu.edu/pub_files/2010/5/rk_thesis.pdf). We can configure the zones using var_h, var_r and var_s for each agent.

<img src="https://github.com/jginesclavero/social_nav2/blob/master/doc/asymmetric_gaussian.png" width="400">
 
#### Parameters
- tf_prefix [string]: Prefix to identify the people TF.
- intimate_z_radius [float]: Intimate zone radius. This zone is not walkable by robot.
- personal_z_radius [float]: Personal zone radius. Total radius of the represented zone.
- orientation_info [bool]: We can, or not, use the orientation in our gaussians. 
- var_h [float]: 1.2 by default.
- var_s [float]: 1.2 by default.
- var_r [float]: 1.2 by default.

  
## Usage
To use social_nav2, we have to include this repository in our ROS2 workspace and compile it. After that, we add the social_nav2 parameters to the NAV2 params file. [Params file example](https://github.com/jginesclavero/social_nav2/blob/master/social_nav2_bringup/params/nav2_params.yaml)

## Demo
### Some dependencies.
We need [PedSim](http://pedsim.silmaril.org/) to simulate the people position and their movements.
```console
  cd ros2_ws/src
  git clone --recursive https://github.com/jginesclavero/pedsim_ros
  cd ..
  colcon build --symlink-install
```
Finally, the demo launch.
```console
  ros2 launch social_nav2_bringup demo_launch.py
```
