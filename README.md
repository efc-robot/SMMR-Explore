# SMMR-Explore
The released 1.0 Version of SMMR-Explore, the supporting open-sourced software package of the paper "SMMR-Explore: SubMap-based Multi-Robot Exploration System with Multi-robot and Multi-target Potential Field Explorer" submitted to ICRA 2021.

Our video is released at [Youtube(english version)](https://www.youtube.com/watch?v=H1zwRIz8OYs&list=PLBKYimzl6wexgpQCXKijgDOflcCny6dBf&index=1) and [Bilibili(Chinese Version)](https://www.bilibili.com/video/BV1QT4y1F71u). Note that videos of all experiments also get released on Youtube Channel. 

Features:
- Only submaps get shared among robots, reducing the communication overhead.
- place Recognition (PR), Relative pose estimation (RelPose) and map merge methods only based on submaps. 
- Fast Multi-robot Multi-target Potential Field (MMPF) exploration method, increasing exploration efficiency with less travel cost than Rapidly Random Tree (RRT).

# Platform
- Multiple cars with nvidia jetson nano board and Lidar sensor.
- ubuntu 18.04 (bash)
- ROS melodic

# Dependency

## Cartographer
Cartographer is a 2D/3D map-building method.
It provides the submaps' and the tranjectories' information when building the map. 

We slightly modifiyed the original Cartographer to make it applicable to multi-robot SLAM and exploration.

Please refer to [Cartographer-for-SMMR](https://github.com/efc-robot/Cartographer-for-SMMR) to install the modified Cartographer to ```carto_catkin_ws```

and 

```
source /PATH/TO/CARTO_CATKIN_WS/devel_isolated/setup.bash
```



## Pytorch for PointNetVLAD
[official_installation_guide](https://pytorch.org/get-started/locally/)

```
pip install torch torchvision
```


## The Pretrained PointNetVLAD
- gem.pth  NN model for place recognition, should be placed in "src/m-explore/map_merge/include/weights/" 
- model_test.ckpt NN model of 2D PointNetVLAD, should be placed in "src/smmr_explore_wrapper/scripts/"
- (for simulation) gazebo, especially gazebo-ros

Note: please contact author to get model files or download model at [download link](https://pan.baidu.com/s/1NBCEYombBhIqCbfqsiV12w) with password as 058o.

## Turtlebot3 Description and Simulation
(robot model for simulation)

```
sudo apt install ros-melodic-turtlebot3*
sudo apt install ros-melodic-bfl
pip install future
sudo apt install ros-melodic-teb-local-planner
```


# Ready to run Exploration in Gazebo? 

## Installation
download the repo and then run
```
catkin_make
source <repo_path>/devel/setup.bash
```

- Template  
```
{env_size}      = 'small' or 'large'
{number_robots} = 'single' or 'two' or 'three'
{method}        = 'rrt' or 'mmpf'
{suffix}        = 'robot' or 'robots' (be 'robot' when number_robots != 'single')
roslaunch turtlebot3sim {env_size}_env_{number_robots}_{suffix} .launch
roslaunch turtlebot3sim {number_robots}_{suffix} .launch
roslaunch {method} {number_robots}_{method}_node.launch
```

Specifically:
### BaseLine: Rapidly Random Tree (RRT)
- Small Environment -- Single Robot
```
roslaunch turtlebot3sim small_env_single_robot.launch
roslaunch turtlebot3sim single_robot.launch
roslaunch rrt single_rrt_node.launch
```

- Small Environment -- Two Robots
```
roslaunch turtlebot3sim small_env_two_robots.launch 
roslaunch turtlebot3sim two_robots.launch
roslaunch rrt two_rrt_node.launch 
```

- Small Environment -- Three Robots
```
roslaunch turtlebot3sim small_env_three_robots.launch 
roslaunch turtlebot3sim three_robots.launch
roslaunch rrt three_rrt_node.launch 
```

- Large Environment -- Single Robot
```
roslaunch turtlebot3sim Large_env_single_robot.launch
roslaunch turtlebot3sim single_robot.launch
roslaunch rrt single_rrt_node.launch
```

- Large Environment -- Two Robots
```
roslaunch turtlebot3sim Large_env_two_robots.launch 
roslaunch turtlebot3sim two_robots.launch
roslaunch rrt two_rrt_node.launch 
```

- Large Environment -- Three Robots
```
roslaunch turtlebot3sim Large_env_three_robots.launch 
roslaunch turtlebot3sim three_robots.launch
roslaunch rrt three_rrt_node.launch 
```

### Multi-robot Multi-target Potential Field (MMPF)
- Small Environment -- Single Robot
```
roslaunch turtlebot3sim small_env_single_robot.launch
roslaunch turtlebot3sim single_robot.launch
roslaunch mmpf single_mmpf_node.launch
```

- Small Environment -- Two Robots
```
roslaunch turtlebot3sim small_env_two_robots.launch 
roslaunch turtlebot3sim two_robots.launch
roslaunch mmpf two_mmpf_node.launch 
```

- Small Environment -- Three Robots
```
roslaunch turtlebot3sim small_env_three_robots.launch 
roslaunch turtlebot3sim three_robots.launch
roslaunch mmpf three_mmpf_node.launch 
```

- Large Environment -- Single Robot
```
roslaunch turtlebot3sim Large_env_single_robot.launch
roslaunch turtlebot3sim single_robot.launch
roslaunch mmpf single_mmpf_node.launch
```

- Large Environment -- Two Robots
```
roslaunch turtlebot3sim Large_env_two_robots.launch 
roslaunch turtlebot3sim two_robots.launch
roslaunch mmpf two_mmpf_node.launch 
```

- Large Environment -- Three Robots
```
roslaunch turtlebot3sim Large_env_three_robots.launch 
roslaunch turtlebot3sim three_robots.launch
roslaunch mmpf three_mmpf_node.launch 
```
### Start Exploration

For all the cases, choose "Publish Point" button in the rviz and then click anywhere in the map to start the exploration.

# Acknowledge
Greatly thanks to endeavor of all members for the whole SMMR system, Every part is INDISPENSABLE.
- Jincheng Yu:    Propose SubMap-based Multi-Robot (SMMR) Exploration System Idea & hard problems killer & project leader. (yjc16@tsinghua.edu.cn)
- Jianming Tong:  propose decision methods: Multi-robot Multi-target Potential Field (MMPF) && improved RRT as baseline. (jtong45@gatech.edu)
- Zhilin Xu:      Propose submap-based relative pose estimation and map merge algorithm, contribute multi-robot potential field for MMPF. (xuzhl18@mails.tsinghua.edu.cn)
- Yuanfan Xu:     Propose 2D PointNetVLAD for filter wrong submaps to reduce computation overhead of Place Recognition (PR).  (xuyf20@mails.tsinghua.edu.cn)
- Haolin Dong:    Propose Iterative Close Points (ICP) registration methods. (donghl17@mails.tsinghua.edu.cn)
- Tianxiang Yang: Implement elementary navigation of the real car using move_base package. (394644551@qq.com)
- Yu Wang:        Project's Advisor && Lab Leader.  (yu-wang@tsinghua.edu.cn)
