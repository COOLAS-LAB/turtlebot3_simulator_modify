# turtlebot3 仿真环境
# 0. 声明
- 项目来源 https://github.com/ROBOTIS-GIT/turtlebot3

# 1. 编译
```shell
mkdir -p turtlebot3_ws/src
cd turtlebot3/src
git clone https://github.com/yonggaogit/turtlebot3_simulator_modify.git
cd ..
catkin_make
```

# 2. 运行
## 2.1 指定机型

turtlebot3 一共提供三种仿真机型（burger, waffle, waffle_pi），在每次 roslaunch 或 rosrun 前需要通过 `export TURTLEBOT3_MODEL=burger`指定机型，建议将其写入`~/.bshrc` 文件，`sudo gedit ~/.bashrc` 后，将 `export TURTLEBOT3_MODEL=burger` 添加至文件尾。避免每次都需要指定，且当前库已经修改 burger 模型 urdf 文件在上面添加了 kinetic 深度摄像头。

## 2.2 单机手动控制
```shell
# turtlebot3_ws 目录下新建 terminal
roslaunch turtlebot3_gazebo single_turtlebot3.launch

# turtlebot3_ws 目录下新建 terminal
roslaunch turtlebot3_teleop single_turtlebot3_teleop_key.launch
# w 前进、x 后退、a 左转、d 右转、s 停止
```
通过修改 `single_turtlebot3.launch` 中 `<arg name="world_name" value="$(find turtlebot3_gazebo)/worlds/turtlebot3_house.world"/>`  的 `turtlebot3_house.world` 为 `turtlebot3_gazebo/worlds` 文件夹下的其它 world 后缀文件可以切换不同的场景。

## 2.3 多机手动控制
```shell
# turtlebot3_ws 目录下新建 terminal
roslaunch turtlebot3_gazebo multi_turtlebot3.launch

# turtlebot3_ws 目录下新建 terminal
roslaunch turtlebot3_teleop multi_turtlebot3_teleop_key.launch
# turtlebot3_0: w 前进、x 后退、a 左转、d 右转、s 停止
# turtlebot3_1: u 前进、m 后退、h 左转、k 右转、j 停止
# turtlebot3_2: 8 前进、2 后退、4 左转、6 右转、5 停止
```
通过修改 `single_turtlebot3.launch` 中 `<arg name="world_name" value="$(find turtlebot3_gazebo)/worlds/turtlebot3_house.world"/>`  的 `turtlebot3_house.world` 为 `turtlebot3_gazebo/worlds` 文件夹下的其它 world 后缀文件可以切换不同的场景。

## 2.4 UWB
- to be continue

## 2.5 单机视觉 SLAM
- to be continue
- 说明： turtlebot3_slam package 有提供激光 SLAM 示例，具体参考官方文档，注意由于 burger 模型被修改成深度摄像头传感器，跑激光 SLAM 需用另外两个机型

## 2.6 单机规划
- to be continue
- 说明： turtlebot3_navigation package 有提供规划示例，但并非实时建图规划，具体可参考官方文档，注意由于 burger 模型已被修改成深度摄像头传感器，跑规划需用另外两个机型

## 2.7 多机SLAM
- to be continue

