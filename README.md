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

turtlebot3 一共提供三种仿真机型（burger, waffle, waffle_pi），在每次 roslaunch 或 rosrun 前需要通过 `export TURTLEBOT3_MODEL=burger`指定机型，建议将其写入`~/.bshrc` 文件，`sudo gedit ~/.bashrc` 后，将 `export TURTLEBOT3_MODEL=burger` 添加至文件尾，避免每次都需要指定，注意添加完后需要重开 terminal 或在当前 terminal 输入 `source ~/.bashrc`，且当前库已经修改 burger 模型 urdf 文件在上面添加了 kinetic 深度摄像头。

## 2.2 单机手动控制
- 运行
```shell
# turtlebot3_ws 目录下新建 terminal
source devel/setup.bash
roslaunch turtlebot3_gazebo single_turtlebot3.launch

# turtlebot3_ws 目录下新建 terminal
source devel/setup.bash
roslaunch turtlebot3_teleop single_turtlebot3_teleop_key.launch
# w 前进、x 后退、a 左转、d 右转、s 停止
```
- 参数说明

通过修改 `single_turtlebot3.launch` 中 `<arg name="world_name" value="$(find turtlebot3_gazebo)/worlds/turtlebot3_house.world"/>`  的 `turtlebot3_house.world` 为 `turtlebot3_gazebo/worlds` 文件夹下的其它 world 后缀文件可以切换不同的场景。

通过修改 `single_turtlebot3.launch` 中
```shell
<arg name="x_pos" default="0.0"/>
<arg name="y_pos" default="0.0"/>
<arg name="z_pos" default="0.0"/>
<arg name="yaw" default="0.0"/>
```
修改无人车的初始位姿（在 world 坐标系下的）

- 常用话题说明

/camera/depth/image_raw 为深度图，可通过 rqt_image_view 或 rviz 查看，/camera/depth/points 为点云，可通过 rviz 查看

/camera/rgb/image_raw 为 rgb 图，可通过 rqt_image_view 或 rviz 查看

/cmd_vel 为速度控制话题，可以通过 `rostopic pub` 指令发布以控制机器人，实际通过 `single_turtlebot3_teleop_key` 脚本键盘控制更方便，涉及到规划算法时需要用到该话题

/odom 为里程计的 ground_truth 值，是以 world 坐标系为基准的


## 2.3 多机手动控制
- 运行
```shell
# turtlebot3_ws 目录下新建 terminal
source devel/setup.bash
roslaunch turtlebot3_gazebo multi_turtlebot3.launch

# turtlebot3_ws 目录下新建 terminal
source devel/setup.bash
roslaunch turtlebot3_teleop multi_turtlebot3_teleop_key.launch
# turtlebot3_0: w 前进、x 后退、a 左转、d 右转、s 停止
# turtlebot3_1: u 前进、m 后退、h 左转、k 右转、j 停止
# turtlebot3_2: 8 前进、2 后退、4 左转、6 右转、5 停止
```
- 参数说明

通过修改 `single_turtlebot3.launch` 中 `<arg name="world_name" value="$(find turtlebot3_gazebo)/worlds/turtlebot3_house.world"/>`  的 `turtlebot3_house.world` 为 `turtlebot3_gazebo/worlds` 文件夹下的其它 world 后缀文件可以切换不同的场景。

通过修改 `single_turtlebot3.launch` 中
```shell
<arg name="first_tb3_x_pos" default="0.0"/>
<arg name="first_tb3_y_pos" default="0.0"/>
<arg name="first_tb3_z_pos" default="0.0"/>
<arg name="first_tb3_yaw"   default="0.0"/>

<arg name="second_tb3_x_pos" default=" 7.0"/>
<arg name="second_tb3_y_pos" default="-1.0"/>
<arg name="second_tb3_z_pos" default=" 0.0"/>
<arg name="second_tb3_yaw"   default=" 1.57"/>

<arg name="third_tb3_x_pos" default=" 0.5"/>
<arg name="third_tb3_y_pos" default=" 3.0"/>
<arg name="third_tb3_z_pos" default=" 0.0"/>
<arg name="third_tb3_yaw"   default=" 0.0"/>
```
来调整三个无人车的初始位姿

- 常用话题说明

同单机一样，只是对三个机器人加上了前缀 `/tb3_0/`、`/tb3_1/`、`/tb3_2/` 加以区分

## 2.4 UWB
只在多机下使用
通过 `/tb3_x/odom` 获取相对于 world 坐标系的位姿，通过该位姿将无人车上的虚拟 UWB 转换到世界坐标系，基于 UWB 的世界坐标系计算相互之间的距离并加上一个的高斯噪声作为仿真。

输出话题为 `/uwb/dis_ra` 与 `/uwb/dis_rb` 均为 `std_msgs/Float64MultiArray` 类型，即数组，表示的是 refrence 无人车（/tb3_0/）与 a 无人车（/tb3_1/） 和 b 无人车（/tb3_2/）之间 UWB 对的距离。

默认 /tb3_0/ 无人车上安装 3 个 UWB，其余两个车上安装两个 UWB。

通过修改 `/turtlebot3_simulations/turtlebot3_gazebo/launch/multi_turtlebot3.launch` 中的

```shell
<arg name="noise_mean" default=" 0.0"/>
<arg name="noise_stddev" default=" 0.5"/>
<arg name="uwb_r1_x" default=" 1.0"/>
<arg name="uwb_r1_y" default=" 2.0"/>
<arg name="uwb_r2_x" default=" 3.0"/>
<arg name="uwb_r2_y" default=" 4.0"/>
<arg name="uwb_r3_x" default=" 0.0"/>
<arg name="uwb_r3_y" default=" 5.0"/>
<arg name="uwb_a0_x" default=" 0.0"/>
<arg name="uwb_a0_y" default=" 0.0"/>
<arg name="uwb_a1_x" default=" 1.0"/>
<arg name="uwb_a1_y" default=" 1.0"/>
<arg name="uwb_b0_x" default=" 0.0"/>
<arg name="uwb_b0_y" default=" 0.0"/>
<arg name="uwb_b1_x" default=" 1.0"/>
<arg name="uwb_b1_y" default=" 0.0"/>
```

|参数|作用|
|-|-|
|noise_mean|测距噪声均值|
|noise_stddev|测距噪声方差|
|uwb_r1_x|reference 车上的第一个 UWB 的 x 坐标（相对于车的机体坐标系）|
|uwb_r1_y|reference 车上的第一个 UWB 的 y 坐标（相对于车的机体坐标系）|
|uwb_r2_x|reference 车上的第二个 UWB 的 x 坐标（相对于车的机体坐标系）|
|uwb_r2_y|reference 车上的第二个 UWB 的 y 坐标（相对于车的机体坐标系）|
|uwb_r3_x|reference 车上的第三个 UWB 的 x 坐标（相对于车的机体坐标系）|
|uwb_r3_y|reference 车上的第三个 UWB 的 y 坐标（相对于车的机体坐标系）|
|uwb_a0_x|a 车上的第一个 UWB 的 x 坐标（相对于车的机体坐标系）|
|uwb_a0_y|a 车上的第一个 UWB 的 y 坐标（相对于车的机体坐标系）|
|uwb_a1_x|a 车上的第二个 UWB 的 x 坐标（相对于车的机体坐标系）|
|uwb_a1_y|a 车上的第二个 UWB 的 y 坐标（相对于车的机体坐标系）|
|uwb_b0_x|b 车上的第一个 UWB 的 x 坐标（相对于车的机体坐标系）|
|uwb_v0_y|b 车上的第一个 UWB 的 y 坐标（相对于车的机体坐标系）|
|uwb_b1_x|b 车上的第二个 UWB 的 x 坐标（相对于车的机体坐标系）|
|uwb_b1_y|b 车上的第二个 UWB 的 y 坐标（相对于车的机体坐标系）|

- to be continue

事实上， UWB 之间的测距是随着距离的增加越来越不准的，且在穿越障碍物时其噪声更大，后期加入这些考虑以实现更贴近实际情况的仿真。

## 2.5 单机视觉 SLAM
- to be continue
- 说明： turtlebot3_slam package 有提供激光 SLAM 示例，具体参考官方文档，注意由于 burger 模型被修改成深度摄像头传感器，跑激光 SLAM 需用另外两个机型

## 2.6 单机规划
- to be continue
- 说明： turtlebot3_navigation package 有提供规划示例，但并非实时建图规划，具体可参考官方文档，注意由于 burger 模型已被修改成深度摄像头传感器，跑规划需用另外两个机型

## 2.7 多机SLAM
- to be continue

## 2.8 车机协同