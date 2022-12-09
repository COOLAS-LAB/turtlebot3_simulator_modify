# turtlebot3 ���滷��
# 0. ����
- ��Ŀ��Դ https://github.com/ROBOTIS-GIT/turtlebot3

# 1. ����
```shell
mkdir -p turtlebot3_ws/src
cd turtlebot3/src
git clone https://github.com/yonggaogit/turtlebot3_simulator_modify.git
cd ..
catkin_make
```

# 2. ����
## 2.1 ָ������

turtlebot3 һ���ṩ���ַ�����ͣ�burger, waffle, waffle_pi������ÿ�� roslaunch �� rosrun ǰ��Ҫͨ�� `export TURTLEBOT3_MODEL=burger`ָ�����ͣ����齫��д��`~/.bshrc` �ļ���`sudo gedit ~/.bashrc` �󣬽� `export TURTLEBOT3_MODEL=burger` ������ļ�β������ÿ�ζ���Ҫָ����ע����������Ҫ�ؿ� terminal ���ڵ�ǰ terminal ���� `source ~/.bashrc`���ҵ�ǰ���Ѿ��޸� burger ģ�� urdf �ļ������������ kinetic �������ͷ��

## 2.2 �����ֶ�����
- ����
```shell
# turtlebot3_ws Ŀ¼���½� terminal
source devel/setup.bash
roslaunch turtlebot3_gazebo single_turtlebot3.launch

# turtlebot3_ws Ŀ¼���½� terminal
source devel/setup.bash
roslaunch turtlebot3_teleop single_turtlebot3_teleop_key.launch
# w ǰ����x ���ˡ�a ��ת��d ��ת��s ֹͣ
```
- ����˵��

ͨ���޸� `single_turtlebot3.launch` �� `<arg name="world_name" value="$(find turtlebot3_gazebo)/worlds/turtlebot3_house.world"/>`  �� `turtlebot3_house.world` Ϊ `turtlebot3_gazebo/worlds` �ļ����µ����� world ��׺�ļ������л���ͬ�ĳ�����

ͨ���޸� `single_turtlebot3.launch` ��
```shell
<arg name="x_pos" default="0.0"/>
<arg name="y_pos" default="0.0"/>
<arg name="z_pos" default="0.0"/>
<arg name="yaw" default="0.0"/>
```
�޸����˳��ĳ�ʼλ�ˣ��� world ����ϵ�µģ�

- ���û���˵��

/camera/depth/image_raw Ϊ���ͼ����ͨ�� rqt_image_view �� rviz �鿴��/camera/depth/points Ϊ���ƣ���ͨ�� rviz �鿴

/camera/rgb/image_raw Ϊ rgb ͼ����ͨ�� rqt_image_view �� rviz �鿴

/cmd_vel Ϊ�ٶȿ��ƻ��⣬����ͨ�� `rostopic pub` ָ����Կ��ƻ����ˣ�ʵ��ͨ�� `single_turtlebot3_teleop_key` �ű����̿��Ƹ����㣬�漰���滮�㷨ʱ��Ҫ�õ��û���

/odom Ϊ��̼Ƶ� ground_truth ֵ������ world ����ϵΪ��׼��


## 2.3 ����ֶ�����
- ����
```shell
# turtlebot3_ws Ŀ¼���½� terminal
source devel/setup.bash
roslaunch turtlebot3_gazebo multi_turtlebot3.launch

# turtlebot3_ws Ŀ¼���½� terminal
source devel/setup.bash
roslaunch turtlebot3_teleop multi_turtlebot3_teleop_key.launch
# turtlebot3_0: w ǰ����x ���ˡ�a ��ת��d ��ת��s ֹͣ
# turtlebot3_1: u ǰ����m ���ˡ�h ��ת��k ��ת��j ֹͣ
# turtlebot3_2: 8 ǰ����2 ���ˡ�4 ��ת��6 ��ת��5 ֹͣ
```
- ����˵��

ͨ���޸� `single_turtlebot3.launch` �� `<arg name="world_name" value="$(find turtlebot3_gazebo)/worlds/turtlebot3_house.world"/>`  �� `turtlebot3_house.world` Ϊ `turtlebot3_gazebo/worlds` �ļ����µ����� world ��׺�ļ������л���ͬ�ĳ�����

ͨ���޸� `single_turtlebot3.launch` ��
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
�������������˳��ĳ�ʼλ��

- ���û���˵��

ͬ����һ����ֻ�Ƕ����������˼�����ǰ׺ `/tb3_0/`��`/tb3_1/`��`/tb3_2/` ��������

## 2.4 UWB
ֻ�ڶ����ʹ��
ͨ�� `/tb3_x/odom` ��ȡ����� world ����ϵ��λ�ˣ�ͨ����λ�˽����˳��ϵ����� UWB ת������������ϵ������ UWB ����������ϵ�����໥֮��ľ��벢����һ���ĸ�˹������Ϊ���档

�������Ϊ `/uwb/dis_ra` �� `/uwb/dis_rb` ��Ϊ `std_msgs/Float64MultiArray` ���ͣ������飬��ʾ���� refrence ���˳���/tb3_0/���� a ���˳���/tb3_1/�� �� b ���˳���/tb3_2/��֮�� UWB �Եľ��롣

Ĭ�� /tb3_0/ ���˳��ϰ�װ 3 �� UWB�������������ϰ�װ���� UWB��

ͨ���޸� `/turtlebot3_simulations/turtlebot3_gazebo/launch/multi_turtlebot3.launch` �е�

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

|����|����|
|-|-|
|noise_mean|���������ֵ|
|noise_stddev|�����������|
|uwb_r1_x|reference ���ϵĵ�һ�� UWB �� x ���꣨����ڳ��Ļ�������ϵ��|
|uwb_r1_y|reference ���ϵĵ�һ�� UWB �� y ���꣨����ڳ��Ļ�������ϵ��|
|uwb_r2_x|reference ���ϵĵڶ��� UWB �� x ���꣨����ڳ��Ļ�������ϵ��|
|uwb_r2_y|reference ���ϵĵڶ��� UWB �� y ���꣨����ڳ��Ļ�������ϵ��|
|uwb_r3_x|reference ���ϵĵ����� UWB �� x ���꣨����ڳ��Ļ�������ϵ��|
|uwb_r3_y|reference ���ϵĵ����� UWB �� y ���꣨����ڳ��Ļ�������ϵ��|
|uwb_a0_x|a ���ϵĵ�һ�� UWB �� x ���꣨����ڳ��Ļ�������ϵ��|
|uwb_a0_y|a ���ϵĵ�һ�� UWB �� y ���꣨����ڳ��Ļ�������ϵ��|
|uwb_a1_x|a ���ϵĵڶ��� UWB �� x ���꣨����ڳ��Ļ�������ϵ��|
|uwb_a1_y|a ���ϵĵڶ��� UWB �� y ���꣨����ڳ��Ļ�������ϵ��|
|uwb_b0_x|b ���ϵĵ�һ�� UWB �� x ���꣨����ڳ��Ļ�������ϵ��|
|uwb_v0_y|b ���ϵĵ�һ�� UWB �� y ���꣨����ڳ��Ļ�������ϵ��|
|uwb_b1_x|b ���ϵĵڶ��� UWB �� x ���꣨����ڳ��Ļ�������ϵ��|
|uwb_b1_y|b ���ϵĵڶ��� UWB �� y ���꣨����ڳ��Ļ�������ϵ��|

- to be continue

��ʵ�ϣ� UWB ֮��Ĳ�������ž��������Խ��Խ��׼�ģ����ڴ�Խ�ϰ���ʱ���������󣬺��ڼ�����Щ������ʵ�ָ�����ʵ������ķ��档

## 2.5 �����Ӿ� SLAM
- to be continue
- ˵���� turtlebot3_slam package ���ṩ���� SLAM ʾ��������ο��ٷ��ĵ���ע������ burger ģ�ͱ��޸ĳ��������ͷ���������ܼ��� SLAM ����������������

## 2.6 �����滮
- to be continue
- ˵���� turtlebot3_navigation package ���ṩ�滮ʾ����������ʵʱ��ͼ�滮������ɲο��ٷ��ĵ���ע������ burger ģ���ѱ��޸ĳ��������ͷ���������ܹ滮����������������

## 2.7 ���SLAM
- to be continue

## 2.8 ����Эͬ