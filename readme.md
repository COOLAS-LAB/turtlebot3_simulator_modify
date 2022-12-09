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
- ���û���˵��

/camera/depth/image_raw Ϊ���ͼ����ͨ�� rqt_image_view �� rviz �鿴��/camera/depth/points Ϊ���ƣ���ͨ�� rviz �鿴

/camera/rgb/image_raw Ϊ rgb ͼ����ͨ�� rqt_image_view �� rviz �鿴

/cmd_vel Ϊ�ٶȿ��ƻ��⣬����ͨ�� `rostopic pub` ָ����Կ��ƻ����ˣ�ʵ��ͨ�� `single_turtlebot3_teleop_key` �ű����̿��Ƹ����㣬�漰���滮�㷨ʱ��Ҫ�õ��û���


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

- ���û���˵��

ͬ����һ����ֻ�Ƕ����������˼�����ǰ׺ `/tb3_0/`��`/tb3_1/`��`/tb3_2/` ��������

## 2.4 UWB
- to be continue

## 2.5 �����Ӿ� SLAM
- to be continue
- ˵���� turtlebot3_slam package ���ṩ���� SLAM ʾ��������ο��ٷ��ĵ���ע������ burger ģ�ͱ��޸ĳ��������ͷ���������ܼ��� SLAM ����������������

## 2.6 �����滮
- to be continue
- ˵���� turtlebot3_navigation package ���ṩ�滮ʾ����������ʵʱ��ͼ�滮������ɲο��ٷ��ĵ���ע������ burger ģ���ѱ��޸ĳ��������ͷ���������ܹ滮����������������

## 2.7 ���SLAM
- to be continue

