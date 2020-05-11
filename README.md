# vista_robot(ur3_with_robotiq_2f_85)

## 介绍

该功能包使用了ur3机械臂和robotiq作为其末端执行器。搭建了urdf模型及其仿真环境，进行了moveit配置，同时使用了ar_track_alvar用于识别ar标签进行pick和place实验。

<img src="https://github.com/dumabushitu/vista_robot/blob/master/media/pickdemo1.png" width="400">
<img src="https://github.com/dumabushitu/vista_robot/blob/master/media/pickdemo2.png" width="400">

仿真环境里识别精度：通过比较gazebo里实际立方体的位置和rviz里经过tf变换得到的位置，识别误差在1mm以内。
pick and place进行了两种方案，对于20cm高立方体采用从侧面抓取，对于10cm立方体采用从上方抓取，c++代码rviz里演示成功，但是gazebo里抓取物块时，物块会抖动或者脱落，设置有摩擦，未找到解决方法。

<img src="https://github.com/dumabushitu/vista_robot/blob/master/media/gazebodemo1.png" width="400">
<img src="https://github.com/dumabushitu/vista_robot/blob/master/media/gazebodemo2.png" width="400">

python版本进行pick时，在机械臂末端绑定了立方体后会报错
```
Attempting to attach object 'cube_marker' to link 'ee_link' but no geometry specified and such an object does not exist in the collision world
```
然后重复进行pick操作直到尝试次数达到上限（这里设置最多5次）。使用panda机械臂进行测试有同样错误出现
```
Attempting to attach object 'cube_marker' to link 'panda_link8' but no geometry specified and such an object does not exist in the collision world
```
查询资料有遇到同样错误的人，但是无可参考的解决方案。

## 使用方法

1. 启动仿真环境（加载仿真环境，加载机器人，加载控制器插件，加载标签检测节点）

```
roslaunch vistar_gazebo  vistar_simulation.launch
```

2. 启动moveit（启动move_group,启动rviz）

```
roslaunch vistar_moveit_config vistar_moveit_planning_execution.launch
```

​3.进行抓取放置

```
rosrun pick_and_place pick_place_node
```
