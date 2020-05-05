# vista_robot(ur3_with_robotiq_2f_85)

## 介绍

该功能包用于识别ar标签进行pick和place仿真

包含ur3和robotiq的机械臂模型，gazebo里的仿真实验环境，标签识别和pick_place。

## 使用方法

1.  rviz里查看模型

- robotiq

```
roslaunch robotiq_2f_85_gripper_description view_robotiq.launch
```

- ur3

```
roslaunch ur3_description view_ur3.launch
```

-  ur3_with_robotiq

```
roslaunch vistar_description view_vistar.launch 
```

2.  gazebo里查看模型

---

- robotiq

```
roslaunch vistar_gazebo robotiq.launch
```

-  ur3

```
roslaunch vistar_gazebo ur3.launch
```

-  vistar

```
roslaunch vistar_gazebo vistar.launch
```

- 加载仿真环境

```
roslaunch vistar_gazebo  vistar_simulation.launch
```

3. 启动moveit

```
roslaunch vistar_moveit_config vistar_moveit_planning_execution.launch
```

​     4.启动标签检测节点

```
roslaunch marker_detection aruco_camera.launch
```

​	5.启动pickplace节点

```
rosrun pick_and_place pick_place_2_node
```



