# vista_robot(ur3_with_robotiq_2f_85)
## Usage
1.  see robot model in rviz you can run the following command
---
- view robotiq

```
roslaunch robotiq_2f_85_gripper_description view_robotiq.launch
```
- view ur3

```
roslaunch ur3_description view_ur3.launch
```
- view ur3_with_robotiq
```
roslaunch vista_description view_vistar.launch 
```
2. see robot model in gazebo
---
- view robotiq
```
roslaunch vista_gazebo robotiq.launch
```
- view ur3
```
roslaunch vista_gazebo ur3.launch
```
- view vistar
```
roslaunch vista_gazebo vistar.launch
```
- view vistar simulation
```
roslaunch vista_gazebo vista_gazebo vistar_simulation.launch
```
3. see robot moveit demo
---
```
roslaunch vistar_moveit_config demo.launch
```
## Problems for your help
1. robotiq in gazeo which is not full and the error with joints has solved
--
2. robotiq gripper in moveit can plan and excute now
---
3. doubt about the model except vista_robot : how to spaw table、cube_marker and camera is the best way.
---
my method is : 

- camera(kinect): write top xacro for [vistar_with_kinect](./vistar_description/urdf/vistar_with_kinect.xacro) ,then push them together to the parameter and spawn in gazeo ,therefore
the camera has its link.

- table and the cube_marker : i dont send them to the parameter but spawn them in gazebo directly.

**you can see the launch file in vistar_gazebo/vistar_simulation.launch for more details.**

4. I can't see artag in rviz when I run the vistar simulation and ar_track_alvar  nomatter i use kinect or usb camera in gazebo. i dont know what'wrong after i had tried many times in some ways.
--
 
