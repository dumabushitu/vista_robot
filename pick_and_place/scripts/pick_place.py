#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
import tf
from geometry_msgs.msg import PoseStamped, Pose
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, ObjectColor
from moveit_msgs.msg import Grasp, GripperTranslation, MoveItErrorCodes

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import quaternion_from_euler
from copy import deepcopy

GROUP_NAME_ARM = 'arm'
GROUP_NAME_GRIPPER = 'gripper'

GRIPPER_FRAME = 'ee_link'

GRIPPER_OPEN = [0.04,0,0,0]
GRIPPER_CLOSED = [0.35]

REFERENCE_FRAME = 'base_link'

class PickAndPlaceDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 初始化ROS节点
        rospy.init_node('pick_and_place_demo')
        
        # 初始化场景对象
        scene = PlanningSceneInterface()
        
        # 创建一个发布场景变化信息的发布者
        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=10)
        
        # 创建一个发布抓取姿态的发布者
        self.gripper_pose_pub = rospy.Publisher('gripper_pose', PoseStamped, queue_size=10)
        
        # 创建一个存储物体颜色的字典对象
        self.colors = dict()
                        
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = MoveGroupCommander(GROUP_NAME_ARM)
        
        # 初始化需要使用move group控制的机械臂中的gripper group
        gripper = MoveGroupCommander(GROUP_NAME_GRIPPER)

        # 获取终端link的名称
        #end_effector_link = arm.get_end_effector_link()
 
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.05)
        arm.set_goal_orientation_tolerance(0.1)

        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)
        
        # 设置目标位置所使用的参考坐标系
        #arm.set_pose_reference_frame(REFERENCE_FRAME)
        
        # 设置每次运动规划的时间限制：20s
        arm.set_planning_time(20)
        
        # 设置pick和place阶段的最大尝试次数
        max_pick_attempts = 5
        max_place_attempts = 5
        rospy.sleep(2)

        # 设置场景物体的名称 
        table1_id = 'table1'
        table2_id = 'table2'
        target_id = 'cube_marker'
                
        # 移除场景中之前运行残留的物体
        scene.remove_world_object(table1_id)
        scene.remove_world_object(table2_id)
        scene.remove_world_object(target_id)
        
        # 移除场景中之前与机器臂绑定的物体
        scene.remove_attached_object(GRIPPER_FRAME, target_id)  
        rospy.sleep(1)
        
        # 控制机械臂先运动到准备位置
        arm.set_named_target('home')
        arm.go()
        
        # 控制夹爪张开
        gripper.set_joint_value_target(GRIPPER_OPEN)
        gripper.go()
        rospy.sleep(1)
     
        # 设置table的三维尺寸[长, 宽, 高]
        table1_size = [0.8, 1.5, 0.03]
        table2_size = [1.5, 0.8, 0.03]
        
        # 将两张桌子加入场景当中
        table1_pose = PoseStamped()
        table1_pose.header.frame_id = 'base_link'
        table1_pose.pose.position.x = 0
        table1_pose.pose.position.y = 1.05
        table1_pose.pose.position.z = -0.05
        table1_pose.pose.orientation.w = 1.0
        scene.add_box(table1_id, table1_pose, table1_size)

        table2_pose = PoseStamped()
        table2_pose.header.frame_id = 'base_link'
        table2_pose.pose.position.x = -1.05
        table2_pose.pose.position.y = -0.1
        table2_pose.pose.position.z = -0.05
        table2_pose.pose.orientation.w = 1.0
        scene.add_box(table2_id, table2_pose, table2_size)
                             
        # 将桌子设置成红色
        self.setColor(table1_id, 0.8, 0.4, 0, 1.0)
        self.setColor(table2_id, 0.8, 0.4, 0, 1.0)

        #监听目标到'base_link'的tf变换
        listener = tf.TransformListener()
        while not rospy.is_shutdown():
            try:
                (trans,rot) = listener.lookupTransform('base_link', 'ar_marker_0', rospy.Time(0))
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.loginfo("Waiting for transform between 'base_link' and 'ar_marker_0'")
                rospy.sleep(1)

        rospy.loginfo("Found transform between 'base_link' and 'ar_marker_0'") 

        # 设置目标物体的尺寸
        target_size = [0.05, 0.05, 0.2]
        
        # 设置目标物体的位置
        target_pose = PoseStamped()
        target_pose.header.frame_id = REFERENCE_FRAME
        target_pose.pose.position.x = trans[0]
        target_pose.pose.position.y = trans[1]
        target_pose.pose.position.z = 0.065
        target_pose.pose.orientation.x = rot[0]
        target_pose.pose.orientation.y = rot[1]
        target_pose.pose.orientation.z = rot[2]
        target_pose.pose.orientation.w = rot[3]
       
        # 将抓取的目标物体加入场景中
        scene.add_box(target_id, target_pose, target_size)
        
        # 将目标物体设置为黄色
        self.setColor(target_id, 0.9, 0.9, 0, 1.0)
        
        # 将场景中的颜色设置发布
        self.sendColors()

        # 设置支持的外观
        arm.set_support_surface_name(table2_id)
        
        # 设置一个place阶段需要放置物体的目标位置
        place_pose = PoseStamped()
        place_pose.header.frame_id = REFERENCE_FRAME
        place_pose.pose.position.x =  -0.5
        place_pose.pose.position.y =  0
        place_pose.pose.position.z =  0.065

        while not rospy.is_shutdown():
            try:
                listener.waitForTransform('ar_marker_0', 'ee_link', rospy.Time(0),rospy.Duration(4.0))
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.loginfo("Waiting for transform between 'ar_marker_0' and 'ee_link'")
                rospy.sleep(1)

        rospy.loginfo("Found transform between 'ar_marker_0' and 'ee_link'") 

        # 将目标位置设置为机器人的抓取目标位置
        grasp_pose = PoseStamped()
        grasp_pose.header.frame_id = 'ar_marker_0'
        grasp_pose.pose.position.x =  0
        grasp_pose.pose.position.y =  -0.15
        grasp_pose.pose.position.z =  -0.1
                
        # 生成抓取姿态
        grasps = self.make_grasps(grasp_pose, [target_id])

        # 将抓取姿态发布，可以在rviz中显示
        for grasp in grasps:
            self.gripper_pose_pub.publish(grasp.grasp_pose)
            rospy.sleep(0.2)
    
        # 追踪抓取成功与否，以及抓取的尝试次数
        result = None
        n_attempts = 0
        
        # 重复尝试抓取，直道成功或者超多最大尝试次数
        while result != MoveItErrorCodes.SUCCESS and n_attempts < max_pick_attempts:
            n_attempts += 1
            rospy.loginfo("Pick attempt: " +  str(n_attempts))
            result = arm.pick(target_id, grasps)
            rospy.sleep(0.2)
        
        # 如果pick成功，则进入place阶段 
        if result == MoveItErrorCodes.SUCCESS:
            result = None
            n_attempts = 0
            
            # 生成放置姿态
            places = self.make_places(place_pose)
            
            # 重复尝试放置，直道成功或者超多最大尝试次数
            while result != MoveItErrorCodes.SUCCESS and n_attempts < max_place_attempts:
                n_attempts += 1
                rospy.loginfo("Place attempt: " +  str(n_attempts))
                for place in places:
                    result = arm.place(target_id, place)
                    if result == MoveItErrorCodes.SUCCESS:
                        break
                rospy.sleep(0.2)
                
            if result != MoveItErrorCodes.SUCCESS:
                rospy.loginfo("Place operation failed after " + str(n_attempts) + " attempts.")
        else:
            rospy.loginfo("Pick operation failed after " + str(n_attempts) + " attempts.")
                
        # 控制机械臂回到初始化位置
        arm.set_named_target('home')
        arm.go()
        
        # 控制夹爪回到张开的状态
        gripper.set_joint_value_target(GRIPPER_OPEN)
        gripper.go()
        rospy.sleep(1)

        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)
        
    # 创建夹爪的姿态数据JointTrajectory
    def make_gripper_posture(self, joint_positions):
        # 初始化夹爪的关节运动轨迹
        t = JointTrajectory()
        
        # 设置夹爪的关节名称
        t.joint_names = ['gripper_right_driver_joint']
        
        # 初始化关节轨迹点
        tp = JointTrajectoryPoint()
        
        # 将输入的关节位置作为一个目标轨迹点
        tp.positions = joint_positions
        
        # 设置夹爪的力度
        tp.effort = [2.0]
        
        # 设置运动时间
        tp.time_from_start = rospy.Duration(1.0)
        
        # 将目标轨迹点加入到运动轨迹中
        t.points.append(tp)
        
        # 返回夹爪的关节运动轨迹
        return t
    
    # 使用给定向量创建夹爪的translation结构
    def make_gripper_translation(self, min_dist, desired, frame, vector):
        # 初始化translation对象
        g = GripperTranslation()
        
        # 设置方向向量
        g.direction.vector.x = vector[0]
        g.direction.vector.y = vector[1]
        g.direction.vector.z = vector[2]
        
        # 设置参考坐标系
        g.direction.header.frame_id = frame
        
        # 设置最小和期望的距离
        g.min_distance = min_dist
        g.desired_distance = desired
        
        return g

    # 创建一个允许的的抓取姿态列表
    def make_grasps(self, initial_pose_stamped, allowed_touch_objects):
        # 初始化抓取姿态对象
        g = Grasp()
        
        # 创建夹爪张开、闭合的姿态
        g.pre_grasp_posture = self.make_gripper_posture(GRIPPER_OPEN)

        g.grasp_posture = self.make_gripper_posture(GRIPPER_CLOSED)
                
        # 设置期望的夹爪靠近、撤离目标的参数
        g.pre_grasp_approach = self.make_gripper_translation(0.05, 0.1, 'ar_marker_0' ,[0.0, 1.0, 0.0])
        g.post_grasp_retreat = self.make_gripper_translation(0.1, 0.15,'base_link', [0.0, 0.0, 1.0])
        
        # 设置抓取姿态
        g.grasp_pose = initial_pose_stamped

        # 需要尝试改变姿态的数据列表
        yaw_vals = [0, 0.1, -0.1, 0.2, -0.2, 0.3, -0.3]
        y_vals = [0, 0.005, 0.01, 0.015, -0.005, -0.01, -0.015]

        # 抓取姿态的列表
        grasps = []

        # 改变姿态，生成抓取动作
        #for Y in yaw_vals:

        # 欧拉角到四元数的转换
        q = quaternion_from_euler(3.1415, 0, 1.5707)                
        # 设置抓取的姿态
        g.grasp_pose.pose.orientation.x = q[0]
        g.grasp_pose.pose.orientation.y = q[1]
        g.grasp_pose.pose.orientation.z = q[2]
        g.grasp_pose.pose.orientation.w = q[3]
        
        # 设置抓取的唯一id号
        g.id = str(len(grasps))
        
        # 设置允许接触的物体
        g.allowed_touch_objects = allowed_touch_objects
        
        # 将本次规划的抓取放入抓取列表中
        grasps.append(deepcopy(g))
       
        # 返回抓取列表
        return grasps
    
    # 创建一个允许的放置姿态列表
    def make_places(self, init_pose):
        # 初始化放置抓取物体的位置
        place = PoseStamped()
        
        # 设置放置抓取物体的位置
        place = init_pose
        
        # 定义x方向上用于尝试放置物体的偏移参数
        x_vals = [0, 0.005, 0.01, 0.015, -0.005, -0.01, -0.015]
        
        # 定义y方向上用于尝试放置物体的偏移参数
        y_vals = [0, 0.005, 0.01, 0.015, -0.005, -0.01, -0.015]
        
        pitch_vals = [0]
        
        # 定义用于尝试放置物体的偏航角参数
        yaw_vals = [0]

        # 定义放置物体的姿态列表
        places = []
        
        # 生成每一个角度和偏移方向上的抓取姿态
        #for Y in yaw_vals:
           # for p in pitch_vals:
        for y in y_vals:
            for x in x_vals:
                place.pose.position.x = init_pose.pose.position.x + x
                place.pose.position.y = init_pose.pose.position.y + y
                
                # 欧拉角到四元数的转换
                q = quaternion_from_euler(3.1415, 0, 3.1415)
                
                # 欧拉角到四元数的转换
                place.pose.orientation.x = q[0]
                place.pose.orientation.y = q[1]
                place.pose.orientation.z = q[2]
                place.pose.orientation.w = q[3]
                
                # 将该放置姿态加入列表
                places.append(deepcopy(place))
        
        # 返回放置物体的姿态列表
        return places
    
    # 设置场景物体的颜色
    def setColor(self, name, r, g, b, a = 0.9):
        # 初始化moveit颜色对象
        color = ObjectColor()
        
        # 设置颜色值
        color.id = name
        color.color.r = r
        color.color.g = g
        color.color.b = b
        color.color.a = a
        
        # 更新颜色字典
        self.colors[name] = color

    # 将颜色设置发送并应用到moveit场景当中
    def sendColors(self):
        # 初始化规划场景对象
        p = PlanningScene()

        # 需要设置规划场景是否有差异     
        p.is_diff = True
        
        # 从颜色字典中取出颜色设置
        for color in self.colors.values():
            p.object_colors.append(color)
        
        # 发布场景物体颜色设置
        self.scene_pub.publish(p)

if __name__ == "__main__":
    PickAndPlaceDemo()

    