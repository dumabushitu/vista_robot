import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
from moveit_commander.conversions import pose_to_list

GRIPPER_OPEN = [0,0,0,0,0,0]
GRIPPER_CLOSED = [0,0,0,0.35,0,0]

REFERENCE_FRAME = 'base_link'

class PickandPlace(object):
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()
    ## 首先初始化`moveit_commander`和 `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('pick_and_place_demo',anonymous=True)

    ## 初始化一个`RobotCommander`对象. 
    robot = moveit_commander.RobotCommander()

    ## 初始化一个`PlanningSceneInterface`对象
    scene = moveit_commander.PlanningSceneInterface()

    ## 初始化 一个`MoveGroupCommander`对象.  T
    group_name = "arm"
    group = moveit_commander.MoveGroupCommander(group_name)

    ## 创建一个 `DisplayTrajectory`发布者 用来在rviz可视化轨迹
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

                        
    #获取调试信息
    # 获取机器人的参考坐标系:
    planning_frame = group.get_planning_frame()
    print "============ Reference frame: %s" % planning_frame

    # 打印规划组终端link的名字:
    eef_link = group.get_end_effector_link()
    print "============ End effector: %s" % eef_link

    # 获取机器人的所有规划组:
    group_names = robot.get_group_names()
    print "============ Robot Groups:", robot.get_group_names()

    # 获取机器人的当前状态
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""


    #各种取值
    self.target_id = 'cube_marker'
    self.target_pose = PoseStamped()
    self.robot = robot
    self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

  def prepare(self):

    table1_id = 'table1'
    table2_id = 'table2'
    target_id = self.target_id
            
    # 移除场景中之前运行残留的物体
    scene.remove_world_object(table1_id)
    scene.remove_world_object(table2_id)
    scene.remove_world_object(target_id)
    
    # 移除场景中之前与机器臂绑定的物体
    scene.remove_attached_object('tool0',target_id )  
    rospy.sleep(1)
    
    group = self.group

    # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
    group.set_goal_position_tolerance(0.01)
    group.set_goal_orientation_tolerance(0.05)

    # 控制机械臂先运动到准备位置
    group.set_named_target('up')
    group.go()
    
    # 控制夹爪张开
    gripper.set_joint_value_target(GRIPPER_OPEN)
    gripper.go()
    rospy.sleep(1)
    group = self.group

  def add_collisions(self):
    scene = self.scene
    target_id = self.target_id
    # 将两张桌子加入场景当中
    table1_size = [0.8, 1.5, 0.03]
    table2_size = [1.5, 0.8, 0.03]
    
    # 设置table1和table2的三维尺寸[长, 宽, 高]
    table1_pose = PoseStamped()
    table1_pose.header.frame_id = REFERENCE_FRAME
    table1_pose.pose.position.x = 0
    table1_pose.pose.position.y = 1.05
    table1_pose.pose.position.z = -0.05
    table1_pose.pose.orientation.w = 1.0
    scene.add_box(table1_id, table1_pose, table1_size)

    table2_pose = PoseStamped()
    table2_pose.header.frame_id = REFERENCE_FRAME
    table2_pose.pose.position.x = -1.05
    table2_pose.pose.position.y = -0.1
    table2_pose.pose.position.z = -0.05
    table2_pose.pose.orientation.w = 1.0
    scene.add_box(table2_id, table2_pose, table2_size)

    #监听目标到base_link的tf变换
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
    target_size = [0.05, 0.05, 0.1]
    
    # 设置目标物体的位置
    target_pose = PoseStamped()
    target_pose.header.frame_id = REFERENCE_FRAME
    target_pose.pose.position.x = trans[0]
    target_pose.pose.position.y = trans[1]
    target_pose.pose.position.z = 0.015
    target_pose.pose.orientation.x = rot[0]
    target_pose.pose.orientation.y = rot[1]
    target_pose.pose.orientation.z = rot[2]
    target_pose.pose.orientation.w = rot[3]
    self.target_pose = target_pose
    
    # 将抓取的目标物体加入场景中
    scene.add_box(target_id, target_pose, target_size)

    return self.wait_for_state_update(box_is_known=True, timeout=4)
  

    def go_to_pose_goal(self):
    group = self.group

    # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
    group.set_goal_position_tolerance(0.01)
    group.set_goal_orientation_tolerance(0.05)

    pose_goal = self.target_pose

    rot=(
    pose_goal.pose.orientation.x,
    pose_goal.pose.orientation.y,
    pose_goal.pose.orientation.z,
    pose_goal.pose.orientation.w)

    p = euler_from_quaternion(rot)
    q = quaternion_from_euler(pi, 0, p[2])  

    pose_goal = PoseStamped()
    pose_goal.position.z += 0.28
    pose_goal.orientation.x = q[0]
    pose_goal.orientation.y = q[1]
    pose_goal.orientation.z = q[2]
    pose_goal.orientation.w = q[3]

    group.set_pose_target(pose_goal)
    
    # 规划运动路径
    traj = group.plan()
        
    # 按照规划的运动路径控制机械臂运动
    group.execute(traj)
    rospy.sleep(1)

  def plan_cartesian_path(self,vector,desire):
    group = self.group

    waypoints = []

    wpose = group.get_current_pose().pose
    wpose.position.x =  vector[0] * desire[0]
    wpose.position.y =  vector[1] * desire[1]
    wpose.position.z =  vector[2] * desire[2]  
    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # 路点列表
                                       0.01,        # 终端步进值
                                       0.0)         # 跳跃阈值

    return plan, fraction

  def display_trajectory(self, plan):
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    display_trajectory_publisher.publish(display_trajectory);

  
  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    target_id = self.target_id
    scene = self.scene
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
        # 查看目标是否在连接列表
        attached_objects = scene.get_attached_objects([target_id])
        is_attached = len(attached_objects.keys()) > 0

        # 查看目标是否在场景中
        is_known = target_id in scene.get_known_object_names()

        # 查看是否是我们期望的状态
        if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

        rospy.sleep(0.1)
        seconds = rospy.get_time()

    return False

  def attach_box(self, timeout=4):
    target_id = self.target_id
    robot = self.robot
    scene = self.scene
    eef_link = self.eef_link
    group_names = self.group_names

    grasping_group = 'gripper'
    gripper.set_named_target('close')
    gripper.go
    sleep(2)

    touch_links = robot.get_link_names(group=grasping_group)
    scene.attach_box(eef_link, target_id, touch_links=touch_links)

    return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

  def detach_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene
    eef_link = self.eef_link

    ## BEGIN_SUB_TUTORIAL detach_object
    ##
    ## Detaching Objects from the Robot
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## We can also detach and remove the object from the planning scene:
    scene.remove_attached_object(eef_link, name=box_name)
    ## END_SUB_TUTORIAL

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)


def main():
  try:
    print "============ Press `Enter` to begin the tutorial by setting up the moveit_commander (press ctrl-d to exit) ..."
    raw_input()
    tutorial = MoveGroupPythonIntefaceTutorial()

    print "============ Press `Enter` to execute a movement using a joint state goal ..."
    raw_input()
    tutorial.go_to_joint_state()

    print "============ Press `Enter` to execute a movement using a pose goal ..."
    raw_input()
    tutorial.go_to_pose_goal()

    print "============ Press `Enter` to plan and display a Cartesian path ..."
    raw_input()
    cartesian_plan, fraction = tutorial.plan_cartesian_path()

    print "============ Press `Enter` to display a saved trajectory (this will replay the Cartesian path)  ..."
    raw_input()
    tutorial.display_trajectory(cartesian_plan)

    print "============ Press `Enter` to execute a saved path ..."
    raw_input()
    tutorial.execute_plan(cartesian_plan)

    print "============ Press `Enter` to add a box to the planning scene ..."
    raw_input()
    tutorial.add_box()

    print "============ Press `Enter` to attach a Box to the Panda robot ..."
    raw_input()
    tutorial.attach_box()

    print "============ Press `Enter` to plan and execute a path with an attached collision object ..."
    raw_input()
    cartesian_plan, fraction = tutorial.plan_cartesian_path(scale=-1)
    tutorial.execute_plan(cartesian_plan)

    print "============ Press `Enter` to detach the box from the Panda robot ..."
    raw_input()
    tutorial.detach_box()

    print "============ Press `Enter` to remove the box from the planning scene ..."
    raw_input()
    tutorial.remove_box()

    print "============ Python tutorial demo complete!"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()