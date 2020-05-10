#include <ros/ros.h>

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>

#include "tf/transform_datatypes.h"

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

void openGripper(trajectory_msgs::JointTrajectory& posture)
{
  posture.joint_names.resize(6);
  posture.joint_names[0] = "gripper_left_driver_joint";
  posture.joint_names[1] = "gripper_left_follower_joint";
  posture.joint_names[2] = "gripper_left_spring_link_joint";
  posture.joint_names[3] = "gripper_right_driver_joint";
  posture.joint_names[4] = "gripper_right_follower_joint";
  posture.joint_names[5] = "gripper_right_spring_link_joint";

  posture.points.resize(1);
  posture.points[0].positions.resize(6);
  posture.points[0].positions[0] = 0;
  posture.points[0].positions[1] = 0;
  posture.points[0].positions[2] = 0;
  posture.points[0].positions[3] = 0;
  posture.points[0].positions[4] = 0;
  posture.points[0].positions[5] = 0;
  posture.points[0].time_from_start = ros::Duration(0.5);
}

void closedGripper(trajectory_msgs::JointTrajectory& posture)
{
posture.joint_names.resize(6);
  posture.joint_names[0] = "gripper_left_driver_joint";
  posture.joint_names[1] = "gripper_left_follower_joint";
  posture.joint_names[2] = "gripper_left_spring_link_joint";
  posture.joint_names[3] = "gripper_right_driver_joint";
  posture.joint_names[4] = "gripper_right_follower_joint";
  posture.joint_names[5] = "gripper_right_spring_link_joint";

  posture.points.resize(1);
  posture.points[0].positions.resize(6);
  posture.points[0].positions[0] = 0;
  posture.points[0].positions[1] = 0;
  posture.points[0].positions[2] = 0;
  posture.points[0].positions[3] = 0.41;
  posture.points[0].positions[4] = 0;
  posture.points[0].positions[5] = 0;
  posture.points[0].time_from_start = ros::Duration(0.5);
}

void pick(moveit::planning_interface::MoveGroupInterface& move_group)
{
  // 创建一个抓取向量，这里只创建一个抓取
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);

  tf::TransformListener listener;
  while (ros::ok()) {
    try{
      listener.waitForTransform("ar_marker_0", "tool0",ros::Time(0), ros::Duration(2));
      break;
    }
    catch (tf::TransformException ex){
      ROS_INFO("Waiting for transform between 'ar_marker_0' and 'tool0'");
      ros::Duration(1.0).sleep();
    }
  }
  ROS_INFO("Found transform between 'ar_marker_0' and 'tool0'");

  // 设置抓取姿态
  grasps[0].grasp_pose.header.frame_id = "ar_marker_0";

  tf2::Quaternion orientation;
  orientation.setRPY(M_PI/2, 0, M_PI);
  grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
  grasps[0].grasp_pose.pose.position.x = 0;
  grasps[0].grasp_pose.pose.position.y = -0.15;
  grasps[0].grasp_pose.pose.position.z = -0.1;

  // 设置 pre-grasp approach
  grasps[0].pre_grasp_approach.direction.header.frame_id = "ar_marker_0";

  grasps[0].pre_grasp_approach.direction.vector.y = 1.0;
  grasps[0].pre_grasp_approach.min_distance = 0.095;
  grasps[0].pre_grasp_approach.desired_distance = 0.115;

  // 设置 post-grasp retreat
  grasps[0].post_grasp_retreat.direction.header.frame_id = "base_link";

  grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
  grasps[0].post_grasp_retreat.min_distance = 0.1;
  grasps[0].post_grasp_retreat.desired_distance = 0.25;

  // 设置抓取前爪子的状态
  openGripper(grasps[0].pre_grasp_posture);

  // 设置爪子抓取的状态
  closedGripper(grasps[0].grasp_posture);

  // 设置支撑面.
  move_group.setSupportSurfaceName("table1");
  // 调用pick使用给定的grasps抓取对象
  move_group.pick("object", grasps);
}

void place(moveit::planning_interface::MoveGroupInterface& group)
{
  // 创建一个尝试的向量，这里只创建一个放置位置
  std::vector<moveit_msgs::PlaceLocation> place_location;
  place_location.resize(1);

  // 设置放置的姿态
  place_location[0].place_pose.header.frame_id = "base_link";
  tf2::Quaternion orientation;
  orientation.setRPY(0, 0, M_PI / 2);
  place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

  //放置对象
  place_location[0].place_pose.pose.position.x = -0.55;
  place_location[0].place_pose.pose.position.y = 0;
  place_location[0].place_pose.pose.position.z = 0.05;

  // 设置pre-place approach

  place_location[0].pre_place_approach.direction.header.frame_id = "base_link";

  place_location[0].pre_place_approach.direction.vector.z = -1.0;
  place_location[0].pre_place_approach.min_distance = 0.095;
  place_location[0].pre_place_approach.desired_distance = 0.115;

  // 设置 post-grasp retreat
  place_location[0].post_place_retreat.direction.header.frame_id = "base_link";

  place_location[0].post_place_retreat.direction.vector.x = 1.0;
  place_location[0].post_place_retreat.min_distance = 0.1;
  place_location[0].post_place_retreat.desired_distance = 0.25;

  //设置放置对象完成后机械爪的状态
  openGripper(place_location[0].post_place_posture);

  // 设置支撑面table2.
  group.setSupportSurfaceName("table2");
  // 调用place将对象放置到给定的位置
  group.place("object", place_location);
}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,const geometry_msgs::Pose& marker_pose)
{
  //先删除相关物体
  std::vector<std::string> object_ids;
  object_ids.push_back("object");
  object_ids.push_back("table1");
  object_ids.push_back("table2");
  planning_scene_interface.removeCollisionObjects(object_ids);

  // 添加桌子和物块
  // 生成规划环境
  // 创建容纳2个碰撞对象的向量  
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(3);

  // 添加第一个放置ur机械臂的桌子.
  collision_objects[0].id = "table1";
  collision_objects[0].header.frame_id = "base_link";

  //定义桌子1的形状和尺寸
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.4;
  collision_objects[0].primitives[0].dimensions[1] = 0.2;
  collision_objects[0].primitives[0].dimensions[2] = 0.4;

  //定义桌子1的位置
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0;
  collision_objects[0].primitive_poses[0].position.y = 0.55;
  collision_objects[0].primitive_poses[0].position.z = -0.25;


  collision_objects[0].operation = collision_objects[0].ADD;

  //添加第二张桌子
  collision_objects[1].id = "table2";
  collision_objects[1].header.frame_id = "base_link";

  //定义桌子的形状和尺寸
  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 0.2;
  collision_objects[1].primitives[0].dimensions[1] = 0.4;
  collision_objects[1].primitives[0].dimensions[2] = 0.4;

  //定义桌子的位置
  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = -0.55;
  collision_objects[1].primitive_poses[0].position.y = 0;
  collision_objects[1].primitive_poses[0].position.z = -0.25;


  collision_objects[1].operation = collision_objects[1].ADD;

  // 定义将要抓取的物块
  collision_objects[2].header.frame_id = "base_link";
  collision_objects[2].id = "object";

  //定义物块的形状和尺寸
  collision_objects[2].primitives.resize(1);
  collision_objects[2].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[2].primitives[0].dimensions.resize(3);
  collision_objects[2].primitives[0].dimensions[0] = 0.05;
  collision_objects[2].primitives[0].dimensions[1] = 0.05;
  collision_objects[2].primitives[0].dimensions[2] = 0.2;

  //定义物块的位置
  collision_objects[2].primitive_poses.resize(1);
  collision_objects[2].primitive_poses[0].position.x = marker_pose.position.x;
  collision_objects[2].primitive_poses[0].position.y = marker_pose.position.y;
  collision_objects[2].primitive_poses[0].position.z = marker_pose.position.z-0.1;
  collision_objects[2].primitive_poses[0].orientation.x = marker_pose.orientation.x;
  collision_objects[2].primitive_poses[0].orientation.y = marker_pose.orientation.y;
  collision_objects[2].primitive_poses[0].orientation.z = marker_pose.orientation.z;
  collision_objects[2].primitive_poses[0].orientation.w = marker_pose.orientation.w;


  collision_objects[2].operation = collision_objects[2].ADD;

  planning_scene_interface.applyCollisionObjects(collision_objects);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pick_place_demo");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::WallDuration(1.0).sleep();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface group("arm");
  group.setPlanningTime(45.0);

  tf::TransformListener listener;
  tf::StampedTransform transform;
  while (ros::ok()) {
    try{
      listener.lookupTransform("base_link", "ar_marker_0",ros::Time(0), transform);
      break;
    }
    catch (tf::TransformException ex){
      ROS_INFO("Waiting for transform between 'base_link' and 'ar_marker_0'");
      ros::Duration(1.0).sleep();
    }
  }
  ROS_INFO("Found transform between 'base_link' and 'ar_marker_0'");

  geometry_msgs::Pose marker_pose;

  marker_pose.position.x=transform.getOrigin().x();
  marker_pose.position.y=transform.getOrigin().y();
  marker_pose.position.z=transform.getOrigin().z();
  marker_pose.orientation.x=transform.getRotation().getX();
  marker_pose.orientation.y=transform.getRotation().getY();
  marker_pose.orientation.z=transform.getRotation().getZ();
  marker_pose.orientation.w=transform.getRotation().getW();

  addCollisionObjects(planning_scene_interface, marker_pose);


  ros::WallDuration(1.0).sleep();

  pick(group);

  ros::WallDuration(1.0).sleep();

  place(group);

  group.setNamedTarget("home");
  group.move();

  ros::WallDuration(1.0).sleep();

	ros::shutdown(); 
  return 0;
}