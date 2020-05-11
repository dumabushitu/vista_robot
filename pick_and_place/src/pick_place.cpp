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
  posture.points[0].positions[3] = 0.4;
  posture.points[0].positions[4] = 0;
  posture.points[0].positions[5] = 0;
  posture.points[0].time_from_start = ros::Duration(0.5);
}

void pick(moveit::planning_interface::MoveGroupInterface& move_group,const geometry_msgs::Pose &marker_pose)
{
  // 创建一个抓取向量，这里只创建一个抓取
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);

  // 设置抓取姿态

  tf2::Quaternion orientation;

  if(marker_pose.position.z>0.1){

    grasps[0].grasp_pose.header.frame_id = "ar_marker_1";
    orientation.setRPY(M_PI/2, 0, M_PI);
    grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
    grasps[0].grasp_pose.pose.position.x = 0;
    grasps[0].grasp_pose.pose.position.y = -0.15;
    grasps[0].grasp_pose.pose.position.z =-(marker_pose.position.z+0.05)/2.0;

    // 设置 pre-grasp approach
    grasps[0].pre_grasp_approach.direction.header.frame_id = "ar_marker_1";

    grasps[0].pre_grasp_approach.direction.vector.y = 1.0;
    grasps[0].pre_grasp_approach.min_distance = 0.095;
    grasps[0].pre_grasp_approach.desired_distance = 0.115;

  }
  else{
    grasps[0].grasp_pose.header.frame_id = "ar_marker_0";
    orientation.setRPY(M_PI, 0, 0);
    grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
    grasps[0].grasp_pose.pose.position.x = 0;
    grasps[0].grasp_pose.pose.position.y = 0;
    grasps[0].grasp_pose.pose.position.z = 0.13;

    // 设置 pre-grasp approach
    grasps[0].pre_grasp_approach.direction.header.frame_id = "base_link";

    grasps[0].pre_grasp_approach.direction.vector.z = -1.0;
    grasps[0].pre_grasp_approach.min_distance = 0.095;
    grasps[0].pre_grasp_approach.desired_distance = 0.115;

  }

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
  if(marker_pose.position.z>0.1){
    move_group.pick("object1", grasps);
  }
  else{
    move_group.pick("object0", grasps);
  }

}

void place(moveit::planning_interface::MoveGroupInterface& group,const geometry_msgs::Pose &marker_pose)
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
  if(marker_pose.position.z>0.1){
    place_location[0].place_pose.pose.position.x = -0.5;
    place_location[0].place_pose.pose.position.y = 0.1;
    place_location[0].place_pose.pose.position.z = (marker_pose.position.z+0.05)/2.0-0.05;
  }
  else{
    place_location[0].place_pose.pose.position.x = -0.47;
    place_location[0].place_pose.pose.position.y = -0.1;
    place_location[0].place_pose.pose.position.z = (marker_pose.position.z+0.05)/2.0-0.05;
  }

  // 设置pre-place approach

  place_location[0].pre_place_approach.direction.header.frame_id = "base_link";

  place_location[0].pre_place_approach.direction.vector.z = -1.0;
  place_location[0].pre_place_approach.min_distance = 0.095;
  place_location[0].pre_place_approach.desired_distance = 0.115;

  // 设置 post-grasp retreat
  place_location[0].post_place_retreat.direction.header.frame_id = "base_link";
  if(marker_pose.position.z>0.1){
    place_location[0].post_place_retreat.direction.vector.x = 1.0;
    place_location[0].post_place_retreat.min_distance = 0.1;
    place_location[0].post_place_retreat.desired_distance = 0.15;
  }
  else{
    place_location[0].post_place_retreat.direction.vector.z = 1.0;
    place_location[0].post_place_retreat.min_distance = 0.08;
    place_location[0].post_place_retreat.desired_distance = 0.15;
  }

  //设置放置对象完成后机械爪的状态
  openGripper(place_location[0].post_place_posture);

  // 设置支撑面table2.
  group.setSupportSurfaceName("table2");

  // 调用place将对象放置到给定的位置
  if(marker_pose.position.z>0.1){
    group.place("object1", place_location);
  }
  else{
    group.place("object0", place_location);
  }
}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,const geometry_msgs::Pose &marker0_pose,const geometry_msgs::Pose &marker1_pose)
{
  //先删除相关物体
  std::vector<std::string> object_ids;
  object_ids.push_back("object0");
  object_ids.push_back("object1");
  object_ids.push_back("table1");
  object_ids.push_back("table2");
  planning_scene_interface.removeCollisionObjects(object_ids);

  // 添加桌子和物块
  // 生成规划环境
  // 创建容纳2个碰撞对象的向量  
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(4);

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
  collision_objects[0].primitive_poses[0].position.y = 0.5;
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
  collision_objects[1].primitive_poses[0].position.x = -0.5;
  collision_objects[1].primitive_poses[0].position.y = 0;
  collision_objects[1].primitive_poses[0].position.z = -0.25;


  collision_objects[1].operation = collision_objects[1].ADD;

  // 定义将要抓取的物块0
  collision_objects[2].header.frame_id = "base_link";
  collision_objects[2].id = "object0";

  //定义物块0的形状和尺寸
  collision_objects[2].primitives.resize(1);
  collision_objects[2].primitives[0].type = collision_objects[2].primitives[0].BOX;
  collision_objects[2].primitives[0].dimensions.resize(3);
  collision_objects[2].primitives[0].dimensions[0] = 0.05;
  collision_objects[2].primitives[0].dimensions[1] = 0.05;
  collision_objects[2].primitives[0].dimensions[2] = marker0_pose.position.z+0.05;

  //定义物块0的位置
  collision_objects[2].primitive_poses.resize(1);
  collision_objects[2].primitive_poses[0].position.x = marker0_pose.position.x;
  collision_objects[2].primitive_poses[0].position.y = marker0_pose.position.y;
  collision_objects[2].primitive_poses[0].position.z = (marker0_pose.position.z+0.05)/2.0-0.05;
  collision_objects[2].primitive_poses[0].orientation.x = marker0_pose.orientation.x;
  collision_objects[2].primitive_poses[0].orientation.y = marker0_pose.orientation.y;
  collision_objects[2].primitive_poses[0].orientation.z = marker0_pose.orientation.z;
  collision_objects[2].primitive_poses[0].orientation.w = marker0_pose.orientation.w;


  collision_objects[2].operation = collision_objects[2].ADD;

    // 定义将要抓取的物块1
  collision_objects[3].header.frame_id = "base_link";
  collision_objects[3].id = "object1";

  //定义物块的形状和尺寸
  collision_objects[3].primitives.resize(1);
  collision_objects[3].primitives[0].type = collision_objects[3].primitives[0].BOX;
  collision_objects[3].primitives[0].dimensions.resize(3);
  collision_objects[3].primitives[0].dimensions[0] = 0.05;
  collision_objects[3].primitives[0].dimensions[1] = 0.05;
  collision_objects[3].primitives[0].dimensions[2] = marker1_pose.position.z+0.05;

  //定义物块的位置
  collision_objects[3].primitive_poses.resize(1);
  collision_objects[3].primitive_poses[0].position.x = marker1_pose.position.x;
  collision_objects[3].primitive_poses[0].position.y = marker1_pose.position.y;
  collision_objects[3].primitive_poses[0].position.z = (marker1_pose.position.z+0.05)/2.0-0.05;
  collision_objects[3].primitive_poses[0].orientation.x = marker1_pose.orientation.x;
  collision_objects[3].primitive_poses[0].orientation.y = marker1_pose.orientation.y;
  collision_objects[3].primitive_poses[0].orientation.z = marker1_pose.orientation.z;
  collision_objects[3].primitive_poses[0].orientation.w = marker1_pose.orientation.w;


  collision_objects[3].operation = collision_objects[3].ADD;

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
  tf::StampedTransform transform0;
  tf::StampedTransform transform1;
  while (ros::ok()) {
    try{
      listener.lookupTransform("base_link", "ar_marker_0",ros::Time(0), transform0);
      listener.waitForTransform("ar_marker_0", "tool0",ros::Time(0), ros::Duration(2));
      listener.lookupTransform("base_link", "ar_marker_1",ros::Time(0), transform1);
      listener.waitForTransform("ar_marker_1", "tool0",ros::Time(0), ros::Duration(2));

      break;
    }
    catch (tf::TransformException ex){
      ROS_INFO("Waiting for transform between 'base_link' and 'ar_marker_0'");
      ROS_INFO("Waiting for transform between 'ar_marker_0' and 'tool0'");
      ROS_INFO("Waiting for transform between 'base_link' and 'ar_marker_1'");
      ROS_INFO("Waiting for transform between 'ar_marker_1' and 'tool0'");
      ros::Duration(1.0).sleep();
    }
  }
  ROS_INFO("Found transform between 'base_link' and 'ar_marker_0'");
  ROS_INFO("Found transform between 'ar_marker_0' and 'tool0'");
  ROS_INFO("Found transform between 'base_link' and 'ar_marker_1'");
  ROS_INFO("Found transform between 'ar_marker_1' and 'tool0'");

  geometry_msgs::Pose marker0_pose;
  geometry_msgs::Pose marker1_pose;

  marker0_pose.position.x=transform0.getOrigin().x();
  marker0_pose.position.y=transform0.getOrigin().y();
  marker0_pose.position.z=transform0.getOrigin().z();
  marker0_pose.orientation.x=transform0.getRotation().getX();
  marker0_pose.orientation.y=transform0.getRotation().getY();
  marker0_pose.orientation.z=transform0.getRotation().getZ();
  marker0_pose.orientation.w=transform0.getRotation().getW();

  marker1_pose.position.x=transform1.getOrigin().x();
  marker1_pose.position.y=transform1.getOrigin().y();
  marker1_pose.position.z=transform1.getOrigin().z();
  marker1_pose.orientation.x=transform1.getRotation().getX();
  marker1_pose.orientation.y=transform1.getRotation().getY();
  marker1_pose.orientation.z=transform1.getRotation().getZ();
  marker1_pose.orientation.w=transform1.getRotation().getW();

  addCollisionObjects(planning_scene_interface,marker0_pose,marker1_pose);


  ros::WallDuration(1.0).sleep();

  pick(group,marker0_pose);

  ros::WallDuration(1.0).sleep();

  place(group,marker0_pose);

  ros::WallDuration(1.0).sleep();

  //group.setNamedTarget("up");
  //group.move();

  pick(group,marker1_pose);

  ros::WallDuration(1.0).sleep();

  place(group,marker1_pose);

  ros::WallDuration(1.0).sleep();

  group.setNamedTarget("home");
  group.move();

  ros::WallDuration(1.0).sleep();

	ros::shutdown(); 
  return 0;
}