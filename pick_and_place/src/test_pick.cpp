#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


void prepare(moveit::planning_interface::MoveGroupInterface& group)
{
      if(group.setNamedTarget ("home"))

      {

        ROS_INFO("arm setNamedTarget Success!");

        group.move();

      }

    else

      ROS_INFO("setNamedTarget failed!");

}

void openGripper(trajectory_msgs::JointTrajectory& posture)
{
  posture.joint_names.resize(1);
  posture.joint_names[0] = "gripper_right_driver_joint";

  posture.points.resize(1);
  posture.points[0].positions.resize(1);
  posture.points[0].positions[0] = 0.04;
  posture.points[0].time_from_start = ros::Duration(0.5);
}

void closedGripper(trajectory_msgs::JointTrajectory& posture)
{
  posture.joint_names.resize(1);
  posture.joint_names[0] = "gripper_right_driver_joint";

  posture.points.resize(1);
  posture.points[0].positions.resize(1);
  posture.points[0].positions[0] = 0.35;
  posture.points[0].time_from_start = ros::Duration(0.5);

}

void pickit(moveit::planning_interface::MoveGroupInterface& group)
{
  // 创建一个抓取向量，这里只创建一个抓取
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);

  tf::TransformListener listener;
  while (ros::ok()) {
    try{
      listener.waitForTransform("ar_marker_0", "ee_link",ros::Time(0), ros::Duration(3));
      break;
    }
    catch (tf::TransformException ex){
      ROS_INFO("Waiting for transform between 'ar_marker_0' and 'ee_link'");
      ros::Duration(1.0).sleep();
    }
  }
ROS_INFO("Found transform between  'ar_marker_0' and 'ee_link'");

  // 设置抓取姿态
  grasps[0].grasp_pose.header.frame_id = "ar_marker_0";
  //tf2::Quaternion orientation;
  //orientation.setRPY(M_PI, 0, M_PI/2);
  //grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
  grasps[0].grasp_pose.pose.position.x = 0;
  grasps[0].grasp_pose.pose.position.y = 0;
  grasps[0].grasp_pose.pose.position.z = 0.13;
  grasps[0].grasp_pose.pose.orientation.x = -0.5;
  grasps[0].grasp_pose.pose.orientation.y = 0.5;
  grasps[0].grasp_pose.pose.orientation.z = 0.5;
  grasps[0].grasp_pose.pose.orientation.w = 0.5;


  // 设置 pre-grasp approach
  grasps[0].pre_grasp_approach.direction.header.frame_id = "base_link";

  grasps[0].pre_grasp_approach.direction.vector.z = -1.0;
  grasps[0].pre_grasp_approach.min_distance = 0.05;
  grasps[0].pre_grasp_approach.desired_distance = 0.1;

  // 设置 post-grasp retreat
  grasps[0].post_grasp_retreat.direction.header.frame_id = "base_link";

  grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
  grasps[0].post_grasp_retreat.min_distance = 0.1;
  grasps[0].post_grasp_retreat.desired_distance = 0.15;

  // 设置抓取前爪子的状态
  openGripper(grasps[0].pre_grasp_posture);

  // 设置爪子抓取的状态
  closedGripper(grasps[0].grasp_posture);

  // 设置支撑面.
  group.setSupportSurfaceName("table1");
  // 调用pick使用给定的grasps抓取对象
  group.pick("cube_marker", grasps);


}

void placeit(moveit::planning_interface::MoveGroupInterface& group)
{
  // 创建一个尝试的向量，这里只创建一个放置位置
  std::vector<moveit_msgs::PlaceLocation> place_location;
  place_location.resize(1);

  // 设置放置的姿态
  place_location[0].place_pose.header.frame_id = "base_link";
  //tf2::Quaternion orientation;
  //orientation.setRPY(M_PI, 0, M_PI);
  //place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation); 

  //放置对象
  place_location[0].place_pose.pose.position.x = 0;
  place_location[0].place_pose.pose.position.y = 0.5;
  place_location[0].place_pose.pose.position.z = 0.065;
  place_location[0].place_pose.pose.orientation.x = -0.707;
  place_location[0].place_pose.pose.orientation.y = 0;
  place_location[0].place_pose.pose.orientation.z = 0.707;
  place_location[0].place_pose.pose.orientation.w = 0;

  // S设置pre-place approach
  place_location[0].pre_place_approach.direction.header.frame_id = "base_link";

  place_location[0].pre_place_approach.direction.vector.z = -1.0;
  place_location[0].pre_place_approach.min_distance = 0.05;
  place_location[0].pre_place_approach.desired_distance = 0.1;

  // 设置 post-grasp retreat

  place_location[0].post_place_retreat.direction.header.frame_id = "base_link";

  place_location[0].post_place_retreat.direction.vector.z = 1.0;
  place_location[0].post_place_retreat.min_distance = 0.1;
  place_location[0].post_place_retreat.desired_distance = 0.15;

  //设置放置对象完成前机械爪的状态
  openGripper(place_location[0].post_place_posture);

  // 设置支撑面table2.
  group.setSupportSurfaceName("table2");
  // 调用place将对象放置到给定的位置
  group.place("cube_marker", place_location);
}


void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene)
{
  //先删除相关物体
  std::vector<std::string> object_ids;
  object_ids.push_back("cube_marker");
  object_ids.push_back("table1");
  object_ids.push_back("table2");
  planning_scene.removeCollisionObjects(object_ids);
  // 添加桌子和物块
  // 生成规划环境
  // 创建容纳2个碰撞对象的向量
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(3);

  // 添加第一个放置ur机械臂的桌子.
  collision_objects[0].id = "table1";
  collision_objects[0].header.frame_id = "world";

  //定义桌子1的形状和尺寸
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.8;
  collision_objects[0].primitives[0].dimensions[1] = 1.5;
  collision_objects[0].primitives[0].dimensions[2] = 0.03;

  //定义桌子1的位置
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0;
  collision_objects[0].primitive_poses[0].position.y = 1.05;
  collision_objects[0].primitive_poses[0].position.z = 1.0;


  collision_objects[0].operation = collision_objects[0].ADD;

  //添加第二张桌子
  collision_objects[1].id = "table2";
  collision_objects[1].header.frame_id = "world";

  //定义桌子的形状和尺寸
  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 1.5;
  collision_objects[1].primitives[0].dimensions[1] = 0.8;
  collision_objects[1].primitives[0].dimensions[2] = 0.03;

  //定义桌子的位置
  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = -1.05;
  collision_objects[1].primitive_poses[0].position.y = -0.1;
  collision_objects[1].primitive_poses[0].position.z = 1.0;


  collision_objects[1].operation = collision_objects[1].ADD;


  // 定义将要抓取的物块
  collision_objects[2].header.frame_id = "base_link";
  collision_objects[2].id = "cube_marker";

  //定义物块的形状和尺寸
  collision_objects[2].primitives.resize(1);
  collision_objects[2].primitives[0].type = collision_objects[2].primitives[0].BOX;
  collision_objects[2].primitives[0].dimensions.resize(3);
  collision_objects[2].primitives[0].dimensions[0] = 0.05;
  collision_objects[2].primitives[0].dimensions[1] = 0.05;
  collision_objects[2].primitives[0].dimensions[2] = 0.1;

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

  //定义物块的位置
  collision_objects[2].primitive_poses.resize(1);
  collision_objects[2].primitive_poses[0].position.x = transform.getOrigin().x();
  collision_objects[2].primitive_poses[0].position.y = transform.getOrigin().y();
  collision_objects[2].primitive_poses[0].position.z = 0.015;
  collision_objects[2].primitive_poses[0].orientation.x = transform.getRotation().getX();
  collision_objects[2].primitive_poses[0].orientation.y = transform.getRotation().getY();
  collision_objects[2].primitive_poses[0].orientation.z = transform.getRotation().getZ();
  collision_objects[2].primitive_poses[0].orientation.w = transform.getRotation().getW();


  collision_objects[2].operation = collision_objects[2].ADD;

  planning_scene.applyCollisionObjects(collision_objects);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vistar_pick_place");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::WallDuration(1.0).sleep();
  moveit::planning_interface::PlanningSceneInterface planning_scene;
  moveit::planning_interface::MoveGroupInterface arm("arm");
  moveit::planning_interface::MoveGroupInterface gripper("gripper");
  arm.setPlanningTime(45.0);

  addCollisionObjects(planning_scene);
  // 等待场景初始化
  ros::WallDuration(1.0).sleep();

  prepare(arm); 
  ros::WallDuration(1.0).sleep();

  pickit(arm);

  ros::WallDuration(1.0).sleep();

  placeit(arm);

  ros::waitForShutdown();
  return 0;
}