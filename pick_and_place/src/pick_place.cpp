#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <visualization_msgs/Marker.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Pose.h>

class MarkerPose{
    public:
      geometry_msgs::Pose marker_pose;
      MarkerPose(){
          sub = nh.subscribe("aruco_single/marker", 1000, &MarkerPose::markerCallback,this);
      }
      void markerCallback(const visualization_msgs::Marker &msg)
      {
          marker_pose.position.x= msg.pose.position.x;
          marker_pose.position.y= msg.pose.position.y;
          marker_pose.position.z= msg.pose.position.z;

          marker_pose.orientation.x = msg.pose.orientation.x;
          marker_pose.orientation.y = msg.pose.orientation.y;
          marker_pose.orientation.z = msg.pose.orientation.z;
          marker_pose.orientation.w = msg.pose.orientation.w;
      }
    private:
      ros::NodeHandle nh;
      ros::Subscriber sub;

};


void prepare(moveit::planning_interface::MoveGroupInterface& move_group)
{
  move_group.setNamedTarget("prepare");
  move_group.move();
  ros::WallDuration(1.0).sleep();

}

void openGripper(moveit::planning_interface::MoveGroupInterface& group)
{
    group.setNamedTarget("open");
    group.move();
    ros::WallDuration(1.0).sleep(); 
}

void closedGripper(moveit::planning_interface::MoveGroupInterface& group)
{
    group.setNamedTarget("close");
    group.move();
    ros::WallDuration(1.0).sleep(); 
}

void pick(moveit::planning_interface::MoveGroupInterface& move_group, const MarkerPose &marker)
{
  // 创建一个抓取向量，这里只创建一个抓取
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);

  // 设置抓取姿态
  grasps[0].grasp_pose.header.frame_id = "base_link";
  tf2::Quaternion orientation;
  orientation.setRPY(0.430, 1.571, 2.001);
  grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);

  grasps[0].grasp_pose.pose.position.x = marker.marker_pose.position.x;
  grasps[0].grasp_pose.pose.position.y = marker.marker_pose.position.y;
  grasps[0].grasp_pose.pose.position.z = 0.3;

  // 设置 pre-grasp approach
  // ++++++++++++++++++++++++++
  grasps[0].pre_grasp_approach.direction.header.frame_id = "base_link";

  grasps[0].pre_grasp_approach.direction.vector.z = -1.0;
  grasps[0].pre_grasp_approach.min_distance = 0.05;
  grasps[0].pre_grasp_approach.desired_distance = 0.1;

  // 设置 post-grasp retreat
  // ++++++++++++++++++++++++++
  grasps[0].post_grasp_retreat.direction.header.frame_id = "base_link";

  grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
  grasps[0].post_grasp_retreat.min_distance = 0.1;
  grasps[0].post_grasp_retreat.desired_distance = 0.25;

  // 设置抓取前爪子的状态
  openGripper(move_group);

  // 设置爪子抓取的状态
  closedGripper(move_group);

  // 设置支撑面.
  move_group.setSupportSurfaceName("table2");
  // 调用pick使用给定的grasps抓取对象
  move_group.pick("cube_marker", grasps);
}

void place(moveit::planning_interface::MoveGroupInterface& group)
{
  // 创建一个尝试的向量，这里只创建一个放置位置
  std::vector<moveit_msgs::PlaceLocation> place_location;
  place_location.resize(1);

  // 设置放置的姿态
  place_location[0].place_pose.header.frame_id = "base_link";
  tf2::Quaternion orientation;
  orientation.setRPY(0.430, 1.571, 2.001);
  place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

  //放置对象
  place_location[0].place_pose.pose.position.x = 0;
  place_location[0].place_pose.pose.position.y = 0;
  place_location[0].place_pose.pose.position.z = 0.05;

  // S设置pre-place approach
  place_location[0].pre_place_approach.direction.header.frame_id = "base_link";

  place_location[0].pre_place_approach.direction.vector.z = -1.0;
  place_location[0].pre_place_approach.min_distance = 0.095;
  place_location[0].pre_place_approach.desired_distance = 0.115;

  // 设置 post-grasp retreat

  place_location[0].post_place_retreat.direction.header.frame_id = "base_link";

  place_location[0].post_place_retreat.direction.vector.z = 1.0;
  place_location[0].post_place_retreat.min_distance = 0.1;
  place_location[0].post_place_retreat.desired_distance = 0.28;

  //设置放置对象完成前机械爪的状态
  openGripper(group);

  // 设置支撑面table2.
  group.setSupportSurfaceName("table1");
  // 调用place将对象放置到给定的位置
  group.place("cube_marker", place_location);
}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene, const MarkerPose &marker)
{

  // 添加桌子和物块
  //
  // 生成规划环境
  // ^^^^^^^^^^^^^^^^^^^^
  // 创建容纳3个碰撞对象的向量
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(3);

  // 添加第一个放置ur机械臂的桌子.
  collision_objects[0].id = "table1";
  collision_objects[0].header.frame_id = "world";

  //定义桌子的形状和尺寸
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 1.5;
  collision_objects[0].primitives[0].dimensions[1] = 0.8;
  collision_objects[0].primitives[0].dimensions[2] = 0.03;

  //定义桌子的位置
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0;
  collision_objects[0].primitive_poses[0].position.y = 0;
  collision_objects[0].primitive_poses[0].position.z = 1.0;


  collision_objects[0].operation = collision_objects[0].ADD;

  // 添加第二个放置物块的桌子
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
  collision_objects[1].primitive_poses[0].position.x = 0;
  collision_objects[1].primitive_poses[0].position.y = 0.8;
  collision_objects[1].primitive_poses[0].position.z = 1.0;

  collision_objects[1].operation = collision_objects[1].ADD;

  // 定义将要抓取的物块
  collision_objects[2].header.frame_id = "base_link";
  collision_objects[2].id = "cube_marker";

  //定义物块的形状和尺寸
  collision_objects[2].primitives.resize(1);
  collision_objects[2].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[2].primitives[0].dimensions.resize(3);
  collision_objects[2].primitives[0].dimensions[0] = 0.05;
  collision_objects[2].primitives[0].dimensions[1] = 0.05;
  collision_objects[2].primitives[0].dimensions[2] = 0.1;

  //定义物块的位置
  collision_objects[2].primitive_poses.resize(1);
  collision_objects[2].primitive_poses[0].position.x = marker.marker_pose.position.x;
  collision_objects[2].primitive_poses[0].position.y = marker.marker_pose.position.y;
  collision_objects[2].primitive_poses[0].position.z = 0.05;
  collision_objects[2].primitive_poses[0].orientation.x = marker.marker_pose.orientation.x;
  collision_objects[2].primitive_poses[0].orientation.y = marker.marker_pose.orientation.y;
  collision_objects[2].primitive_poses[0].orientation.z = marker.marker_pose.orientation.z;
  collision_objects[2].primitive_poses[0].orientation.w = marker.marker_pose.orientation.w;


  collision_objects[2].operation = collision_objects[2].ADD;

  planning_scene.applyCollisionObjects(collision_objects);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vistar_pick_place");
  ros::AsyncSpinner spinner(2);
  spinner.start();
  MarkerPose marker;

  moveit::planning_interface::PlanningSceneInterface planning_scene;
  moveit::planning_interface::MoveGroupInterface arm("arm");
  moveit::planning_interface::MoveGroupInterface gripper("gripper");
  arm.setPlanningTime(45.0);

  prepare(arm);

  addCollisionObjects(planning_scene,marker);

  // 等待场景初始化
  ros::WallDuration(1.0).sleep();

  pick(arm,marker);

  ros::WallDuration(1.0).sleep();

  place(arm);

  ros::waitForShutdown();
  return 0;
}