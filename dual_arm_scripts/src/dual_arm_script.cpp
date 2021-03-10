#include "ros/ros.h"
#include <vector>
#include <string>
#include "geometry_msgs/PoseStamped.h"

#include <tf/transform_listener.h>    // Includes the TF conversions

#include <thread>
#include <cmath>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit/collision_detection/collision_matrix.h>

const double tau = 6.283185307179586476925;

moveit_msgs::CollisionObject makeCollisionObject()
{
  moveit_msgs::CollisionObject screw_tool_m4;
  screw_tool_m4.header.frame_id = "screw_tool_m4_link";
  screw_tool_m4.id = "screw_tool_m4";

  screw_tool_m4.primitives.resize(3);
  screw_tool_m4.primitive_poses.resize(3);
  // The bit cushion and motor
  screw_tool_m4.primitives[0].type = screw_tool_m4.primitives[0].BOX;
  screw_tool_m4.primitives[0].dimensions.resize(3);
  screw_tool_m4.primitives[0].dimensions[0] = 0.026;
  screw_tool_m4.primitives[0].dimensions[1] = 0.04;
  screw_tool_m4.primitives[0].dimensions[2] = 0.055;
  screw_tool_m4.primitive_poses[0].position.x = 0;
  screw_tool_m4.primitive_poses[0].position.y = -0.009;
  screw_tool_m4.primitive_poses[0].position.z = 0.0275;

  // The "shaft" + suction attachment
  screw_tool_m4.primitives[1].type = screw_tool_m4.primitives[1].BOX;
  screw_tool_m4.primitives[1].dimensions.resize(3);
  screw_tool_m4.primitives[1].dimensions[0] = 0.02;
  screw_tool_m4.primitives[1].dimensions[1] = 0.03;
  screw_tool_m4.primitives[1].dimensions[2] = 0.08;
  screw_tool_m4.primitive_poses[1].position.x = 0;
  screw_tool_m4.primitive_poses[1].position.y = -0.0055;  // 21 mm distance from axis
  screw_tool_m4.primitive_poses[1].position.z = -0.04;

  // The cylinder representing the tip
  screw_tool_m4.primitives[2].type = screw_tool_m4.primitives[2].CYLINDER;
  screw_tool_m4.primitives[2].dimensions.resize(2);
  screw_tool_m4.primitives[2].dimensions[0] = 0.038;    // Cylinder height
  screw_tool_m4.primitives[2].dimensions[1] = 0.0035;   // Cylinder radius
  screw_tool_m4.primitive_poses[2].position.x = 0;
  screw_tool_m4.primitive_poses[2].position.y = 0;  // 21 mm distance from axis
  screw_tool_m4.primitive_poses[2].position.z = -0.099;
  screw_tool_m4.operation = screw_tool_m4.ADD;

  // The tool tip
  screw_tool_m4.subframe_poses.resize(1);
  screw_tool_m4.subframe_names.resize(1);
  screw_tool_m4.subframe_poses[0].position.z = -.12;
  screw_tool_m4.subframe_poses[0].orientation = tf::createQuaternionMsgFromRollPitchYaw(0, tau/4, -tau/4);
  screw_tool_m4.subframe_names[0] = "screw_tool_m4_tip";
  return screw_tool_m4;
}

bool spawnTool(std::string screw_tool_id, moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(1);

  collision_objects[0] = screw_tool_m4;
  collision_objects[0].operation = collision_objects[0].ADD;

  planning_scene_interface.applyCollisionObjects(collision_objects);

  return true;
}

// Remove the tool from the scene so it does not cause unnecessary collision calculations
bool despawnTool(std::string screw_tool_id, moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(1);

    collision_objects[0].id = screw_tool_id;
    collision_objects[0].operation = collision_objects[0].REMOVE;
    planning_scene_interface.applyCollisionObjects(collision_objects);

    return true;
}

bool attachDetachTool(std::string screw_tool_id, std::string robot_name, std::string attach_or_detach, moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
  moveit_msgs::AttachedCollisionObject att_coll_object;

  if (screw_tool_id == "screw_tool_m4") att_coll_object.object = makeCollisionObject();
  else { ROS_WARN_STREAM("No screw tool specified to " << attach_or_detach); }

  att_coll_object.link_name = robot_name + "_robotiq_85_tip_link";

  if (attach_or_detach == "attach") att_coll_object.object.operation = att_coll_object.object.ADD;
  else if (attach_or_detach == "detach") att_coll_object.object.operation = att_coll_object.object.REMOVE;
  
  ROS_INFO_STREAM(attach_or_detach << "ing tool " << screw_tool_id);
  planning_scene_interface.applyAttachedCollisionObject(att_coll_object);
  return true;
}

bool equipUnequipTool(std::string robot_name, std::string screw_tool_id, std::string equip_or_unequip, 
                moveit::planning_interface::MoveGroupInterface& group, moveit::planning_interface::PlanningSceneInterface& psi)
{
  bool equip = (equip_or_unequip == "equip");
  bool unequip = (equip_or_unequip == "unequip");

  // spawnTool("screw_tool_m4", psi);

  group.clearPoseTargets();
  group.setStartStateToCurrentState();
  group.setNamedTarget("home");
  group.move();
  
  geometry_msgs::PoseStamped ps_tool_holder;
  ps_tool_holder.header.frame_id = screw_tool_id + "_pickup_link";
  ps_tool_holder.pose.position.x = -.06;
  ps_tool_holder.pose.position.z = -.008;
  ps_tool_holder.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, -tau/12, 0);
  ps_tool_holder.pose.position.x = 0.017;

  ROS_INFO("Moving to pose in tool holder.");

  group.clearPoseTargets();
  group.setStartStateToCurrentState();
  group.setPoseTarget(ps_tool_holder);
  group.setEndEffectorLink(robot_name + "_robotiq_85_tip_link");
  group.move();
  
  if (equip)
  {
    // closeGripper(robot_name);
    attachDetachTool(screw_tool_id, robot_name, "attach", psi);
  }
  else if (unequip) 
  {
    // openGripper(robot_name);
    attachDetachTool(screw_tool_id, robot_name, "detach", psi);
  }
  
  ROS_INFO("Moving back.");
  group.clearPoseTargets();
  group.setStartStateToCurrentState();
  group.setNamedTarget("home");
  group.move();
  
  // despawnTool("screw_tool_m4", psi);

  return true;
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "test");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface a_bot_group("a_bot");
  // moveit::planning_interface::MoveGroupInterface b_bot_group("b_bot");
  moveit::planning_interface::PlanningSceneInterface psi;

  equipUnequipTool("a_bot", "screw_tool_m4", "equip", a_bot_group, psi);
  equipUnequipTool("a_bot", "screw_tool_m4", "unequip", a_bot_group, psi);
  // equipUnequipTool("b_bot", "screw_tool_m4", "equip", b_bot_group, psi);
  // equipUnequipTool("b_bot", "screw_tool_m4", "unequip", b_bot_group, psi);

  return 1;
}



