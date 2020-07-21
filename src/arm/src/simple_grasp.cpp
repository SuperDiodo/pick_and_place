// ROS
#include <ros/ros.h>

// MoveIt!
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define NUM_BOX 5
#define TABLE_HEIGHT 0.3
#define TABLE_LENGHT 1
#define TABLE_WIDTH 0.5
#define SPACING (TABLE_LENGHT - 0.1) / NUM_BOX

double fRand(double fMin, double fMax)
{
  double f = (double)rand() / RAND_MAX;
  return fMin + f * (fMax - fMin);
}

std::vector<geometry_msgs::Pose> add_box(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface, moveit::planning_interface::MoveGroupInterface &group)
{

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(NUM_BOX);
  std::vector<geometry_msgs::Pose> poses;
  srandom(time(NULL));

  for (int i = 0; i < NUM_BOX; i++)
  {

    collision_objects[i].header.frame_id = group.getPlanningFrame();

    /* The id of the object is used to identify it. */
    collision_objects[i].id = "box" + std::to_string(i);

    /* Define a box to add to the world. */
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.05;
    primitive.dimensions[1] = 0.05;
    primitive.dimensions[2] = 0.05;

    /* A pose for the box (specified relative to frame_id) */
    geometry_msgs::Pose box_pose;

    box_pose.orientation.w = 1.0;
    box_pose.position.x = fRand(0.1, 0.6);
    box_pose.position.y = fRand(-0.5, 0.5);
    box_pose.position.z = 0;

    poses.push_back(box_pose);

    collision_objects[i].primitives.push_back(primitive);
    collision_objects[i].primitive_poses.push_back(box_pose);
    collision_objects[i].operation = collision_objects[i].ADD;
  }

  planning_scene_interface.addCollisionObjects(collision_objects);
  return poses;
}

geometry_msgs::Pose add_table(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface, moveit::planning_interface::MoveGroupInterface &group)
{

  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = group.getPlanningFrame();

  /* The id of the object is used to identify it. */
  collision_object.id = "table";

  /* Define a box to add to the world. */
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = TABLE_WIDTH;
  primitive.dimensions[1] = TABLE_LENGHT;
  primitive.dimensions[2] = TABLE_HEIGHT;

  /* A pose for the box (specified relative to frame_id) */
  geometry_msgs::Pose table_pose;

  table_pose.orientation.w = 1.0;
  table_pose.position.x = -0.5;
  table_pose.position.y = 0;
  table_pose.position.z = primitive.dimensions[2] / 2;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(table_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);
  planning_scene_interface.addCollisionObjects(collision_objects);

  return table_pose;
}

void openGripper(trajectory_msgs::JointTrajectory &posture)
{
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.04;
  posture.points[0].positions[1] = 0.04;
  posture.points[0].time_from_start = ros::Duration(0.5);
}

void closedGripper(trajectory_msgs::JointTrajectory &posture)
{
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.02;
  posture.points[0].positions[1] = 0.02;
  posture.points[0].time_from_start = ros::Duration(0.5);
}

void pick(moveit::planning_interface::MoveGroupInterface &move_group, geometry_msgs::Pose target_pose, int index)
{
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);

  grasps[0].grasp_pose.header.frame_id = "panda_link0";
  tf2::Quaternion orientation;
  orientation.setRPY(M_PI, 0, M_PI / 4);
  grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
  grasps[0].grasp_pose.pose.position.x = target_pose.position.x;
  grasps[0].grasp_pose.pose.position.y = target_pose.position.y;
  grasps[0].grasp_pose.pose.position.z = target_pose.position.z + 0.1;

  grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0";
  grasps[0].pre_grasp_approach.direction.vector.z = -1.0;
  grasps[0].pre_grasp_approach.min_distance = 0.095;
  grasps[0].pre_grasp_approach.desired_distance = 0.115;

  grasps[0].post_grasp_retreat.direction.header.frame_id = "panda_link0";
  grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
  grasps[0].post_grasp_retreat.min_distance = 0.1;
  grasps[0].post_grasp_retreat.desired_distance = 0.25;

  openGripper(grasps[0].pre_grasp_posture);
  closedGripper(grasps[0].grasp_posture);
  move_group.pick("box" + std::to_string(index), grasps);
}

void place(moveit::planning_interface::MoveGroupInterface &group, geometry_msgs::Pose table_pose, int index)
{

  std::vector<moveit_msgs::PlaceLocation> place_location;
  place_location.resize(1);

  place_location[0].place_pose.header.frame_id = "panda_link0";

  tf2::Quaternion orientation;
  orientation.setRPY(0, 0, 0);
  place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

  place_location[0].pre_place_approach.direction.header.frame_id = "panda_link0";
  place_location[0].pre_place_approach.direction.vector.z = -1.0;
  place_location[0].pre_place_approach.min_distance = 0.12;
  place_location[0].pre_place_approach.desired_distance = 0.2;

  place_location[0].post_place_retreat.direction.header.frame_id = "panda_link0";
  place_location[0].post_place_retreat.direction.vector.z = 1.0;
  place_location[0].post_place_retreat.min_distance = 0.1;
  place_location[0].post_place_retreat.desired_distance = 0.25;

  bool success;

  do
  {
    place_location[0].place_pose.pose.position.x = fRand(-0.5 + TABLE_WIDTH / 2, -0.5 - TABLE_WIDTH / 2);
    place_location[0].place_pose.pose.position.y = fRand(-0.5, 0.5);
    place_location[0].place_pose.pose.position.z = table_pose.position.z + 0.15 + 0.025;
    std::cout << "Random place location estimated: " << std::endl;
    std::cout << "X: " << place_location[0].place_pose.pose.position.x << std::endl;
    std::cout << "Y: " << place_location[0].place_pose.pose.position.y << std::endl;
    std::cout << "Z: " << place_location[0].place_pose.pose.position.z << std::endl;

    openGripper(place_location[0].post_place_posture);
    group.setSupportSurfaceName("table");
    success = group.place("box" + std::to_string(index), place_location) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
  } while (!success);
}

int main(int argc, char **argv)
{

  srandom(time(NULL));
  ros::init(argc, argv, "arm_pick_and_place");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface group("arm");

  std::vector<geometry_msgs::Pose> poses = add_box(planning_scene_interface, group);
  geometry_msgs::Pose table_pose = add_table(planning_scene_interface, group);

  group.setPlanningTime(10.0);

  int i = 0;
  for (auto pose : poses)
  {
    pick(group, pose, i);
    place(group, table_pose, i);
    i++;
  }

  group.setNamedTarget("home");
  group.move();

  ros::waitForShutdown();
  return 0;
}