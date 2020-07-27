#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseArray.h>
#include <unistd.h>

double fRand(double fMin, double fMax)
{
  double f = (double)rand() / RAND_MAX;
  return fMin + f * (fMax - fMin);
}

class cr35ia
{
public:
  cr35ia(std::string planning_group)
  {
    move_group = new moveit::planning_interface::MoveGroupInterface(planning_group);
    move_group->setPlanningTime(45.0);
    detected = false;
  }

  void generate_object()
  {

    ros::NodeHandle ps;
    ros::Subscriber pose_sub = ps.subscribe("/poses", 1, &cr35ia::getPoses, this);

    while(detected == false){
      std::cout << "No pose detected" << std::endl;
      usleep(500000);
    }

    ps.shutdown();

    for (auto &pose : detected_poses_array.poses)
    {
      geometry_msgs::Pose target_pose;
      target_pose.position.x = pose.position.x + 1;
      target_pose.position.y = pose.position.y;
      target_pose.position.z = 0;

      tf2::Quaternion orientation;
      orientation.setRPY(0, 0, fRand(-3.14, 3.14));
      target_pose.orientation = tf2::toMsg(orientation);

      targets.push_back(target_pose);
    }

    std::cout << "Detected pose: " << std::endl; 
    for(auto& pose: targets){
      std::cout << "X: " << pose.position.x << std::endl;
      std::cout << "Y: " << pose.position.y << std::endl;
    }

    for (int i = 0; i < targets.size(); i++)
    {

      moveit_msgs::CollisionObject collision_object;
      collision_object.header.frame_id = move_group->getPlanningFrame();

      /* The id of the object is used to identify it. */
      collision_object.id = std::to_string(i);

      /* Define a box to add to the world. */
      shape_msgs::SolidPrimitive primitive;
      primitive.type = primitive.BOX;
      primitive.dimensions.resize(3);
      primitive.dimensions[0] = 0.1;
      primitive.dimensions[1] = 0.1;
      primitive.dimensions[2] = 0.1;

      /* A pose for the box (specified relative to frame_id) */
      geometry_msgs::Pose box_pose;

      box_pose.orientation.w = 1.0;
      box_pose.position.x = targets[i].position.x;
      box_pose.position.y = targets[i].position.y;
      box_pose.position.z = 0;

      box_pose.orientation = targets[i].orientation;

      collision_object.primitives.push_back(primitive);
      collision_object.primitive_poses.push_back(box_pose);
      collision_object.operation = collision_object.ADD;

      collision_objects.push_back(collision_object);
    }

    planning_scene_interface.addCollisionObjects(collision_objects);
  }

  void pick_and_place()
  {

    for (int i = 0; i < targets.size(); i++)
    {
      pick(i);
      place(i, false);
    }
  }

  void pick(int i)
  {

    std::vector<moveit_msgs::Grasp> grasps;

    grasp.grasp_pose.header.frame_id = "base_link";

    grasp.grasp_pose.pose.position.x = targets[i].position.x;
    grasp.grasp_pose.pose.position.y = targets[i].position.y;
    grasp.grasp_pose.pose.position.z = targets[i].position.z + 0.2;

    grasp.pre_grasp_approach.direction.header.frame_id = "base_link";
    grasp.pre_grasp_approach.direction.vector.z = -1.0;
    grasp.pre_grasp_approach.min_distance = 0.1;
    grasp.pre_grasp_approach.desired_distance = 0.12;

    grasp.post_grasp_retreat.direction.header.frame_id = "base_link";
    grasp.post_grasp_retreat.direction.vector.z = 1.0;
    grasp.post_grasp_retreat.min_distance = 0.2;
    grasp.post_grasp_retreat.desired_distance = 0.3;

    openGripper(false);
    closedGripper(false);

    tf2::Quaternion orientation;
    tf2::fromMsg(targets[i].orientation, orientation);

    /* set correct angles for each target pose */
    double roll, pitch, yaw;
    tf2::Matrix3x3 matrix_orientation(orientation);
    matrix_orientation.getRPY(roll, pitch, yaw);
    orientation.setRPY(roll, pitch + M_PI / 2, yaw + M_PI / 2);

    grasp.grasp_pose.pose.orientation = tf2::toMsg(orientation);
    grasps.push_back(grasp);
    move_group->pick(std::to_string(i), grasps);
  }

  void place(int i, bool return_start)
  {
    std::vector<moveit_msgs::PlaceLocation> places;
    place_location.place_pose.header.frame_id = "base_link";

    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, 0);
    place_location.place_pose.pose.orientation = tf2::toMsg(orientation);

    place_location.pre_place_approach.direction.header.frame_id = "base_link";
    place_location.pre_place_approach.direction.vector.z = -1.0;
    place_location.pre_place_approach.min_distance = 0.10;
    place_location.pre_place_approach.desired_distance = 0.2;

    place_location.post_place_retreat.direction.header.frame_id = "base_link";
    place_location.post_place_retreat.direction.vector.z = 1.0;
    place_location.post_place_retreat.min_distance = 0.1;
    place_location.post_place_retreat.desired_distance = 0.25;

    /* generate different poses for placing */
    for (int i = 0; i < 10; i++)
    {
      place_location.place_pose.pose.position.x = fRand(1.6, 0.5);
      place_location.place_pose.pose.position.y = fRand(-1.5, 1.5);
      place_location.place_pose.pose.position.z = 0;
      openGripper(true);
      places.push_back(place_location);
    }

    std::cout << "number of estimated place poses: " << places.size() << std::endl;
    move_group->place(std::to_string(i), places);

    if (return_start)
    {
      move_group->setNamedTarget("start");
      move_group->move();
    }
  }

private:
  geometry_msgs::Pose target_pose, table_pose;
  moveit_msgs::Grasp grasp;
  moveit_msgs::PlaceLocation place_location;
  trajectory_msgs::JointTrajectory trajectory;
  moveit::planning_interface::MoveGroupInterface *move_group;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  geometry_msgs::PoseArray detected_poses_array;
  std::vector<geometry_msgs::Pose> targets;
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  bool detected;

  void getPoses(const geometry_msgs::PoseArray &msgs)
  {
    detected_poses_array = msgs;
    detected = true;
  }

  /* 0.0 open */
  void openGripper(bool pick)
  {
    if (!pick)
    {
      grasp.pre_grasp_posture.joint_names.resize(1);
      grasp.pre_grasp_posture.joint_names[0] = "finger_joint";

      grasp.pre_grasp_posture.points.resize(1);
      grasp.pre_grasp_posture.points[0].positions.resize(1);
      grasp.pre_grasp_posture.points[0].positions[0] = 0.0;
      grasp.pre_grasp_posture.points[0].time_from_start = ros::Duration(0.5);
    }
    else
    {
      place_location.post_place_posture.joint_names.resize(1);
      place_location.post_place_posture.joint_names[0] = "finger_joint";

      place_location.post_place_posture.points.resize(1);
      place_location.post_place_posture.points[0].positions.resize(1);
      place_location.post_place_posture.points[0].positions[0] = 0.0;
      place_location.post_place_posture.points[0].time_from_start = ros::Duration(0.5);
    }
  }

  /* 0.7 collision, 0.699 closed */
  void closedGripper(bool pick)
  {
    if (!pick)
    {
      grasp.grasp_posture.joint_names.resize(1);
      grasp.grasp_posture.joint_names[0] = "finger_joint";
      grasp.grasp_posture.points.resize(1);
      grasp.grasp_posture.points[0].positions.resize(1);
      grasp.grasp_posture.points[0].positions[0] = 0.3;
      grasp.grasp_posture.points[0].time_from_start = ros::Duration(0.5);
    }
    else
    {
      place_location.post_place_posture.joint_names.resize(1);
      place_location.post_place_posture.joint_names[0] = "finger_joint";
      place_location.post_place_posture.points.resize(1);
      place_location.post_place_posture.points[0].positions.resize(1);
      place_location.post_place_posture.points[0].positions[0] = 0.699;
      place_location.post_place_posture.points[0].time_from_start = ros::Duration(0.5);
    }
  }
};

int main(int argc, char **argv)
{
  srandom(time(NULL));
  ros::init(argc, argv, "arm_pick_and_place");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  cr35ia arm("arm");
  arm.generate_object(); // it also set the target
  arm.pick_and_place();

  ros::waitForShutdown();
  return 0;
}