#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseArray.h>
#include <gazebo_msgs/ModelStates.h>
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
    detected_poses = false;
    detected_model = false;
  }

  void generate_scene()
  {
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group->getPlanningFrame();

    /* ORIGIN TABLE */
    collision_object.id = "box_surface";

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 2;
    primitive.dimensions[1] = 5;
    primitive.dimensions[2] = 0.1;

    geometry_msgs::Pose box_surface;

    box_surface.orientation.w = 1.0;
    box_surface.position.x = 1.5;
    box_surface.position.y = 0;
    box_surface.position.z = 0.8;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_surface);
    collision_object.operation = collision_object.ADD;

    collision_objects.push_back(collision_object);

    /* DESTINATION TABLE */
    collision_object.id = "place_location";

    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 2;
    primitive.dimensions[1] = 2.5;
    primitive.dimensions[2] = 0.1;

    geometry_msgs::Pose place_location;

    place_location.orientation.w = 1.0;
    place_location.position.x = -1.5;
    place_location.position.y = 0;
    place_location.position.z = 0.8;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(place_location);
    collision_object.operation = collision_object.ADD;

    collision_objects.push_back(collision_object);
    planning_scene_interface.addCollisionObjects(collision_objects);
  }

  void generate_object()
  {

    ros::NodeHandle ps;
    ros::Subscriber pose_sub = ps.subscribe("/gazebo/model_states", 1, &cr35ia::getModelStates, this);

    /* wait for a pose */
    while (detected_model == false)
    {
      std::cout << "No pose detected" << std::endl;
      usleep(500000);
    }

    ps.shutdown();

    std::cout << "Model poses: " << std::endl;
    for (auto &pose : model_poses)
    {
      std::cout << "X: " << pose.position.x << std::endl;
      std::cout << "Y: " << pose.position.y << std::endl;
      std::cout << "Z: " << pose.position.z << std::endl;
      std::cout << "Orientation: " << pose.orientation.x << " " << pose.orientation.y << " " << pose.orientation.z << std::endl;
    }

    for (int i = 0; i < model_poses.size(); i++)
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
      primitive.dimensions[1] = 0.2;
      primitive.dimensions[2] = 0.1;

      /* A pose for the box (specified relative to frame_id) */
      geometry_msgs::Pose box_pose;

      box_pose.orientation.w = 1.0;
      box_pose.position.x = model_poses[i].position.x;
      box_pose.position.y = model_poses[i].position.y;
      box_pose.position.z = model_poses[i].position.z;

      box_pose.orientation = model_poses[i].orientation;

      collision_object.primitives.push_back(primitive);
      collision_object.primitive_poses.push_back(box_pose);
      collision_object.operation = collision_object.ADD;

      collision_objects.push_back(collision_object);

      /* moveit_msgs::ObjectColor object_color;
      object_color.id = collision_object.id;
      object_color.color.r = 239;
      object_color.color.g = 41;
      object_color.color.b = 41;
      object_color.color.a = 1;

      objects_color.push_back(object_color);*/
    }

    //planning_scene_interface.addCollisionObjects(collision_objects, objects_color);
    planning_scene_interface.addCollisionObjects(collision_objects);
  }

  void set_targets(std::string topic){
    ros::NodeHandle ps;
    ros::Subscriber pose_sub = ps.subscribe(topic, 10, &cr35ia::getPoses, this);

    /* wait for a pose */
    while (detected_poses == false)
    {
      std::cout << "No pose detected" << std::endl;
      usleep(500000);
    }

    ps.shutdown();

    std::cout << "Target poses: " << std::endl;
    for (int i = 0; i < pose_array.poses.size(); i++)
    {
      std::cout << "X: " << pose_array.poses[i].position.x << std::endl;
      std::cout << "Y: " << pose_array.poses[i].position.y << std::endl;
      std::cout << "Z: " << pose_array.poses[i].position.z << std::endl;
      std::cout << "Orientation: " << pose_array.poses[i].orientation.x << " " << pose_array.poses[i].orientation.y << " " << pose_array.poses[i].orientation.z << std::endl;
      targets.push_back(pose_array.poses[i]);
    }
  }

  void pick_and_place()
  {
    for (int i = 0; i < targets.size(); i++)
    {
      pick(i);
      place(i, false);
    }
  }

private:
  geometry_msgs::Pose target_pose, table_pose;
  moveit_msgs::Grasp grasp;
  moveit_msgs::PlaceLocation place_location;
  trajectory_msgs::JointTrajectory trajectory;
  moveit::planning_interface::MoveGroupInterface *move_group;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  geometry_msgs::PoseArray pose_array;
  std::vector<geometry_msgs::Pose> targets;
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  std::vector<moveit_msgs::ObjectColor> objects_color;
  bool detected_poses, detected_model;
  std::vector<geometry_msgs::Pose> model_poses;

  void getPoses(const geometry_msgs::PoseArray &msgs)
  {
    pose_array = msgs;
    detected_poses = true;
  }

  void getModelStates(const gazebo_msgs::ModelStates &msgs)
  {
    int pos = 0; 

    for(int i = 0; i < msgs.pose.size(); i++){
      if (msgs.name[i].find("target", pos) != std::string::npos){
        detected_model = true;
        model_poses.push_back(msgs.pose[i]);
      }
    }
    
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
    grasp.post_grasp_retreat.min_distance = 0.3;
    grasp.post_grasp_retreat.desired_distance = 0.5;

    openGripper(false);
    closedGripper(false);

    for (int j = 1; j <= 3; j += 2)
    {
      for (int k = 1; k <= 3; k += 2)
      {
        tf2::Quaternion orientation;
        tf2::fromMsg(targets[i].orientation, orientation);

        /* set correct angles for each target pose */
        double roll, pitch, yaw;
        tf2::Matrix3x3 matrix_orientation(orientation);
        matrix_orientation.getRPY(roll, pitch, yaw);
        orientation.setRPY(roll, pitch + k * M_PI / 2, yaw + j * M_PI / 2);

        grasp.grasp_pose.pose.orientation = tf2::toMsg(orientation);
        grasps.push_back(grasp);
      }
    }

    std::cout << "Generated " << grasps.size() << " grasp poses" << std::endl;
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
    for (int i = 0; i < 1000; i++)
    {
      place_location.place_pose.pose.position.x = -1;
      place_location.place_pose.pose.position.y = fRand(-1.24, 1.24);
      place_location.place_pose.pose.position.z = 0.9;
      openGripper(true);
      places.push_back(place_location);
    }

    std::cout << "Generated " << places.size() << " place locations" << std::endl;
    move_group->setSupportSurfaceName("place_location");
    move_group->place(std::to_string(i), places);

    if (return_start)
    {
      move_group->setNamedTarget("start");
      move_group->move();
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
  arm.generate_scene();
  arm.generate_object();
  arm.set_targets("/poses");
  arm.pick_and_place();

  ros::waitForShutdown();
  return 0;
}