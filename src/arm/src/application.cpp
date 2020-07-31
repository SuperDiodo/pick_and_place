#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseArray.h>
#include <gazebo_msgs/ModelStates.h>
#include <unistd.h>

/* generate random double in a range */
double fRand(double fMin, double fMax)
{
  double f = (double)rand() / RAND_MAX;
  return fMin + f * (fMax - fMin);
}

/* fanuc cr35ia arm's class */
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

  /* High level method, simulate pick_and_place pipeline */
  void pick_and_place(std::string topic)
  {
    generate_scene();
    generate_object();
    set_targets(topic);

    for (int i = 0; i < targets.size(); i++)
    {
      pick(i);
      place(i, false);
    }
  }

private:
  moveit_msgs::Grasp grasp;
  moveit_msgs::PlaceLocation place_location;
  moveit::planning_interface::MoveGroupInterface *move_group;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  geometry_msgs::PoseArray pose_array;
  std::vector<geometry_msgs::Pose> targets;
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  std::vector<moveit_msgs::ObjectColor> objects_color;
  bool detected_poses, detected_model;
  gazebo_msgs::ModelStates model;

  /* callback kinect topic */
  void getPoses(const geometry_msgs::PoseArray &msgs)
  {
    pose_array = msgs;
    detected_poses = true;
  }

  /* callback gazebo topic */
  void getModelStates(const gazebo_msgs::ModelStates &msgs)
  {
    model = msgs;
    int pos = 0;

    for (int i = 0; i < msgs.pose.size(); i++)
    {
      if (msgs.name[i].find("target", pos) != std::string::npos)
      {
        detected_model = true;
      }
    }
  }

  /* add in RVIZ the tables */
  void generate_scene()
  {
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group->getPlanningFrame();

    /* ORIGIN TABLE */
    collision_object.id = "origin_table";

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 2;
    primitive.dimensions[1] = 3;
    primitive.dimensions[2] = 0.8;

    geometry_msgs::Pose origin_table;

    origin_table.orientation.w = 1.0;
    origin_table.position.x = 1.5;
    origin_table.position.y = 0;
    origin_table.position.z = 0.4;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(origin_table);
    collision_object.operation = collision_object.ADD;

    collision_objects.push_back(collision_object);

    /* DESTINATION TABLE */
    collision_object.id = "place_location";

    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 1;
    primitive.dimensions[1] = 3;
    primitive.dimensions[2] = 0.8;

    geometry_msgs::Pose place_location;

    place_location.orientation.w = 1.0;
    place_location.position.x = -1;
    place_location.position.y = 0;
    place_location.position.z = 0.4;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(place_location);
    collision_object.operation = collision_object.ADD;

    collision_objects.push_back(collision_object);
    planning_scene_interface.addCollisionObjects(collision_objects);
  }

  /* add in RVIZ the targets */
  void generate_object()
  {
    ros::NodeHandle ps;
    ros::Subscriber pose_sub = ps.subscribe("/gazebo/model_states", 10, &cr35ia::getModelStates, this);

    std::cout << "wait for model_state.." << std::endl;
    while (detected_model == false)
      usleep(500000);

    ps.shutdown();

    int ids = 0;

    /* generate a collision object for each target in the model_state */
    for (int i = 0; i < model.pose.size(); i++)
    {
      int pos = 0;
      if (model.name[i].find("target", pos) != std::string::npos)
      {
        moveit_msgs::CollisionObject collision_object;
        collision_object.header.frame_id = move_group->getPlanningFrame();

        /* The id of the object is used to identify it. */
        collision_object.id = std::to_string(ids);

        std::cout << "Collision object name: " << model.name[i] << std::endl;
        std::cout << "X: " << model.pose[i].position.x << std::endl;
        std::cout << "Y: " << model.pose[i].position.y << std::endl;
        std::cout << "Z: " << model.pose[i].position.z << std::endl;
        std::cout << "Orientation: " << model.pose[i].orientation.x << " " << model.pose[i].orientation.y << " " << model.pose[i].orientation.z << std::endl;

        ids++;

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
        box_pose.position.x = model.pose[i].position.x;
        box_pose.position.y = model.pose[i].position.y;
        box_pose.position.z = model.pose[i].position.z;

        box_pose.orientation = model.pose[i].orientation;

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;

        collision_objects.push_back(collision_object);

        moveit_msgs::ObjectColor object_color;
        object_color.id = collision_object.id;
        object_color.color.r = 239;
        object_color.color.g = 41;
        object_color.color.b = 41;
        object_color.color.a = 1;

        objects_color.push_back(object_color);
      }
    }

    planning_scene_interface.addCollisionObjects(collision_objects, objects_color);
  }

  /* set the estimated pose from kinect as targets*/
  void set_targets(std::string topic)
  {
    ros::NodeHandle ps;
    ros::Subscriber pose_sub = ps.subscribe(topic, 10, &cr35ia::getPoses, this);

    std::cout << "waiting for targets.." << std::endl;
    while (detected_poses == false)
      usleep(500000);

    ps.shutdown();

    /* data association */
    int name = 0;
    std::cout << "\nTarget poses: " << std::endl;
    for (int j = 0; j < model.pose.size(); j++)
    {
      int pos = 0;
      if (model.name[j].find("target", pos) != std::string::npos)
      {
        for (int i = 0; i < pose_array.poses.size(); i++)
        {

          double err_x = abs(model.pose[j].position.x - pose_array.poses[i].position.x);
          double err_y = abs(model.pose[j].position.y - pose_array.poses[i].position.y);
          double err_z = abs(model.pose[j].position.z - pose_array.poses[i].position.z);

          if (err_x < 0.1 && err_y < 0.1 && err_z < 0.1)
          {
            std::cout << "Target name: " << name << std::endl;
            std::cout << "X: " << pose_array.poses[i].position.x << std::endl;
            std::cout << "Y: " << pose_array.poses[i].position.y << std::endl;
            std::cout << "Z: " << pose_array.poses[i].position.z << std::endl;
            std::cout << "Orientation: " << pose_array.poses[i].orientation.x << " " << pose_array.poses[i].orientation.y << " " << pose_array.poses[i].orientation.z << std::endl;
            targets.push_back(pose_array.poses[i]);

            name++;
          }
        }
      }
    }
  }

  /* set closing-hand joint pose for grasp/place pose */
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

  /* set opening-hand joint pose for grasp/place pose */
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

  /* generate grasps poses and try to pick the target */
  void pick(int target_id)
  {
    std::vector<moveit_msgs::Grasp> grasps;

    grasp.grasp_pose.header.frame_id = "base_link";

    grasp.grasp_pose.pose.position.x = targets[target_id].position.x;
    grasp.grasp_pose.pose.position.y = targets[target_id].position.y;
    grasp.grasp_pose.pose.position.z = targets[target_id].position.z + 0.25;

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

    for (int k = 1; k <= 2; k++)
    {
      tf2::Quaternion orientation;
      tf2::fromMsg(targets[target_id].orientation, orientation);

      /* set correct angles for each target pose */
      double roll, pitch, yaw;
      tf2::Matrix3x3 matrix_orientation(orientation);
      matrix_orientation.getRPY(roll, pitch, yaw);
      orientation.setRPY(roll, pitch + pow(-1, k) * M_PI / 2, yaw);
      grasp.grasp_pose.pose.orientation = tf2::toMsg(orientation);
      grasps.push_back(grasp);
    }

    std::cout << "Generated " << grasps.size() << " grasp poses" << std::endl;
    move_group->pick(std::to_string(target_id), grasps);
  }

  /* generate place poses and try to place the target */
  void place(int target_id, bool return_start)
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
      place_location.place_pose.pose.position.z = 0.85;
      openGripper(true);
      places.push_back(place_location);
    }

    std::cout << "Generated " << places.size() << " place locations" << std::endl;
    move_group->setSupportSurfaceName("place_location");
    move_group->place(std::to_string(target_id), places);

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
  arm.pick_and_place("/poses");

  ros::waitForShutdown();
  return 0;
}