
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

double fRand(double fMin, double fMax)
{
  double f = (double)rand() / RAND_MAX;
  return fMin + f * (fMax - fMin);
}

int main(int argc, char **argv)
{
 
  if(argc < 4){
    std::cout << "Not enough argument: [min range pose][max range pose][attempts]" << std::endl;
    return 1;
  }  

  ros::init(argc, argv, "simple_move_group_node");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP = "arm";

  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  const robot_state::JointModelGroup *joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

  move_group.setPoseReferenceFrame("world");

  srandom(time(NULL));
  bool success = false;
  int i = 0, attempts = atoi(argv[3]);

  double min = strtod(argv[1], NULL), max = strtod(argv[2], NULL);

  geometry_msgs::Pose target_pose1;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  do
  {
    
    target_pose1.orientation.w = fRand(min, max);
    target_pose1.position.x = fRand(min, max);
    target_pose1.position.y = fRand(min, max);
    target_pose1.position.z = fRand(min, max);
    move_group.setPoseTarget(target_pose1);

    move_group.setPlanningTime(10.0);

    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    i++;

    std::cout << "attempt: " << i << std::endl;

  } while (success == false && i < attempts);

  if (success == true)
  {
    std::cout << "Planning successful, pose [w][x][y][z]: " << target_pose1.orientation.w << " " << target_pose1.position.x << " " << target_pose1.position.y << " " << target_pose1.position.z << std::endl;
    move_group.execute(my_plan);	
  }
  else{
    std::cout << "Planning failed with " << attempts << std::endl;
  }

  ros::shutdown();
  return 0;
}
