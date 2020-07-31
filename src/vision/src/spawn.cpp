#include <ros/ros.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

bool detected = false;
std::vector<geometry_msgs::Pose> model_poses;
std::vector<std::string> model_name;
gazebo_msgs::ModelStates model;

double fRand(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

void getModelStates(const gazebo_msgs::ModelStates &msgs)
{
    model = msgs;
}

int main(int argc, char **argv)
{
    srandom(time(NULL));
    ros::init(argc, argv, "gazebo_set_states_client");

    ros::NodeHandle ps;
    ros::Subscriber pose_sub = ps.subscribe("/gazebo/model_states", 1000, getModelStates);
    ros::Duration duration(1. / 24.);

    /* wait for a pose */
    while (detected == false)
    {
        ros::spinOnce();

        for (int i = 0; i < model.pose.size(); i++)
        {
            int pos = 0;
            if (model.name[i].find("target", pos) != std::string::npos)
            {
                detected = true;
                ps.shutdown();
            }
        }

        duration.sleep();
    }

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

    for (int i = 0; i < model.pose.size(); i++)
    {
        int pos = 0;
        if (model.name[i].find("target", pos) != std::string::npos)
        {
            std::cout << "changing " << model.name[i] << std::endl;
            gazebo_msgs::SetModelState objstate;

            objstate.request.model_state.model_name = model.name[i];

            std::cout << model.name[i] << std::endl;

            objstate.request.model_state.pose = model.pose[i];

            tf2::Quaternion orientation;
            orientation.setRPY(0, 0, fRand(-0.75, 0.75));
            objstate.request.model_state.pose.orientation = tf2::toMsg(orientation);

            objstate.request.model_state.twist.linear.x = 0.0;
            objstate.request.model_state.twist.linear.y = 0.0;
            objstate.request.model_state.twist.linear.z = 0.0;
            objstate.request.model_state.twist.angular.x = 0.0;
            objstate.request.model_state.twist.angular.y = 0.0;
            objstate.request.model_state.twist.angular.z = 0.0;
            objstate.request.model_state.reference_frame = "world";
            client.call(objstate);
        }
    }
}