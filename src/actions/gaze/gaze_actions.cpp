#include <vizzy_behavior_trees/actions/gaze_actions.hpp>
#include "behaviortree_cpp_v3/bt_factory.h"



BT::NodeStatus GazeActionBT::tick()
{

    vizzy_msgs::GazeGoal goal;

    BT::Optional<geometry_msgs::PoseStamped> pose = getInput<geometry_msgs::PoseStamped>("fixation_pose");
    BT::Optional<std::string> frame_id = getInput<std::string>("frame_id");
    BT::Optional<std::string> action_name = getInput<std::string>("action_name");


    if(!pose)
    {
        ROS_ERROR_STREAM("missing required inputs [pose]: " << pose.error());

        return BT::NodeStatus::FAILURE;

    }if (!frame_id)
    {
        ROS_ERROR_STREAM("missing required inputs [frame_id]: " << frame_id.error());

        return BT::NodeStatus::FAILURE;

    }if (!action_name)
    {
        ROS_ERROR_STREAM("missing required inputs [action_name]: " << action_name.error());

        return BT::NodeStatus::FAILURE;

    }


    if(client_PTR == NULL) { 
        client_PTR = RosBlackBoard::getActionClientOrInit<GazeClient>(action_name.value(), this);
        if(client_PTR == NULL)
            return BT::NodeStatus::FAILURE;
    }

    geometry_msgs::PoseStamped poseStamped = pose.value();

    goal.type = goal.CARTESIAN;
    goal.fixation_point_error_tolerance = 0.05;
    if(poseStamped.header.frame_id == "")
    {
        goal.fixation_point.header.frame_id = frame_id.value();
    }else
    {
        goal.fixation_point.header.frame_id = poseStamped.header.frame_id;
    }

    goal.fixation_point.point = poseStamped.pose.position;

    if(!client_PTR->isServerConnected())
        return BT::NodeStatus::FAILURE;

    client_PTR->sendGoal(goal);

    return BT::NodeStatus::SUCCESS;

}

void GazeActionBT::cleanup(bool halted)
{
    if(halted && client_PTR != NULL)
    {

    }

}

void GazeActionBT::halt()
{
    std::cout << name() <<": Halted." << std::endl;
    cleanup(true);
    CoroActionNode::halt();
}