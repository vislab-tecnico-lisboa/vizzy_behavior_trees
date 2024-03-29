#include <vizzy_behavior_trees/actions/move_base_actions.hpp>
#include "behaviortree_cpp_v3/bt_factory.h"



std::map<std::string, std::shared_ptr<MoveBaseClient>> MoveBaseActionBT::_moveBaseClients;
std::map<std::string, bool> MoveBaseActionBT::_moveBaseClientsInitializing;

BT::NodeStatus MoveBaseActionBT::tick()
{

    move_base_msgs::MoveBaseGoal goal;

    BT::Optional<geometry_msgs::PoseStamped> pose = getInput<geometry_msgs::PoseStamped>("goal_pose");
    BT::Optional<std::string> frame_id = getInput<std::string>("frame_id");
    BT::Optional<std::string> action_name = getInput<std::string>("action_name");

    if(!pose)
    {
        throw BT::RuntimeError("missing required inputs [pose]: ",
                                pose.error() );
    }if (!frame_id)
    {
        throw BT::RuntimeError("missing required inputs [frame_id]: ",
                                frame_id.error() );

    }if (!action_name)
    {
        throw BT::RuntimeError("missing required inputs [action_name]: ",
                                action_name.error() );
    }

    if(client_PTR == NULL)
    {
        client_PTR = RosBlackBoard::getActionClientOrInit<MoveBaseClient>(action_name.value(), this);
        if(client_PTR == NULL)
            return BT::NodeStatus::FAILURE;
    }

    goal.target_pose = pose.value();
    
    if(goal.target_pose.header.frame_id != "" && goal.target_pose.header.frame_id != frame_id.value())
    {

        try
        {
            geometry_msgs::PoseStamped pose_out;
            auto transform = buffer_.lookupTransform(frame_id.value(), goal.target_pose.header.frame_id, ros::Time(0));
            tf2::doTransform(pose.value(), pose_out, transform);
            goal.target_pose = pose_out;
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("Move base cannot publish to the requested frame_id. Using point's frame_id. Reason: %s",ex.what());
            goal.target_pose.header.frame_id = frame_id.value();

        }

    }else{

        goal.target_pose.header.frame_id = frame_id.value();
    }


    std::cout << "[MoveBase]: Started." << std::endl <<
        "Pose: " << goal << std::endl;

    client_PTR->sendGoal(goal);

    auto move_state = client_PTR->getState();

    while(!move_state.isDone())
    {
        move_state = client_PTR->getState();
        setStatusRunningAndYield();
    }

    cleanup(false);

    if(client_PTR->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        return BT::NodeStatus::SUCCESS;
    }else{
        return BT::NodeStatus::FAILURE;
    }

}

void MoveBaseActionBT::cleanup(bool halted)
{
    if(halted && client_PTR != NULL)
    {
        client_PTR->cancelAllGoals();
    }
}

void MoveBaseActionBT::halt()
{
    std::cout << name() <<": Halted." << std::endl;
    cleanup(true);
    CoroActionNode::halt();
}

