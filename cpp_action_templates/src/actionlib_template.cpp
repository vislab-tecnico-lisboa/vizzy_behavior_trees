/*
  -----------------------------------------------------
  A template to create behavior tree actions to control ROS actions.
  Don't forget to replace ACTIONLIB_TEMPLATE with the name of your
  action!
*/

#include <YOURPACKAGE/INCLUDE_DIR/actionlib_template.hpp>
#include "behaviortree_cpp_v3/bt_factory.h"


std::map<std::string, std::shared_ptr<ACTIONLIB_TEMPLATE_Client>> ACTIONLIB_TEMPLATE_BT::_ACTIONLIB_TEMPLATE_Clients;
std::map<std::string, bool> ACTIONLIB_TEMPLATE_BT::_ACTIONLIB_TEMPLATE_ClientsInitializing;

BT::NodeStatus ACTIONLIB_TEMPLATE_BT::tick()
{

    /*Inside this method you decide what you want this node to do once it gets
    ticked!*/


    /*You need to get input from the ports... at least the action name!*/
    BT::Optional<std::string> action_name = getInput<std::string>("action_name");
    
    /*Add more ports as necessary!*/
    /* ------- Examples --------- */
    /*BT::Optional<geometry_msgs::PoseStamped> pose = getInput<geometry_msgs::PoseStamped>("goal_pose");
    BT::Optional<std::string> frame_id = getInput<std::string>("frame_id");*/

    /*You should check if input ports have information or not*/
    if (!action_name)
    {
        throw BT::RuntimeError("missing required inputs [action_name]: ",
                                action_name.error() );
    }

    /*
    if(!pose)
    {
        throw BT::RuntimeError("missing required inputs [pose]: ",
                                pose.error() );
    }if (!frame_id)
    {
        throw BT::RuntimeError("missing required inputs [frame_id]: ",
                                frame_id.error() );
    }*/

    if(client_PTR == NULL)
    {
        client_PTR = RosBlackBoard::getActionClientOrInit<ACTIONLIB_TEMPLATE_Client>(action_name.value(), this);
        if(client_PTR == NULL)
            return BT::NodeStatus::FAILURE;
    }

    /*Check define your goal as needed*/

    SOME_ACTION_MSGS::SOME_ACTION_GOAL goal;

    /*--- EXAMPLE FOR A POSE STAMPED --- */
    /*goal.target_pose = pose.value();
    goal.target_pose.header.frame_id = frame_id.value();*/

    std::cout << "[ACTIONLIB_TEMPLATE]: Started." << std::endl <<
        "Pose: " << goal << std::endl;

    client_PTR->sendGoal(goal);

    auto state = client_PTR->getState();

    /*This keeps checking the action state.
    The code does not get stuck here: setStatusRunningAndYield
    will make the action node return RUNNING and keep the behavior
    tree running without getting stuck. The next time this node receives a tick
    it will resume from the point where setStatusRunningAndYield was called.
    This is happens in a single thread is its possible through the use of 
    corotines. This is more efficient and safer than using threads.*/
    while(!state.isDone())
    {
        state = client_PTR->getState();
        setStatusRunningAndYield();
    }

    cleanup(false);

    if(client_PTR->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        /*You can set some action output here if you want
        That might result from your action. */

        /*---- Example ----*/
        /* **You action computed an appropriate pose for the robot to enter
        a group a returned it for you to use. You expose it in the result output**

        auto action_result = client_PTR->getResult();
        auto result = setOutput("result", action_result->result);*/

        return BT::NodeStatus::SUCCESS;
    }else{
        return BT::NodeStatus::FAILURE;
    }

}

void ACTIONLIB_TEMPLATE_BT::cleanup(bool halted)
{
    if(halted && client_PTR != NULL)
    {
        client_PTR->cancelAllGoals();
    }
}

void ACTIONLIB_TEMPLATE_BT:halt()
{
    std::cout << name() <<": Halted." << std::endl;
    cleanup(true);
    CoroActionNode::halt();
}