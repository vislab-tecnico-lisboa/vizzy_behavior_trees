#include <vizzy_behavior_trees/actions/move_base_actions.hpp>


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




    /*Check if the action client is registered in the moveBaseClient map
    If it isn't register it. This allows us to have multiple actions for
    the same kind of actionlib (i.e. MoveBaseAction) and avoid creating
    duplicate action clients.*/
    

    auto action_client_pair = _moveBaseClients.find(action_name.value());
    auto init_pair = _moveBaseClientsInitializing.find(action_name.value());

    if(init_pair != _moveBaseClientsInitializing.end())
    {
        if(init_pair->second)
        {
            return BT::NodeStatus::FAILURE;
        }
    }

    std::shared_ptr<MoveBaseClient> client_PTR;

    if(action_client_pair == _moveBaseClients.end())
    {
        //Create action client and add it to the list of all clients

        client_PTR = std::make_shared<MoveBaseClient>(action_name.value());

        ROS_INFO_STREAM("Waiting for action server of: " << action_name.value()); 

        _moveBaseClientsInitializing[action_name.value()] = true;

        if(!client_PTR->waitForServer(ros::Duration(1)))
        {
            ROS_WARN_STREAM("Could not connect to action server: " << action_name.value());
            _moveBaseClients.erase(action_name.value());
            _moveBaseClientsInitializing.erase(action_name.value());
            return BT::NodeStatus::FAILURE;
        }

        _moveBaseClientsInitializing[action_name.value()] = false;

        ROS_INFO_STREAM("Found action server of: " << action_name.value()); 
        _moveBaseClients[action_name.value()] = client_PTR;

    }else
    {
        client_PTR = action_client_pair->second;
    }


    goal.target_pose = pose.value();
    goal.target_pose.header.frame_id = frame_id.value();

    std::cout << "[MoveBase]: Started." << std::endl <<
        "Pose: " << goal << std::endl;

    _halt_requested.store(false);

    _movebase_client_PTR->sendGoal(goal);

    auto move_state = _movebase_client_PTR->getState();
    
    setStatus(BT::NodeStatus::RUNNING);

    while(!move_state.isDone())
    {
        if(_halt_requested)
        {
            _movebase_client_PTR->cancelAllGoals();
            return BT::NodeStatus::FAILURE;
        }

        move_state = _movebase_client_PTR->getState();
        SleepMS(100);
    }


    if(_movebase_client_PTR->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        return BT::NodeStatus::SUCCESS;
    }else{
        return BT::NodeStatus::FAILURE;
    }

}

void MoveBaseActionBT::halt()
{
    _halt_requested.store(true);
}

