#include <vizzy_behavior_trees/actions/arm_cartesian_actions.hpp>


std::map<std::string, std::shared_ptr<CartesianClient>> CartesianActionBT::_cartesianClients;
std::map<std::string, bool> CartesianActionBT::_cartesianClientsInitializing;


BT::NodeStatus CartesianActionBT::tick()
{

    vizzy_msgs::CartesianGoal goal;

    BT::Optional<geometry_msgs::PoseStamped> pose = getInput<geometry_msgs::PoseStamped>("pose");
    BT::Optional<std::string> tipo = getInput<std::string>("type");
    BT::Optional<std::string> frame_id = getInput<std::string>("frame_id");
    BT::Optional<std::string> action_name = getInput<std::string>("action_name");

    if(!tipo)
    {
        throw BT::RuntimeError("missing required inputs [type]: ",
                                tipo.error() );
    }if(!pose)
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




    /*This allows us to have multiple action names for
    the same kind of actionlib action (i.e. CartesianAction for more than one robot)
    and avoid creating duplicate action clients (i.e. using the same action in multiple
    parts of the behavior_tree).*/


    auto action_client_pair = _cartesianClients.find(action_name.value());
    auto init_pair = _cartesianClientsInitializing.find(action_name.value());

    if(init_pair != _cartesianClientsInitializing.end())
    {
        if(init_pair->second)
        {
            return BT::NodeStatus::FAILURE;
        }
    }


    std::shared_ptr<CartesianClient> client_PTR;

    if(action_client_pair == _cartesianClients.end())
    {
        //Create action client and add it to the list of all clients

        client_PTR = std::make_shared<CartesianClient>(action_name.value());

        ROS_INFO_STREAM("Waiting for action server of: " << action_name.value());

        _cartesianClientsInitializing[action_name.value()] = true;

        if(!client_PTR->waitForServer(ros::Duration(1)))
        {
            ROS_WARN_STREAM("Could not connect to action server: " << action_name.value());
            _cartesianClients.erase(action_name.value());
            _cartesianClientsInitializing.erase(action_name.value());
            return BT::NodeStatus::FAILURE;
        }

        _cartesianClientsInitializing[action_name.value()] = false;

        ROS_INFO_STREAM("Found action server of: " << action_name.value());
        _cartesianClients[action_name.value()] = client_PTR;
        ROS_INFO_STREAM("Number of cartesian clients: " << _cartesianClients.size());

    }else
    {
        client_PTR = action_client_pair->second;
    }

    if(tipo.value() == "CARTESIAN")
    {

        geometry_msgs::PoseStamped poseStamped = pose.value();

        goal.type = goal.CARTESIAN;

        if(poseStamped.header.frame_id == "")
        {
            goal.end_effector_pose.header.frame_id = frame_id.value();
        }else
        {
            goal.end_effector_pose.header.frame_id = poseStamped.header.frame_id;
        }

        goal.end_effector_pose.pose = poseStamped.pose;

    }else if(tipo.value() == "HOME"){
        goal.type = goal.HOME;
    }else if(tipo.value() == "GRAB"){
        goal.type = goal.GRAB;
    }else if(tipo.value() == "RELEASE"){
        goal.type = goal.RELEASE;
    }else if(tipo.value() == "POINT"){
        goal.type = goal.POINT;
    }

    std::cout << "[Cartesian]: Started." << std::endl <<
        "Goal: " << goal << std::endl;

    _halt_requested.store(false);

    client_PTR->isServerConnected();

    client_PTR->sendGoal(goal);

    auto move_state = client_PTR->getState();

    setStatus(BT::NodeStatus::RUNNING);

    while(!move_state.isDone())
    {
        if(_halt_requested)
        {
            vizzy_msgs::CartesianGoal goal;
            goal.type = goal.PREEMPT;
            client_PTR->sendGoal(goal);
            return BT::NodeStatus::FAILURE;
        }

        move_state = client_PTR->getState();
        SleepMS(100);
    }


    if(client_PTR->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        return BT::NodeStatus::SUCCESS;
    }else{
        return BT::NodeStatus::FAILURE;
    }


}

void CartesianActionBT::halt()
{

    _halt_requested.store(true);
}
