#include <vizzy_behavior_trees/actions/gaze_actions.hpp>


std::map<std::string, std::shared_ptr<GazeClient>> GazeActionBT::_gazeClients;
std::map<std::string, bool> GazeActionBT::_gazeClientsInitializing;


BT::NodeStatus GazeActionBT::tick()
{

    vizzy_msgs::GazeGoal goal;

    BT::Optional<geometry_msgs::PoseStamped> pose = getInput<geometry_msgs::PoseStamped>("fixation_pose");
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
    

    auto action_client_pair = _gazeClients.find(action_name.value());
    auto init_pair = _gazeClientsInitializing.find(action_name.value());

    if(init_pair != _gazeClientsInitializing.end())
    {
        if(init_pair->second)
        {
            return BT::NodeStatus::FAILURE;
        }
    }

    std::shared_ptr<GazeClient> client_PTR;

    if(action_client_pair == _gazeClients.end())
    {
        //Create action client and add it to the list of all clients

        client_PTR = std::make_shared<GazeClient>(action_name.value());

        ROS_INFO_STREAM("Waiting for action server of: " << action_name.value()); 

        _gazeClientsInitializing[action_name.value()] = true;

        if(!client_PTR->waitForServer(ros::Duration(1)))
        {
            ROS_WARN_STREAM("Could not connect to action server: " << action_name.value());
            _gazeClients.erase(action_name.value());
            _gazeClientsInitializing.erase(action_name.value());
            return BT::NodeStatus::FAILURE;
        }

        _gazeClientsInitializing[action_name.value()] = false;

        ROS_INFO_STREAM("Found action server of: " << action_name.value()); 
        _gazeClients[action_name.value()] = client_PTR;
        ROS_INFO_STREAM("Number of gaze clients: " << _gazeClients.size());

    }else
    {
        client_PTR = action_client_pair->second;
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

    std::cout << "[Gaze]: Started." << std::endl <<
        "Fixation point: " << goal << std::endl;

    _halt_requested.store(false);

    client_PTR->isServerConnected();

    client_PTR->sendGoal(goal);
    
    auto move_state = client_PTR->getState();

    return BT::NodeStatus::SUCCESS;

}

void GazeActionBT::halt()
{
}