#include <vizzy_behavior_trees/actions/gaze_actions.hpp>


BT::NodeStatus GazeActionBT::tick()
{

    auto Now = [](){ return std::chrono::high_resolution_clock::now(); };

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


    if(client_PTR == NULL) { 
        client_PTR = RosBlackBoard::getActionClientOrInit<GazeClient>("gaze");

        ROS_INFO_STREAM("Waiting for action server of: gaze"); 


        TimePoint init_time = Now();
        TimePoint timeout_time = Now()+std::chrono::milliseconds(5000);

        while(!client_PTR->isServerConnected())
        {

            if(Now() > timeout_time)
            {
                ROS_WARN_STREAM("Could not connect to action server: gaze");
                return BT::NodeStatus::FAILURE;
            }

            setStatusRunningAndYield();
        }

        ROS_INFO_STREAM("Found action server of: gaze");
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

    client_PTR->isServerConnected();

    client_PTR->sendGoal(goal);

    return BT::NodeStatus::SUCCESS;

}

void GazeActionBT::cleanup(bool halted)
{
    if(halted)
    {

    }

}

void GazeActionBT::halt()
{
    std::cout << name() <<": Halted." << std::endl;
    cleanup(true);
    CoroActionNode::halt();
}