#include <vizzy_behavior_trees/actions/torso_actions.hpp>


std::map<std::string, ros::Publisher> TorsoRoutineBT::_publishers;

TorsoRoutineBT::TorsoRoutineBT(const std::string& name, const NodeConfiguration& config)
    : SyncActionNode(name, config), nh_(), last_ang(-1000)
{




}

BT::NodeStatus TorsoRoutineBT::tick()
{

    BT::Optional<std::string> topic = getInput<std::string>("topic");
    BT::Optional<double> angle = getInput<double>("angle");


    if(!topic)
    {
        throw BT::RuntimeError("missing required inputs [topic]: ",
                                   topic.error() );
    }if (!angle)
    {
        throw BT::RuntimeError("missing required inputs [torso_angle (deg)]: ",
                                   angle.error() );
    }

    double ang_val = angle.value()*M_PI/180;

    if(ang_val == last_ang)
    {
        return BT::NodeStatus::SUCCESS;
    }

    auto publisher_pair = _publishers.find(topic.value());

    if(publisher_pair == _publishers.end())
    {
        _publishers[topic.value()] = nh_.advertise<std_msgs::Float64>(topic.value(), 1, true);
        ROS_INFO_STREAM("Created publisher to topic: " << topic.value());
    }

    publisher_pair = _publishers.find(topic.value());
    ros::Publisher &pub = publisher_pair->second;


    std_msgs::Float64 command;

    command.data = ang_val;
    last_ang = ang_val;


    pub.publish(command);

    if(pub.getNumSubscribers() < 1)
    {
        return BT::NodeStatus::FAILURE;
    }else{
        return BT::NodeStatus::SUCCESS;
    }

}