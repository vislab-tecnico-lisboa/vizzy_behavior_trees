#include <vizzy_behavior_trees/actions/torso_actions.hpp>


std::map<std::string, ros::Publisher> TorsoRoutineBT::_publishers;

TorsoRoutineBT::TorsoRoutineBT(const std::string& name, const NodeConfiguration& config)
    : AsyncActionNode(name, config), nh_()
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


    auto publisher_pair = _publishers.find(topic.value());

    if(publisher_pair == _publishers.end())
    {
        _publishers[topic.value()] = nh_.advertise<std_msgs::Float64>(topic.value(), 1);
        ROS_INFO_STREAM("Created publisher to topic: " << topic.value());
    }

    publisher_pair = _publishers.find(topic.value());
    ros::Publisher &pub = publisher_pair->second;

    double ang_val = angle.value()*180/M_PI;

    std_msgs::Float64 command;

    command.data = ang_val;

    _halt_requested.store(false);

    pub.publish(command);
    setStatus(BT::NodeStatus::RUNNING);
    SleepMS(1000);

    return BT::NodeStatus::SUCCESS;

}
void TorsoRoutineBT::halt()
{
    _halt_requested.store(true);
}
