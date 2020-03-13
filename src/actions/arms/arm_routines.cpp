#include <vizzy_behavior_trees/actions/arm_routines.hpp>

std::map<std::string, ros::Publisher> ArmRoutineBT::_publishers;

ArmRoutineBT::ArmRoutineBT(const std::string& name, const NodeConfiguration& config)
    : SyncActionNode(name, config), nh_()
{
}

BT::NodeStatus ArmRoutineBT::tick()
{

    BT::Optional<std::string> topic = getInput<std::string>("topic");
    BT::Optional<std::string> gesture = getInput<std::string>("gesture");

    if(!topic)
    {
        throw BT::RuntimeError("missing required inputs [topic]: ",
                                   topic.error() );
    }if (!gesture)
    {
        throw BT::RuntimeError("missing required inputs [gesture]: ",
                                   gesture.error() );
    }


    auto publisher_pair = _publishers.find(topic.value());

    if(publisher_pair == _publishers.end())
    {
        _publishers[topic.value()] = nh_.advertise<std_msgs::Int16>(topic.value(), 1);
        ROS_INFO_STREAM("Created publisher to topic: " << topic.value());
    }

    publisher_pair = _publishers.find(topic.value());
    ros::Publisher &pub = publisher_pair->second;

    std::string gest = gesture.value();

    std_msgs::Int16 command;

    if(gest == "HOME")
    {
        command.data = 0;
    }else if(gest == "WAVE")
    {
        command.data = 1;
    }else if(gest == "STRETCH")
    {
        command.data = 2;
    }else if(gest == "HANDSHAKE")
    {
        command.data = 3;
    }else if(gest == "ASK_HANDSHAKE")
    {
        command.data = 4;
    }else if(gest == "HANDSHAKE_PID")
    {
        command.data = 5;
    }else if(gest == "ARMS_DOWN")
    {
        command.data = 6;
    }else if(gest == "HAPPY_EMOTIONLESS")
    {
        command.data = 7;
    }else if(gest == "HAPPY_EMOTIVE")
    {
        command.data = 8;
    }else if(gest == "SAD")
    {
        command.data = 9;
    }else if(gest == "ANGRY")
    {
        command.data = 10;
    }else if(gest == "FEAR")
    {
        command.data = 11;
    }else if(gest == "SURPRISE")
    {
        command.data = 13;
    }else if(gest == "STRETCH_OPEN")
    {
        command.data = 14;
    }else if(gest == "SURPRISE_OPEN")
    {
        command.data = 15;
    }else{
        return BT::NodeStatus::FAILURE;
    }

    if(pub.getNumSubscribers() < 1)
    {
        return BT::NodeStatus::FAILURE;
    }else{
        pub.publish(command);
        return BT::NodeStatus::SUCCESS;
    }


    return BT::NodeStatus::SUCCESS;

}
