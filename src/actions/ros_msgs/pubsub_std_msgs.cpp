#include <vizzy_behavior_trees/actions/ros_msgs/pubsub_std_msgs.hpp>
#include "behaviortree_cpp_v3/bt_factory.h"


//GetInt16

void GetInt16BT::callback(const std_msgs::Int16::ConstPtr &msg)
{

    this->number = msg->data;

    if(!initialized_)
        initialized_ = true;
}


GetInt16BT::GetInt16BT(const std::string& name, const NodeConfiguration& config)
    : SyncActionNode(name, config)
    {
        ros::NodeHandle nh_;
        BT::Optional<std::string> topic = TreeNode::getInput<std::string>("topic");

        if(!topic){
            throw BT::RuntimeError("missing required inputs [topic]: ",
                                    topic.error()); 
        }

        ros::SubscribeOptions ops = ros::SubscribeOptions::create<std_msgs::Int16>
            (topic.value(), 1, boost::bind(&GetInt16BT::callback, this, _1), ros::VoidPtr(), &queue_);

        sub_ = nh_.subscribe(ops);
    }

BT::NodeStatus GetInt16BT::tick()
{
    if(sub_.getNumPublishers() < 1)
    {
        return BT::NodeStatus::FAILURE;
    }

    queue_.callAvailable();

    /*If no data was received we cannot make an computations*/
    if(!initialized_)
        return BT::NodeStatus::FAILURE;

    auto result = setOutput("number", this->number);

    if(!result)
    {
        std::cout << result.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

   return BT::NodeStatus::SUCCESS;
}


//Float 64
void GetFloat64BT::callback(const std_msgs::Float64::ConstPtr &msg)
{
    this->number = msg->data;

    if(!initialized_)
        initialized_ = true;
}


GetFloat64BT::GetFloat64BT(const std::string& name, const NodeConfiguration& config)
    : SyncActionNode(name, config)
    {
        ros::NodeHandle nh_;
        BT::Optional<std::string> topic = TreeNode::getInput<std::string>("topic");

        if(!topic){
            throw BT::RuntimeError("missing required inputs [topic]: ",
                                    topic.error()); 
        }

        ros::SubscribeOptions ops = ros::SubscribeOptions::create<std_msgs::Float64>
            (topic.value(), 1, boost::bind(&GetFloat64BT::callback, this, _1), ros::VoidPtr(), &queue_);

        sub_ = nh_.subscribe(ops);
    }

BT::NodeStatus GetFloat64BT::tick()
{
    
    if(sub_.getNumPublishers() < 1)
    {
        return BT::NodeStatus::FAILURE;
    }

    queue_.callAvailable();

    /*If no data was received we cannot make an computations*/
    if(!initialized_)
        return BT::NodeStatus::FAILURE;


    auto result = setOutput("number", this->number);

    if(!result)
    {
        std::cout << result.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

   return BT::NodeStatus::SUCCESS;
}


//Pub std_msgs

std::map<std::string, ros::Publisher> PubStringBT::_publishers;

PubStringBT::PubStringBT(const std::string& name, const NodeConfiguration& config)
    : SyncActionNode(name, config), nh_()
{
}

BT::NodeStatus PubStringBT::tick()
{

    BT::Optional<std::string> topic = getInput<std::string>("topic");
    BT::Optional<std::string> message = getInput<std::string>("message");


    /*Check if inputs are set*/
    if(!topic)
    {
        throw BT::RuntimeError("missing required inputs [topic]: ",
                                   topic.error() );
    }

    if(!message)
    {
        throw BT::RuntimeError("missing required inputs [message]: ",
                                   message.error() );
    }
    


    auto publisher_pair = _publishers.find(topic.value());

    if(publisher_pair == _publishers.end())
    {
        _publishers[topic.value()] = nh_.advertise<std_msgs::String>(topic.value(), 1, true);
        ROS_INFO_STREAM("Created publisher to topic: " << topic.value());
    }

    publisher_pair = _publishers.find(topic.value());
    ros::Publisher &pub = publisher_pair->second;


    std_msgs::String command;
    command.data = message.value();

    if(pub.getNumSubscribers() < 1)
    {
        std::stringstream ss;
        std::string nm = this->registrationName();
        ss << nm;
        ss << " failed to publish: ";
        ss << command.data;

        ROS_WARN_STREAM(ss.str());

        return BT::NodeStatus::FAILURE;
    }else{
        pub.publish(command);
        return BT::NodeStatus::SUCCESS;
    }
 
}