#include <vizzy_behavior_trees/actions/ros_msgs/get_std_msgs.hpp>
#include "behaviortree_cpp_v3/bt_factory.h"



//GetInt16

void GetInt16BT::callback(const std_msgs::Int16::ConstPtr &msg)
{

    this->number = msg->data;
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

    setStatus(BT::NodeStatus::RUNNING);
    queue_.callOne();


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

    setStatus(BT::NodeStatus::RUNNING);
    queue_.callOne();


    auto result = setOutput("number", this->number);

    if(!result)
    {
        std::cout << result.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

   return BT::NodeStatus::SUCCESS;
}