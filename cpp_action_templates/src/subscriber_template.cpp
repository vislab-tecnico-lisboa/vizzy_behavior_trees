/*
  -----------------------------------------------------
  A template to create behavior tree actions to read from ROS topics.
  Don't forget to replace SUBSCRIBER_TEMPLATE with the name of your
  action!
*/

#include <YOURPACKAGE/INCLUDE_DIR/subscriber_template.hpp>
#include "behaviortree_cpp_v3/bt_factory.h"


void SUBSCRIBER_TEMPLATE_BT::callback(const std_msgs::Int16::ConstPtr &msg)
{

    this->last_msg = *msg;
}


SUBSCRIBER_TEMPLATE_BT::SUBSCRIBER_TEMPLATE_BT(const std::string& name, const NodeConfiguration& config)
    : SyncActionNode(name, config)
    {
        ros::NodeHandle nh_;
        BT::Optional<std::string> topic = TreeNode::getInput<std::string>("topic");

        if(!topic){
            throw BT::RuntimeError("missing required inputs [topic]: ",
                                    topic.error()); 
        }

        ros::SubscribeOptions ops = ros::SubscribeOptions::create<std_msgs::Int16>
            (topic.value(), 1, boost::bind(&SUBSCRIBER_TEMPLATE_BT::callback, this, _1), ros::VoidPtr(), &queue_);

        sub_ = nh_.subscribe(ops);
    }

BT::NodeStatus SUBSCRIBER_TEMPLATE_BT::tick()
{
    if(sub_.getNumPublishers() < 1)
    {
        return BT::NodeStatus::FAILURE;
    }

    setStatus(BT::NodeStatus::RUNNING);

    /*Each tick will check for a new message and update it. if there are some publishers*/
    queue_.callOne();


    auto result = setOutput("message", this->last_msg);

    if(!result)
    {
        std::cout << result.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

   return BT::NodeStatus::SUCCESS;
}