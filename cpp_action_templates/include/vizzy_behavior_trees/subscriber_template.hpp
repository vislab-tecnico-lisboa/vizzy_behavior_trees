/*
  -----------------------------------------------------
  A template to create behavior tree actions to read from ROS topics.
  Don't forget to replace SUBSCRIBER_TEMPLATE with the name of your
  action!
*/

#ifndef SUBSCRIBER_TEMPLATE_BT_HPP_
#define SUBSCRIBER_TEMPLATE_BT_HPP_

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>

/*Include files necessary subscribe to a topc go here*/
/* --- Example --- */



using namespace BT;

class SUBSCRIBER_TEMPLATE_BT : public SyncActionNode
{

    public:

        SUBSCRIBER_MESSAGE_TYPE last_msg;
        ros::Subscriber sub_;
        ros::CallbackQueue queue_;

        SUBSCRIBER_TEMPLATE_BT(const std::string& name, const NodeConfiguration& config);

    static PortsList providedPorts()
    {
        return { InputPort<std::string>("topic"),
                 OutputPort<SUBSCRIBER_MESSAGE_TYPE>("message")
            /* Extra ports that you need
            --- Example ---
            InputPort<int>("number")*/};
    }

    NodeStatus tick() override;
    void callback(const SOME_ROS_MESSAGE_TYPE::ConstPtr &msg);

};


#endif