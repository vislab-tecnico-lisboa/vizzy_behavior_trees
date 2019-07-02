#ifndef GET_STD_MSGS_HPP_
#define GET_STD_MSGS_HPP_

#include <behaviortree_cpp/behavior_tree.h>
#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <ros/callback_queue.h>
#include <std_msgs/Int16.h>

using namespace BT;

class GetInt16BT : public SyncActionNode
{

    public:

        int number;
        ros::Subscriber sub_;
        ros::CallbackQueue queue_;

        GetInt16BT(const std::string& name, const NodeConfiguration& config);

    static PortsList providedPorts()
    {
        return { InputPort<std::string>("topic"), 
            OutputPort<int>("number")};
    }

    NodeStatus tick() override;
    void callback(const std_msgs::Int16::ConstPtr &msg);

};



#endif