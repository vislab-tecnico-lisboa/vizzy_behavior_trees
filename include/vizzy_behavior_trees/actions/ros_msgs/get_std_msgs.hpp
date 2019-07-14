#ifndef GET_STD_MSGS_HPP_
#define GET_STD_MSGS_HPP_

#include <behaviortree_cpp/behavior_tree.h>
#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <ros/callback_queue.h>

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

class GetFloat64BT : public SyncActionNode
{

    public:

        int number;
        ros::Subscriber sub_;
        ros::CallbackQueue queue_;

        GetFloat64BT(const std::string& name, const NodeConfiguration& config);

    static PortsList providedPorts()
    {
        return { InputPort<std::string>("topic"), 
            OutputPort<double>("number")};
    }

    NodeStatus tick() override;
    void callback(const std_msgs::Float64::ConstPtr &msg);

};


#endif