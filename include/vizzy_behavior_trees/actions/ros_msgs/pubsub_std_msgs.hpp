#ifndef GET_STD_MSGS_HPP_
#define GET_STD_MSGS_HPP_

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <ros/callback_queue.h>

#include <vizzy_behavior_trees/util.hpp>
#include <vizzy_behavior_trees/rosbt_blackboard.hpp>
#include <std_msgs/String.h>

using namespace BT;

class GetInt16BT : public SyncActionNode
{

    public:

        int number;
        ros::Subscriber sub_;
        ros::CallbackQueue queue_;
        bool initialized_ = false;

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

        double number;
        ros::Subscriber sub_;
        ros::CallbackQueue queue_;
        bool initialized_ = false;

        GetFloat64BT(const std::string& name, const NodeConfiguration& config);

    static PortsList providedPorts()
    {
        return { InputPort<std::string>("topic"), 
            OutputPort<double>("number")};
    }

    NodeStatus tick() override;
    void callback(const std_msgs::Float64::ConstPtr &msg);

};


//Pub String

using namespace BT;

class PubStringBT : public BT::SyncActionNode
{
    public:
        PubStringBT(const std::string& name, const BT::NodeConfiguration& config);

        static BT::PortsList providedPorts()
        {
            return{BT::InputPort<std::string>("topic"),
                   BT::InputPort<std::string>("message")
                   };
        }

        ros::NodeHandle nh_;

        BT::NodeStatus tick() override;

    private:
        static std::map<std::string, ros::Publisher> _publishers;
};


#endif