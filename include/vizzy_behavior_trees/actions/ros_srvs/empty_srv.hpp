#ifndef EMPTY_SRV_HPP_
#define EMPTY_SRV_HPP_

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_srvs/Empty.h>

using namespace BT;

class EmptySrvBT : public SyncActionNode
{

    public:

        int number;
        ros::ServiceClient srv_;

        EmptySrvBT(const std::string& name, const NodeConfiguration& config);

    static PortsList providedPorts()
    {
        return { InputPort<std::string>("service")};
    }

    NodeStatus tick() override;
};


#endif