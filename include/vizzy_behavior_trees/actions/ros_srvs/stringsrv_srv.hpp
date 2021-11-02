#ifndef STRINGSRV_SRV_HPP_
#define STRINGSRV_SRV_HPP_

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <vizzy_behavior_trees/StringSrv.h>

using namespace BT;

class StringSrvBT : public SyncActionNode
{

    public:

        int number;
        ros::ServiceClient srv_;

        StringSrvBT(const std::string& name, const NodeConfiguration& config);

    static PortsList providedPorts()
    {
        return { InputPort<std::string>("service_name"),
                 InputPort<std::string>("cmd"),
                 InputPort<std::string>("arg1"),
                 InputPort<std::string>("arg2"),
                 OutputPort<std::string>("message")};
    }

    NodeStatus tick() override;
};


#endif
