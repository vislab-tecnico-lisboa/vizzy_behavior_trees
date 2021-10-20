#ifndef TRIGGER_SRV_HPP_
#define TRIGGER_SRV_HPP_

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_srvs/Trigger.h>

using namespace BT;

class TriggerSrvBT : public SyncActionNode
{

    public:

        int number;
        ros::ServiceClient srv_;

        TriggerSrvBT(const std::string& name, const NodeConfiguration& config);

    static PortsList providedPorts()
    {
        return { InputPort<std::string>("service_name"),
                 OutputPort<std::string>("message")};
    }

    NodeStatus tick() override;
};


#endif
