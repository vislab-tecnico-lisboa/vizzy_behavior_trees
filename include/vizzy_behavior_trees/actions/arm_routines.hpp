#ifndef ARM_ROUTINES_BT_HPP_
#define ARM_ROUTINES_BT_HPP_

#include <behaviortree_cpp/behavior_tree.h>
#include <std_msgs/Int16.h>
#include <vizzy_behavior_trees/util.hpp>
#include <ros/ros.h>

using namespace BT;

class ArmRoutineBT : public BT::AsyncActionNode
{
    public:
        ArmRoutineBT(const std::string& name, const BT::NodeConfiguration& config);

        static BT::PortsList providedPorts()
        {
            return{BT::InputPort<std::string>("topic"),
                   BT::InputPort<std::string>("gesture")};
        }

        ros::NodeHandle nh_;

        BT::NodeStatus tick() override;
        virtual void halt() override;

    private:
        std::atomic_bool _halt_requested;
        static std::map<std::string, ros::Publisher> _publishers;
};


#endif
