#ifndef TORSO_ACTIONS_HPP_
#define TORSO_ACTIONS_HPP_

#include <behaviortree_cpp/behavior_tree.h>
#include <std_msgs/Float64.h>
#include <vizzy_behavior_trees/util.hpp>
#include <ros/ros.h>

using namespace BT;

class TorsoRoutineBT : public BT::AsyncActionNode
{
    public:
        TorsoRoutineBT(const std::string& name, const BT::NodeConfiguration& config);

        double last_ang;

        static BT::PortsList providedPorts()
        {
            return{BT::InputPort<std::string>("topic"),
                   BT::InputPort<double>("angle")};
        }

        ros::NodeHandle nh_;

        BT::NodeStatus tick() override;
        void halt() override {};

    private:
        static std::map<std::string, ros::Publisher> _publishers;
};


#endif