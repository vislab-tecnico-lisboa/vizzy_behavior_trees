#ifndef GENERAL_ACTIONS_BT_HPP_
#define GENERAL_ACTIONS_BT_HPP_

#include <behaviortree_cpp/behavior_tree.h>
#include <vizzy_behavior_trees/util.hpp>

class WaitForXSeconds : public BT::AsyncActionNode
{
    public:
        WaitForXSeconds(const std::string& name, const BT::NodeConfiguration& config)
        : AsyncActionNode(name, config)
        {}

        static BT::PortsList providedPorts()
        {
            return{BT::InputPort<std::string>("seconds"),
                   BT::InputPort<std::string>("result")};
        }


        BT::NodeStatus tick() override;
        virtual void halt() override;

    private:
        std::atomic_bool _halt_requested;
};


#endif