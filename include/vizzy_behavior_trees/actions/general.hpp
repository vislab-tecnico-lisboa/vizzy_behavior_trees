#ifndef GENERAL_ACTIONS_BT_HPP_
#define GENERAL_ACTIONS_BT_HPP_

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <vizzy_behavior_trees/util.hpp>
#include <vizzy_behavior_trees/rosbt_blackboard.hpp>

class DebugAction : public BT::AsyncActionNode
{
    public:
        DebugAction(const std::string& name, const BT::NodeConfiguration& config)
        : AsyncActionNode(name, config)
        {}

        static BT::PortsList providedPorts()
        {
            return{BT::InputPort<std::string>("string")};
        }


        BT::NodeStatus tick() override;
        virtual void halt() override {};

    private:
        std::atomic_bool _halt_requested;
};

class WaitForXSeconds : public BT::CoroActionNode
{
    public:
        WaitForXSeconds(const std::string& name, const BT::NodeConfiguration& config)
        : CoroActionNode(name, config)
        {}

        static BT::PortsList providedPorts()
        {
            return{BT::InputPort<std::string>("seconds"),
                   BT::InputPort<std::string>("result")};
        }


        BT::NodeStatus tick() override;
        void cleanup(bool halted);
        virtual void halt() override;

    private:
        std::atomic_bool _halt_requested;
};


#endif