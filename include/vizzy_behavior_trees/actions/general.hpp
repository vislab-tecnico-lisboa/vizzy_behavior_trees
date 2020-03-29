#ifndef GENERAL_ACTIONS_BT_HPP_
#define GENERAL_ACTIONS_BT_HPP_

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <vizzy_behavior_trees/util.hpp>
#include <vizzy_behavior_trees/rosbt_blackboard.hpp>
#include <vizzy_behavior_trees/GeneralAction.h>
#include <vizzy_behavior_trees/GeneralGoal.h>
#include <actionlib/client/simple_action_client.h>


typedef actionlib::SimpleActionClient<vizzy_behavior_trees::GeneralAction> GeneralActionClient;


class GeneralActionBT : public BT::CoroActionNode
{
    public:

        GeneralActionClient* client_PTR;

        GeneralActionBT(const std::string& name, const BT::NodeConfiguration& config)
            : CoroActionNode(name, config)
        {
            client_PTR = NULL;
        }

        static BT::PortsList providedPorts()
        {
            return{BT::InputPort<std::string>("constants"),
                   BT::OutputPort<std::string>("result"),
                   BT::InputPort<std::string>("variables"),
                   BT::InputPort<std::string>("action_name")};
        }

        BT::NodeStatus tick() override;
        virtual void halt() override;
        void cleanup(bool halted);

    private:
        std::atomic_bool _halt_requested;
};

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

/*This action will return FAILURE until x miliseconds*/
class TimerAction : public BT::SyncActionNode
{
    public:
        TimerAction(const std::string& name, const BT::NodeConfiguration& config)
        : SyncActionNode(name, config), first_time(true), previous_time_d(0)
        {}

        static BT::PortsList providedPorts()
        {
            return{BT::InputPort<std::string>("s_between_success")};
        }


        BT::NodeStatus tick() override;

    private:
        std::atomic_bool _halt_requested;
        int current_time;
        bool first_time;
        TimePoint timeout;
        int previous_time_d;
    
};

#endif