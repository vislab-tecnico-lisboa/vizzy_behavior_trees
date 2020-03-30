/*
  -----------------------------------------------------
  A template to create the original library's (by facontidavide) AsyncActionNode actions
  Don't forget to replace ASYNCACTION_TEMPLATE with the name of your
  action!

  This action creates a new thread! Avoid using it! Use CoroActionNode and SyncActionNode
  as much as possible instead of this one
*/

#ifndef ASYNCACTION_TEMPLATE_ACTIONS_BT_H_
#define ASYNCACTION_TEMPLATE_ACTIONS_BT_H_

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <vizzy_behavior_trees/util.hpp>
#include <vizzy_behavior_trees/conversions.hpp>
#include <map>
#include <vizzy_behavior_trees/rosbt_blackboard.hpp>

class ASYNCACTION_TEMPLATE_BT : public BT::AsyncActionNode
{
    public:
        ASYNCACTION_TEMPLATE_BT(const std::string& name, const BT::NodeConfiguration& config)
        : AsyncActionNode(name, config)
        {}

        static BT::PortsList providedPorts()
        {
            return{BT::InputPort<SOME_TYPE>("some_input")};
        }


        BT::NodeStatus tick() override;
        virtual void halt() override {};

    private:
        std::atomic_bool _halt_requested;
};

#endif
