/*
  -----------------------------------------------------
  A template to create the original library's (by facontidavide) CoroActionNode actions
  Don't forget to replace COROACTION_TEMPLATE with the name of your
  action!
*/

#ifndef COROACTION_TEMPLATE_ACTIONS_BT_H_
#define COROACTION_TEMPLATE_ACTIONS_BT_H_

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <vizzy_behavior_trees/util.hpp>
#include <vizzy_behavior_trees/conversions.hpp>
#include <map>
#include <vizzy_behavior_trees/rosbt_blackboard.hpp>

class COROACTION_TEMPLATE_BT : public BT::CoroActionNode
{
    public:

        COROACTION_TEMPLATE_BT(const std::string& name, const BT::NodeConfiguration& config)
            : CoroActionNode(name, config)
        {
            //Some initialization operations that you need
        }

        static BT::PortsList providedPorts()
        {
            return{BT::InputPort<SOME_TYPE>("some_input")
                   /*Some extra inputs as you need...
                   --- Examples ---
                   BT::InputPort<int>("extra_input"),
                   BT::InputPort<std::string>("fancy_input")*/};
        }

        BT::NodeStatus tick() override;
        virtual void halt() override;
        void cleanup(bool halted);

    private:
        std::atomic_bool _halt_requested;
};

#endif
