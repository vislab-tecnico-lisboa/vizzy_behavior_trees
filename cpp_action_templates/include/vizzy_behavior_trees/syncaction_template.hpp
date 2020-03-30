/*
  -----------------------------------------------------
  A template to create the original library's (by facontidavide) SyncActionNode actions
  Don't forget to replace SYNCACTION_TEMPLATE with the name of your
  action!
*/

#ifndef SYNCACTION_TEMPLATE_ACTIONS_BT_H_
#define SYNCACTION_TEMPLATE_ACTIONS_BT_H_

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <vizzy_behavior_trees/util.hpp>
#include <vizzy_behavior_trees/conversions.hpp>
#include <map>
#include <vizzy_behavior_trees/rosbt_blackboard.hpp>

class SYNCACTION_TEMPLATE_BT : public BT::AsyncActionNode
{
    public:
        SYNCACTION_TEMPLATE_BT(const std::string& name, const BT::NodeConfiguration& config)
        : AsyncActionNode(name, config)
        {}

        static BT::PortsList providedPorts()
        {
            return{BT::InputPort<SOME_TYPE>("some_input")
            /*Add extra InputPorts and OutputPorts as you need...
            ---- Examples ---
            BT::InputPort<std::string>("speech_action"),
            BT::OutputPort<std::string>("utterance")*/};
        }


        BT::NodeStatus tick() override;
};

#endif