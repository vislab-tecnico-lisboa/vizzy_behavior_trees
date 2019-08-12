#ifndef SPEECH_ACTIONS_BT_H_
#define SPEECH_ACTIONS_BT_H_
#include <behaviortree_cpp/behavior_tree.h>
#include <actionlib/client/simple_action_client.h>
#include <woz_dialog_msgs/SpeechGoal.h>
#include <woz_dialog_msgs/SpeechAction.h>
#include <woz_dialog_msgs/SpeechFeedback.h>
#include <vizzy_behavior_trees/util.hpp>
#include <vizzy_behavior_trees/rosbt_blackboard.hpp>


typedef actionlib::SimpleActionClient<woz_dialog_msgs::SpeechAction> SpeechClient;

class SpeechActionBT : public BT::CoroActionNode
{
    public:

        SpeechClient* client_PTR;

        SpeechActionBT(const std::string& name, const BT::NodeConfiguration& config)
            : CoroActionNode(name, config)
        {
            client_PTR = NULL;
        }

        static BT::PortsList providedPorts()
        {
            return{BT::InputPort<std::string>("action_name"),
                   BT::InputPort<std::string>("utterance"),
                   BT::InputPort<std::string>("voice"),
                   BT::InputPort<std::string>("language")};
        }

        BT::NodeStatus tick() override;
        void cleanup(bool halted);
        virtual void halt() override;

    private:
        std::atomic_bool _halt_requested;
};


#endif
