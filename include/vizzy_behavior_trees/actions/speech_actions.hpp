#ifndef SPEECH_ACTIONS_BT_H_
#define SPEECH_ACTIONS_BT_H_
#include <behaviortree_cpp/behavior_tree.h>
#include <actionlib/client/simple_action_client.h>
#include <woz_dialog_msgs/SpeechGoal.h>
#include <woz_dialog_msgs/SpeechAction.h>
#include <woz_dialog_msgs/SpeechFeedback.h>
#include <vizzy_behavior_trees/util.hpp>


typedef actionlib::SimpleActionClient<woz_dialog_msgs::SpeechAction> SpeechClient;

class SpeechActionBT : public BT::AsyncActionNode
{
    public:
        SpeechActionBT(const std::string& name, const BT::NodeConfiguration& config)
            : AsyncActionNode(name, config)
        {
        }

        static BT::PortsList providedPorts()
        {
            return{BT::InputPort<std::string>("action_name"),
                   BT::InputPort<std::string>("utterance"),
                   BT::InputPort<std::string>("voice"),
                   BT::InputPort<std::string>("language")};
        }

        BT::NodeStatus tick() override;

        virtual void halt() override;

    private:
        static std::map<std::string, std::shared_ptr<SpeechClient>> _speechClients;
        static std::map<std::string, bool> _speechClientsInitializing;
        std::atomic_bool _halt_requested;
};


#endif
