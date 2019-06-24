#ifndef SPEECH_ACTIONS_BT_H
#define SPEECH_ACTIONS_BT_H_
#include <behaviortree_cpp/behavior_tree.h>
#include <actionlib/client/simple_action_client.h>
#include <woz_dialog_msgs/SpeechGoal.h>
#include <woz_dialog_msgs/SpeechAction.h>
#include <woz_dialog_msgs/SpeechFeedback.h>

inline void SleepMS(int ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}



class SpeechActionBT : public BT::AsyncActionNode 
{
    public:
        SpeechActionBT(const std::string& name, const BT::NodeConfiguration& config, 
            actionlib::SimpleActionClient<woz_dialog_msgs::SpeechAction> *speech_client)
            : AsyncActionNode(name, config),
            _speech_client_PTR(speech_client)
        {
        }

        static BT::PortsList providedPorts()
        {
            return{BT::InputPort<std::string>("utterance"),
                   BT::InputPort<std::string>("voice"),
                   BT::InputPort<std::string>("language")};
        }

        BT::NodeStatus tick() override;

        virtual void halt() override;

    private:
        actionlib::SimpleActionClient<woz_dialog_msgs::SpeechAction> *_speech_client_PTR;
        std::atomic_bool _halt_requested;
};


#endif