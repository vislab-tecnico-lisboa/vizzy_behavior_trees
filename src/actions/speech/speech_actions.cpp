#include <vizzy_behavior_trees/actions/speech_actions.hpp>
#include "behaviortree_cpp_v3/bt_factory.h"



BT::NodeStatus SpeechActionBT::tick()
{
    auto Now = [](){ return std::chrono::high_resolution_clock::now(); };

    woz_dialog_msgs::SpeechGoal goal;

    BT::Optional<std::string> action_name = getInput<std::string>("action_name");
    BT::Optional<std::string> msg = getInput<std::string>("utterance");
    BT::Optional<std::string> lang = getInput<std::string>("language");
    BT::Optional<std::string> voice = getInput<std::string>("voice");

    if(!msg)
    {
        throw BT::RuntimeError("missing required inputs [utterance]: ",
                                   msg.error() );
    }if (!lang)
    {
        throw BT::RuntimeError("missing required inputs [language]: ",
                                   lang.error() );

    }if (!voice)
    {
        throw BT::RuntimeError("missing required inputs [voice]: ", 
                                   voice.error() );
    }if (!action_name)
    {
        throw BT::RuntimeError("missing required inputs [action_name]: ", 
                                   voice.error() );
    }


    if(client_PTR == NULL)
    {
        client_PTR = RosBlackBoard::getActionClientOrInit<SpeechClient>(action_name.value(), this);
        if(client_PTR == NULL)
            return BT::NodeStatus::FAILURE;
    }

    goal.language = lang.value();
    goal.message = msg.value();
    goal.voice = voice.value();
    goal.speed = goal.MEDIUM;


    std::cout << "[Speech]: Started." << std::endl <<
        "Message: " << goal.message << std::endl <<
        "Voice: " << goal.voice << std::endl <<
        "Language: " << goal.language << std::endl;

    _halt_requested.store(false);

    client_PTR->sendGoalAndWait(goal);

    auto speech_state = client_PTR->getState();

    while(!speech_state.isDone())
    {
        speech_state = client_PTR->getState();
        setStatusRunningAndYield();
    }

    auto result = client_PTR->getResult();

    cleanup(false);

    if(result->success)
    {
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        return BT::NodeStatus::FAILURE;
    }

}

void SpeechActionBT::cleanup(bool halted)
{
    if(halted)
    {
        client_PTR->cancelAllGoals();
    }
}

void SpeechActionBT::halt()
{
    std::cout << name() <<": Halted." << std::endl;
    cleanup(true);
    CoroActionNode::halt();
}
