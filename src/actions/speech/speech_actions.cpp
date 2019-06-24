#include <vizzy_behavior_trees/speech_actions.hpp>
#include <behaviortree_cpp/bt_factory.h>


BT::NodeStatus SpeechActionBT::tick()
{
    woz_dialog_msgs::SpeechGoal goal;

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
    }

    goal.language = lang.value();
    goal.message = msg.value();
    goal.voice = voice.value();


    std::cout << "[Speech]: Started." << std::endl <<
        "Message: " << goal.message << std::endl <<
        "Voice: " << goal.voice << std::endl << 
        "Language: " << goal.language << std::endl;

    _halt_requested.store(false);

    _speech_client_PTR->sendGoalAndWait(goal);


    auto result = _speech_client_PTR->getResult();


    if(_halt_requested)
    {
        _speech_client_PTR->cancelAllGoals();
        return BT::NodeStatus::FAILURE;
    }

    if(result->success)
    {
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        return BT::NodeStatus::FAILURE;
    }

}

void SpeechActionBT::halt()
{
    _halt_requested.store(true);
}