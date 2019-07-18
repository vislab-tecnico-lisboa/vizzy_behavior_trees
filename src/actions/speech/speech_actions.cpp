#include <vizzy_behavior_trees/actions/speech_actions.hpp>

std::map<std::string, std::shared_ptr<SpeechClient>> SpeechActionBT::_speechClients;
std::map<std::string, bool> SpeechActionBT::_speechClientsInitializing;

BT::NodeStatus SpeechActionBT::tick()
{
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


    /*Check if the action client is registered in the speechClient map
    If it isn't register it. This allows us to have multiple actions for
    the same kind of actionlib (i.e. Speech) and avoid creating
    duplicate action clients.*/


    if(client_PTR == NULL)
    {
        auto action_client_pair = _speechClients.find(action_name.value());
        auto init_pair = _speechClientsInitializing.find(action_name.value());

        if(init_pair != _speechClientsInitializing.end())
        {
            if(init_pair->second)
            {
                return BT::NodeStatus::FAILURE;
            }
        }


        if(action_client_pair == _speechClients.end())
        {
            //Create action client and add it to the list of all clients

            client_PTR = std::make_shared<SpeechClient>(action_name.value());

            ROS_INFO_STREAM("Waiting for action server of: " << action_name.value());

            _speechClientsInitializing[action_name.value()] = true;

            if(!client_PTR->waitForServer(ros::Duration(1)))
            {
                ROS_WARN_STREAM("Could not connect to action server: " << action_name.value());
                _speechClients.erase(action_name.value());
                _speechClientsInitializing.erase(action_name.value());
                return BT::NodeStatus::FAILURE;
            }

            _speechClientsInitializing[action_name.value()] = false;

            ROS_INFO_STREAM("Found action server of: " << action_name.value());
            _speechClients[action_name.value()] = client_PTR;
            ROS_INFO_STREAM("Number of move_base clients: " << _speechClients.size());

        }else
        {
            client_PTR = action_client_pair->second;
        }
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
    SpeechActionBT::halt();
}
