#include <vizzy_behavior_trees/actions/general.hpp>


BT::NodeStatus TimerAction::tick()
{
    int time_d;

    BT::Optional<std::string> time = getInput<std::string>("s_between_success");

    if(!time)
    {
        throw BT::RuntimeError("missing required inputs [ms_between_success]: ",
                                   time.error() );
    }

    time_d = std::atof(time.value().c_str())*1000;


    if(first_time)
    {
        first_time = false;
        TimePoint initial_time = RosBlackBoard::Now();
        timeout = initial_time + std::chrono::milliseconds(time_d);
        previous_time_d = time_d;

        return BT::NodeStatus::SUCCESS;
    }

    if(previous_time_d != time_d)
    {
        TimePoint initial_time = RosBlackBoard::Now();
        timeout = initial_time + std::chrono::milliseconds(time_d);
        previous_time_d = time_d;
    }

    if(RosBlackBoard::Now() < timeout)
    {
        return BT::NodeStatus::FAILURE;

    }else{

        TimePoint initial_time = RosBlackBoard::Now();
        timeout = initial_time + std::chrono::milliseconds(time_d);
        return BT::NodeStatus::SUCCESS;
    }

}

BT::NodeStatus DebugAction::tick()
{
    BT::Optional<std::string> toPrint = getInput<std::string>("string");

    if(!toPrint)
    {
        throw BT::RuntimeError("missing required inputs [string]: ",
                                   toPrint.error() );
    }

    std::cout << toPrint.value() << std::endl;

    return BT::NodeStatus::SUCCESS;
}



//WaitForXSeconds
BT::NodeStatus WaitForXSeconds::tick()
{
    int time_d;

    BT::Optional<std::string> time = getInput<std::string>("seconds");
    BT::Optional<std::string> result = getInput<std::string>("result");

    if(!time)
    {
        throw BT::RuntimeError("missing required inputs [time]: ",
                                   time.error() );
    }if (!result)
    {
        throw BT::RuntimeError("missing required inputs [result]: ",
                                   result.error() );
    }

    _halt_requested.store(false);

    double current = 0;

    time_d = std::atof(time.value().c_str())*1000;


    TimePoint initial_time = RosBlackBoard::Now();
    TimePoint timeout = initial_time + std::chrono::milliseconds(time_d);

    while(RosBlackBoard::Now() < timeout)
    {
        setStatusRunningAndYield();
    }


    if(result.value() == "SUCCESS")
    {
        return BT::NodeStatus::SUCCESS;
    }else{
        return BT::NodeStatus::FAILURE;
    }

}

void WaitForXSeconds::cleanup(bool halted)
{
    if(halted)
    {
        //Preempt all actions here

    }
}

void WaitForXSeconds::halt()
{
    cleanup(true);
    CoroActionNode::halt();
}