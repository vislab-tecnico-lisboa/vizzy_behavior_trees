#include <vizzy_behavior_trees/actions/general.hpp>


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
    double time_d;

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

    time_d = std::atof(time.value().c_str());

    setStatus(BT::NodeStatus::RUNNING);

    while(current<time_d)
    {
        SleepMS(10);
        current+=0.01;
        if(_halt_requested)
        {
            std::cout << "Halt requested!" << std::endl;
            return BT::NodeStatus::FAILURE;
        }
    }

    if(result.value() == "SUCCESS")
    {
        return BT::NodeStatus::SUCCESS;
    }else{
        return BT::NodeStatus::FAILURE;
    }

}

void WaitForXSeconds::halt()
{
    _halt_requested.store(true);
}
