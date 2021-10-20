#include <vizzy_behavior_trees/actions/general.hpp>
#include "behaviortree_cpp_v3/bt_factory.h"




BT::NodeStatus GeneralActionBT::tick()
{

    auto Now = [](){ return std::chrono::high_resolution_clock::now(); };

    vizzy_behavior_trees::GeneralGoal goal;

    BT::Optional<std::string> action_name = getInput<std::string>("action_name");
    BT::Optional<std::string> constants = getInput<std::string>("constants");
    BT::Optional<std::string> variables = getInput<std::string>("variables");
    BT::Optional<std::string> result = getInput<std::string>("result");

    auto conf = config();

    std::string constants_string =  constants.value();

    //Remove spaces from string
    constants_string.erase(remove_if(constants_string.begin(), constants_string.end(), isspace), constants_string.end());

    size_t pos = 0;
    std::string token;
    std::string delimiter(",");

    if(constants_string.length() > 0)
    {
        while ((pos = constants_string.find(delimiter)) != std::string::npos) {
            token = constants_string.substr(0, pos);
            goal.constants.push_back(token);
            constants_string.erase(0, pos + delimiter.length());
        
        }
        goal.constants.push_back(constants_string);
    }


    std::string varsname_string = variables.value();
    std::vector<std::string> varsname_list;

    //Remove spaces from string
    varsname_string.erase(remove_if(varsname_string.begin(), varsname_string.end(), isspace), varsname_string.end());

    pos = 0;
    if(varsname_string.length() > 0)
    {
        while ((pos = varsname_string.find(delimiter)) != std::string::npos) {
            token = varsname_string.substr(0, pos);
            varsname_list.push_back(token);
            varsname_string.erase(0, pos + delimiter.length());
        
        }
        varsname_list.push_back(varsname_string);
    }



    for(auto varname : varsname_list)
    {

        std::string vstr;

        auto anyVar = conf.blackboard->getAny(varname);

        if(!anyVar || anyVar->empty())
        {
            ROS_WARN_STREAM("[BT " << this->name() << "]: Cannot find variable " << varname << ". Check your BT");
            return BT::NodeStatus::FAILURE;
        }

        if(anyVar->isNumber())
        {
            if(anyVar->type() ==  typeid(int64_t))
            {
                std::stringstream ss;
                ss << anyVar->cast<int64_t>();
                vstr = ss.str();

            }else if(anyVar->type() ==  typeid(uint64_t))
            {
                std::stringstream ss;
                ss << anyVar->cast<uint64_t>();
                vstr = ss.str();

            }else if(anyVar->type() == typeid(double))
            {
                std::stringstream ss;
                ss << anyVar->cast<double>();
                vstr = ss.str();
            }

        }else if(anyVar->isString())
        {
            vstr = anyVar->cast<SafeAny::SimpleString>().toStdString();

        }else{

            ROS_WARN_STREAM("[BT " << this->name() << "]: variable " << varname << "is neither number \
            or string. This node cannot process it :(");
            return BT::NodeStatus::FAILURE;

        }

        goal.variables.push_back(vstr);

    }

    if (!action_name)
    {
        throw BT::RuntimeError("missing required inputs [action_name]: ",
                                action_name.error() );
    }


    if(client_PTR == NULL) { 
        client_PTR = RosBlackBoard::getActionClientOrInit<GeneralActionClient>(action_name.value(), this);
        if(client_PTR == NULL)
            return BT::NodeStatus::FAILURE;
    }


    client_PTR->isServerConnected();
    client_PTR->sendGoal(goal);

    auto state = client_PTR->getState();


    while(!state.isDone())
    {
        state = client_PTR->getState();
        setStatusRunningAndYield();
    }

    cleanup(false);

    if(client_PTR->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        auto action_result = client_PTR->getResult();
        auto result = setOutput("result", action_result->result);
        
        return BT::NodeStatus::SUCCESS;
    }else{
        return BT::NodeStatus::FAILURE;
    }


    return BT::NodeStatus::SUCCESS;

}

void GeneralActionBT::cleanup(bool halted)
{
    if(halted && client_PTR != NULL)
    {
        client_PTR->cancelAllGoals();
    }

}

void GeneralActionBT::halt()
{
    std::cout << name() <<": Halted." << std::endl;
    cleanup(true);
    CoroActionNode::halt();
}



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

        return BT::NodeStatus::FAILURE;
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

        first_time = true;
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