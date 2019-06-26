#include <vizzy_behavior_trees/conditions/general.hpp>


BT::NodeStatus BiggerThan::tick()
{
    auto a = getInput<std::string>("A");
    auto b = getInput<std::string>("B");


    if(!a)
    {
        throw BT::RuntimeError("missing required inputs [A]: ",
                                a.error()); 
    }
    if(!b)
    {
        throw BT::RuntimeError("missing required inputs [B]: ",
                                b.error()); 
    }

    double a_d = std::atof(a.value().c_str());
    double b_d = std::atof(b.value().c_str());

    if(a_d>b_d)
        return BT::NodeStatus::SUCCESS;
    else
        return BT::NodeStatus::FAILURE;


}