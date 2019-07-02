#include <vizzy_behavior_trees/conditions/general.hpp>


BT::NodeStatus CompareInt::tick()
{
    auto comp = getInput<std::string>("Condition");
    auto a = getInput<int>("A");
    auto b = getInput<int>("B");


    if(!comp)
    {
        throw BT::RuntimeError("missing required inputs [Condition]: ",
                                comp.error()); 
    }else if(comp.value() != "<" &&
             comp.value() != "<=" &&
             comp.value() != "==" &&
             comp.value() != ">=" &&
             comp.value() != ">")
             {
                 throw BT::RuntimeError("invalid comparison. Possible comparison operators: <, <=, ==, >=, >");
             }
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


    if(comp.value() == "<")
    {
        if(a < b)
        {
            return BT::NodeStatus::SUCCESS;
        }else{
            return BT::NodeStatus::FAILURE;
        }
    }else if(comp.value() == "<=")
    {
        if(a <= b)
        {
            return BT::NodeStatus::SUCCESS;
        }else{
            return BT::NodeStatus::FAILURE;
        }
    }else if(comp.value() == "==")
    {
        if(a == b)
        {
            return BT::NodeStatus::SUCCESS;
        }else{
            return BT::NodeStatus::FAILURE;}
    }else if(comp.value() == ">=")
    {
        if(a>=b){
            return BT::NodeStatus::SUCCESS;
        }else{
            return BT::NodeStatus::FAILURE;
        }
    }else if(comp.value() == ">")
    {
        if(a>b){
            return BT::NodeStatus::SUCCESS;
        }else{
            return BT::NodeStatus::FAILURE;
        }
    }else{
        return BT::NodeStatus::FAILURE;
    }

}


BT::NodeStatus CompareDouble::tick()
{
    auto comp = getInput<std::string>("Condition");
    auto a = getInput<int>("A");
    auto b = getInput<int>("B");


    if(!comp)
    {
        throw BT::RuntimeError("missing required inputs [Condition]: ",
                                comp.error()); 
    }else if(comp.value() != "<" &&
             comp.value() != "<=" &&
             comp.value() != "==" &&
             comp.value() != ">=" &&
             comp.value() != ">")
             {
                 throw BT::RuntimeError("invalid comparison. Possible comparison operators: <, <=, ==, >=, >");
             }
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


    if(comp.value() == "<")
    {
        if(a < b)
        {
            return BT::NodeStatus::SUCCESS;
        }else{
            return BT::NodeStatus::FAILURE;
        }
    }else if(comp.value() == "<=")
    {
        if(a <= b)
        {
            return BT::NodeStatus::SUCCESS;
        }else{
            return BT::NodeStatus::FAILURE;
        }
    }else if(comp.value() == "==")
    {
        if(a == b)
        {
            return BT::NodeStatus::SUCCESS;
        }else{
            return BT::NodeStatus::FAILURE;}
    }else if(comp.value() == ">=")
    {
        if(a>=b){
            return BT::NodeStatus::SUCCESS;
        }else{
            return BT::NodeStatus::FAILURE;
        }
    }else if(comp.value() == ">")
    {
        if(a>b){
            return BT::NodeStatus::SUCCESS;
        }else{
            return BT::NodeStatus::FAILURE;
        }
    }else{
        return BT::NodeStatus::FAILURE;
    }

}