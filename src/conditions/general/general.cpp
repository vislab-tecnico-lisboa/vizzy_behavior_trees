#include <vizzy_behavior_trees/conditions/general.hpp>
#include "behaviortree_cpp_v3/bt_factory.h"



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
             comp.value() != ">" &&
             comp.value() != "!=")
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
    }
    else if(comp.value() == "!=")
    {
        if (a!=b)
        {
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
    auto a = getInput<double>("A");
    auto b = getInput<double>("B");


    if(!comp)
    {
        throw BT::RuntimeError("missing required inputs [Condition]: ",
                                comp.error()); 
    }else if(comp.value() != "<" &&
             comp.value() != "<=" &&
             comp.value() != "==" &&
             comp.value() != ">=" &&
             comp.value() != ">" &&
             comp.value() != "!=")
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
    }
    else if(comp.value() == "!=")
    {
        if (a!=b)
        {
            return BT::NodeStatus::SUCCESS;
        }else{
            return BT::NodeStatus::FAILURE;
        }

    }else{
        return BT::NodeStatus::FAILURE;
    }

}

BT::NodeStatus CheckBool::tick()
{
    auto var = getInput<bool>("variable");

    if(!var)
    {
        throw BT::RuntimeError("missing required inputs [variable]: ",
                                var.error()); 
    }

    if(var.value())
    {
        return BT::NodeStatus::SUCCESS;
    }else
    {
        return BT::NodeStatus::FAILURE;
    }
    
}