#ifndef GENERAL_CONDITIONS_HPP_
#define GENERAL_CONDITIONS_HPP_
#include <behaviortree_cpp_v3/behavior_tree.h>

class CompareInt : public BT::ConditionNode
{
    public:
        CompareInt(const std::string& name, const BT::NodeConfiguration& config)
        : ConditionNode(name, config)
        {}

        static BT::PortsList providedPorts()
        {
            return{BT::InputPort<std::string>("Condition"),
                   BT::InputPort<int>("A"),
                   BT::InputPort<int>("B")};
        }


        BT::NodeStatus tick() override;

};

class CompareDouble : public BT::ConditionNode
{
    public:
        CompareDouble(const std::string& name, const BT::NodeConfiguration& config)
        : ConditionNode(name, config)
        {}

        static BT::PortsList providedPorts()
        {
            return{BT::InputPort<std::string>("Condition"),
                   BT::InputPort<double>("A"),
                   BT::InputPort<double>("B")};
        }


        BT::NodeStatus tick() override;

};

class CheckBool : public BT::ConditionNode
{
    public:
        CheckBool(const std::string& name, const BT::NodeConfiguration& config)
        : ConditionNode(name, config)
        {}

        static BT::PortsList providedPorts()
        {
            return{BT::InputPort<std::string>("variable")};
        }


        BT::NodeStatus tick() override;

};


#endif