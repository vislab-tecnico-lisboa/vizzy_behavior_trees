#ifndef GENERAL_CONDITIONS_HPP_
#define GENERAL_CONDITIONS_HPP_
#include <behaviortree_cpp/behavior_tree.h>

class BiggerThan : public BT::ConditionNode
{
    public:
        BiggerThan(const std::string& name, const BT::NodeConfiguration& config)
        : ConditionNode(name, config)
        {}

        static BT::PortsList providedPorts()
        {
            return{BT::InputPort<std::string>("A"),
                   BT::InputPort<std::string>("B")};
        }


        BT::NodeStatus tick() override;

};

#endif