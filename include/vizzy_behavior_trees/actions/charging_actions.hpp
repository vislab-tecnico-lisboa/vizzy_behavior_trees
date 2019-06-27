#ifndef CHARGING_ACTIONS_BT_HPP_
#define CHARGING_ACTIONS_BT_HPP_ 

#include <vizzy_msgs/ChargeAction.h>
#include <vizzy_msgs/ChargeGoal.h>
#include <actionlib/client/simple_action_client.h>
#include <behaviortree_cpp/behavior_tree.h>
#include <vizzy_behavior_trees/util.hpp>

typedef actionlib::SimpleActionClient<vizzy_msgs::ChargeAction> ChargeClient;

class ChargeActionBT : public BT::AsyncActionNode 
{
    public:

        /*This allows us to have multiple action names for
        the same kind of actionlib action (i.e. ChargeAction for more than one robot)
        and avoid creating duplicate action clients (i.e. using the same action in multiple
        parts of the behavior_tree).*/


        ChargeActionBT(const std::string& name, const BT::NodeConfiguration& config)
            : AsyncActionNode(name, config)
        {
        }

        static BT::PortsList providedPorts()
        {
            return{BT::InputPort<std::string>("action_name")};
                   
        }

        BT::NodeStatus tick() override;

        virtual void halt() override;

    private:
        std::atomic_bool _halt_requested;
        static std::map<std::string, std::shared_ptr<ChargeClient>> _chargeClients;
        static std::map<std::string, bool> _chargeClientsInitializing;
};


#endif