#ifndef CHARGING_ACTIONS_BT_HPP_
#define CHARGING_ACTIONS_BT_HPP_

#include <vizzy_msgs/ChargeAction.h>
#include <vizzy_msgs/ChargeGoal.h>
#include <vizzy_msgs/BatteryState.h>
#include <vizzy_msgs/BatteryStateRequest.h>
#include <vizzy_msgs/BatteryStateResponse.h>
#include <vizzy_msgs/BatteryChargingState.h>
#include <vizzy_msgs/BatteryChargingStateRequest.h>
#include <vizzy_msgs/BatteryChargingStateResponse.h>
#include <actionlib/client/simple_action_client.h>
#include <behaviortree_cpp/behavior_tree.h>
#include <vizzy_behavior_trees/util.hpp>
#include <vizzy_behavior_trees/rosbt_blackboard.hpp>

typedef actionlib::SimpleActionClient<vizzy_msgs::ChargeAction> ChargeClient;

class ChargeActionBT : public BT::CoroActionNode
{
    public:

        /*This allows us to have multiple action names for
        the same kind of actionlib action (i.e. ChargeAction for more than one robot)
        and avoid creating duplicate action clients (i.e. using the same action in multiple
        parts of the behavior_tree).*/


        ChargeClient* client_PTR;


        ChargeActionBT(const std::string& name, const BT::NodeConfiguration& config)
            : CoroActionNode(name, config)
        {
            client_PTR = NULL;
        }

        static BT::PortsList providedPorts()
        {
            return{BT::InputPort<std::string>("action_name"),
                   BT::InputPort<std::string>("action")};
        }

        BT::NodeStatus tick() override;
        void cleanup(bool halted);
        virtual void halt() override;

    private:
        std::atomic_bool _halt_requested;
        static std::map<std::string, std::shared_ptr<ChargeClient>> _chargeClients;
        static std::map<std::string, bool> _chargeClientsInitializing;
};


class CheckBatteryBT : public BT::SyncActionNode
{
    public:


        CheckBatteryBT(const std::string& name, const BT::NodeConfiguration& config)
            : SyncActionNode(name, config)
        {

            BT::Optional<std::string> service_name = getInput<std::string>("service_name");


            if (!service_name)
            {
                throw BT::RuntimeError("missing required inputs [service_name]: ",
                                        service_name.error() );
            }

            client = nh.serviceClient<vizzy_msgs::BatteryState>(service_name.value());

        }

        static BT::PortsList providedPorts()
        {
            return{BT::InputPort<std::string>("service_name"),
                   BT::OutputPort<int>("battery_state"),
                   BT::OutputPort<double>("battery_percentage")};
        }

        ros::NodeHandle nh;
        ros::ServiceClient client;

        BT::NodeStatus tick() override;

};



class CheckChargingBT : public BT::SyncActionNode
{
    public:


        CheckChargingBT(const std::string& name, const BT::NodeConfiguration& config)
            : SyncActionNode(name, config)
        {

            BT::Optional<std::string> service_name = getInput<std::string>("service_name");


            if (!service_name)
            {
                throw BT::RuntimeError("missing required inputs [service_name]: ",
                                        service_name.error() );
            }

            client = nh.serviceClient<vizzy_msgs::BatteryChargingState>(service_name.value());

        }

        static BT::PortsList providedPorts()
        {
            return{BT::InputPort<std::string>("service_name"),
                   BT::OutputPort<int>("charging_state")};
        }

        ros::NodeHandle nh;
        ros::ServiceClient client;

        BT::NodeStatus tick() override;

};
#endif
