#include <vizzy_behavior_trees/actions/charging_actions.hpp>
#include "behaviortree_cpp_v3/bt_factory.h"



std::map<std::string, std::shared_ptr<ChargeClient>> ChargeActionBT::_chargeClients;
std::map<std::string, bool> ChargeActionBT::_chargeClientsInitializing;

BT::NodeStatus ChargeActionBT::tick()
{

    vizzy_msgs::ChargeGoal goal;

    BT::Optional<std::string> action_name = getInput<std::string>("action_name");
    BT::Optional<std::string> action = getInput<std::string>("action");


    if (!action_name)
    {
        throw BT::RuntimeError("missing required inputs [action_name]: ",
                                action_name.error() );
    }
    if (!action)
    {
        throw BT::RuntimeError("missing required inputs [action]: ",
                                action.error() );
    }

    if(action.value() == "CHARGE")
    {
        goal.goal = goal.CHARGE;
    }else if(action.value() == "STOP_CHARGE")
    {
        goal.goal = goal.STOP_CHARGE;
    }else{
        return BT::NodeStatus::FAILURE;
    }

    if(client_PTR == NULL)
    {
        client_PTR = RosBlackBoard::getActionClientOrInit<ChargeClient>(action_name.value(), this);
        if(client_PTR == NULL)
            return BT::NodeStatus::FAILURE;
    }


    std::cout << "[Charge]: Started." << std::endl <<
        "Pose: " << goal << std::endl;

    _halt_requested.store(false);

    client_PTR->sendGoal(goal);

    auto charge_state = client_PTR->getState();

    while(!charge_state.isDone())
    {
        charge_state = client_PTR->getState();
        setStatusRunningAndYield();
    }

    cleanup(false);

    if(client_PTR->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        return BT::NodeStatus::SUCCESS;
    }else{
        return BT::NodeStatus::FAILURE;
    }

}

void ChargeActionBT::cleanup(bool halted)
{
    if(halted)
    {
        client_PTR->cancelAllGoals();

    }

}

void ChargeActionBT::halt()
{
    std::cout << name() <<": Halted." << std::endl;
    cleanup(true);
    CoroActionNode::halt();
}

//CheckBattery
BT::NodeStatus CheckBatteryBT::tick()
{
    vizzy_msgs::BatteryState srv;
    
    if(!client.call(srv))
    {
        return BT::NodeStatus::FAILURE;
    }else{

        int battery_state = srv.response.battery_state;
        double percentage = srv.response.percentage;

        setOutput("battery_state", battery_state);
        setOutput("percentage", percentage);
    }

}

//CheckCharging

BT::NodeStatus CheckChargingBT::tick()
{
    vizzy_msgs::BatteryChargingState srv;

    if(!client.call(srv))
    {
        return BT::NodeStatus::FAILURE;
    }else{

        int battery_state = srv.response.battery_charging_state;

        setOutput("charging_state", battery_state);
    }
}