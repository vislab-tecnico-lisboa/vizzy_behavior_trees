#include <vizzy_behavior_trees/actions/charging_actions.hpp>

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




    /*Check if the action client is registered in the chargeClient map
    If it isn't register it. This allows us to have multiple actions for
    the same kind of actionlib (i.e. ChargeAction) and avoid creating
    duplicate action clients.*/

    if(client_PTR == NULL) 
    {
        auto action_client_pair = _chargeClients.find(action_name.value());
        auto init_pair = _chargeClientsInitializing.find(action_name.value());

        if(init_pair != _chargeClientsInitializing.end())
        {
            if(init_pair->second)
            {
                return BT::NodeStatus::FAILURE;
            }
        }

        if(action_client_pair == _chargeClients.end())
        {
            //Create action client and add it to the list of all clients

            client_PTR = std::make_shared<ChargeClient>(action_name.value());

            ROS_INFO_STREAM("Waiting for action server of: " << action_name.value());

            _chargeClientsInitializing[action_name.value()] = true;

            if(!client_PTR->waitForServer(ros::Duration(1)))
            {
                ROS_WARN_STREAM("Could not connect to action server: " << action_name.value());
                _chargeClients.erase(action_name.value());
                _chargeClientsInitializing.erase(action_name.value());
                return BT::NodeStatus::FAILURE;
            }

            _chargeClientsInitializing[action_name.value()] = false;

            ROS_INFO_STREAM("Found action server of: " << action_name.value());
            _chargeClients[action_name.value()] = client_PTR;
            ROS_INFO_STREAM("Number of move_base clients: " << _chargeClients.size());

        }else
        {
            client_PTR = action_client_pair->second;
        }

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
    ChargeActionBT::halt();
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