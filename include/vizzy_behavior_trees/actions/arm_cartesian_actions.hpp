#ifndef ARM_CARTESIAN_ACTIONS_BT_HPP_
#define ARM_CARTESIAN_ACTIONS_BT_HPP_

#include <behaviortree_cpp/behavior_tree.h>
#include <actionlib/client/simple_action_client.h>
#include "geometry_msgs/PoseStamped.h"
#include <vizzy_msgs/CartesianAction.h>
#include <vizzy_msgs/CartesianGoal.h>
#include <vizzy_behavior_trees/util.hpp>
#include <vizzy_behavior_trees/conversions.hpp>


typedef actionlib::SimpleActionClient<vizzy_msgs::CartesianAction> CartesianClient;


class CartesianActionBT : public BT::AsyncActionNode
{
    public:

        /*This allows us to have multiple action names for
        the same kind of actionlib action (i.e. CartesianAction for more than one robot)
        and avoid creating duplicate action clients (i.e. using the same action in multiple
        parts of the behavior_tree).*/


        CartesianActionBT(const std::string& name, const BT::NodeConfiguration& config)
            : AsyncActionNode(name, config)
        {
        }

        static BT::PortsList providedPorts()
        {
            return{BT::InputPort<std::string>("type"),
                   BT::InputPort<geometry_msgs::PoseStamped>("pose"),
                   BT::InputPort<std::string>("frame_id"),
                   BT::InputPort<std::string>("action_name")};
        }

        BT::NodeStatus tick() override;

        virtual void halt() override;

    private:
        std::atomic_bool _halt_requested;
        static std::map<std::string, std::shared_ptr<CartesianClient>> _cartesianClients;
        static std::map<std::string, bool> _cartesianClientsInitializing;
};


#endif
