#ifndef GAZE_ACTIONS_BT_HPP_
#define GAZE_ACTIONS_BT_HPP_

#include <behaviortree_cpp/behavior_tree.h>
#include <actionlib/client/simple_action_client.h>
#include "geometry_msgs/PoseStamped.h"
#include <vizzy_msgs/GazeAction.h>
#include <vizzy_msgs/GazeGoal.h>
#include <vizzy_behavior_trees/conversions.hpp>
#include <vizzy_behavior_trees/rosbt_blackboard.hpp>

typedef actionlib::SimpleActionClient<vizzy_msgs::GazeAction> GazeClient;


class GazeActionBT : public BT::CoroActionNode
{
    public:

        GazeClient* client_PTR;

        GazeActionBT(const std::string& name, const BT::NodeConfiguration& config)
            : CoroActionNode(name, config)
        {
            client_PTR = NULL;
        }

        static BT::PortsList providedPorts()
        {
            return{BT::InputPort<geometry_msgs::PoseStamped>("fixation_pose"),
                   BT::InputPort<std::string>("frame_id"),
                   BT::InputPort<std::string>("action_name")};
        }

        BT::NodeStatus tick() override;
        virtual void halt() override;
        void cleanup(bool halted);

    private:
        std::atomic_bool _halt_requested;
};


#endif
