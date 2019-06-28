#ifndef GAZE_ACTIONS_BT_HPP_
#define GAZE_ACTIONS_BT_HPP_

#include <behaviortree_cpp/behavior_tree.h>
#include <actionlib/client/simple_action_client.h>
#include "geometry_msgs/PoseStamped.h"
#include <vizzy_msgs/GazeAction.h>
#include <vizzy_msgs/GazeGoal.h>
#include <vizzy_behavior_trees/conversions.hpp>

typedef actionlib::SimpleActionClient<vizzy_msgs::GazeAction> GazeClient;


class GazeActionBT : public BT::AsyncActionNode
{
    public:

        /*This allows us to have multiple action names for
        the same kind of actionlib action (i.e. MoveBaseAction for more than one robot)
        and avoid creating duplicate action clients (i.e. using the same action in multiple
        parts of the behavior_tree).*/


        GazeActionBT(const std::string& name, const BT::NodeConfiguration& config)
            : AsyncActionNode(name, config)
        {
        }

        static BT::PortsList providedPorts()
        {
            return{BT::InputPort<geometry_msgs::PoseStamped>("fixation_pose"),
                   BT::InputPort<std::string>("frame_id"),
                   BT::InputPort<std::string>("action_name")};
        }

        BT::NodeStatus tick() override;

        virtual void halt() override;

    private:
        std::atomic_bool _halt_requested;
        static std::map<std::string, std::shared_ptr<GazeClient>> _gazeClients;
        static std::map<std::string, bool> _gazeClientsInitializing;
};


#endif
