#ifndef MOVE_BASE_ACTIONS_BT_H_
#define MOVE_BASE_ACTIONS_BT_H_

#include <behaviortree_cpp/behavior_tree.h>
#include <actionlib/client/simple_action_client.h>
#include "geometry_msgs/PoseStamped.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseResult.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <vizzy_behavior_trees/util.hpp>
#include <map>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

namespace BT
{
    template <> inline geometry_msgs::PoseStamped convertFromString(StringView str)
    {

        // We expect real numbers separated by semicolons
        auto parts = splitString(str, ';');
        if (parts.size() != 7)
        {
            throw RuntimeError("Invalid input. It should be: x;y;z;qx;qy;qz;qw");
        }
        else{
            geometry_msgs::PoseStamped output;
            output.pose.position.x =  convertFromString<double>(parts[0]);
            output.pose.position.y =  convertFromString<double>(parts[1]);
            output.pose.position.z =  convertFromString<double>(parts[2]);
            output.pose.orientation.x =  convertFromString<double>(parts[3]);
            output.pose.orientation.y =  convertFromString<double>(parts[4]);
            output.pose.orientation.z =  convertFromString<double>(parts[5]);
            output.pose.orientation.w =  convertFromString<double>(parts[6]);

            return output;
        }
    }
} // end namespace BT

class MoveBaseActionBT : public BT::AsyncActionNode 
{
    public:

        /*This allows us to have multiple action names for
        the same kind of actionlib action (i.e. MoveBaseAction for more than one robot)
        and avoid creating duplicate action clients (i.e. using the same action in multiple
        parts of the behavior_tree).*/


        MoveBaseActionBT(const std::string& name, const BT::NodeConfiguration& config)
            : AsyncActionNode(name, config)
        {
        }

        static BT::PortsList providedPorts()
        {
            return{BT::InputPort<geometry_msgs::PoseStamped>("goal_pose"),
                   BT::InputPort<std::string>("frame_id"),
                   BT::InputPort<std::string>("action_name")};
        }

        BT::NodeStatus tick() override;

        virtual void halt() override;

    private:
        MoveBaseClient *_movebase_client_PTR;
        std::atomic_bool _halt_requested;
        static std::map<std::string, std::shared_ptr<MoveBaseClient>> _moveBaseClients;
        static std::map<std::string, bool> _moveBaseClientsInitializing;
};

#endif
