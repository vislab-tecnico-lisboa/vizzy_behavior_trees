/*
  -----------------------------------------------------
  A template to create behavior tree actions to control ROS actions.
  Don't forget to replace ACTIONLIB_TEMPLATE with the name of your
  action!
*/

#ifndef ACTIONLIB_TEMPLATE_ACTIONS_BT_H_
#define ACTIONLIB_TEMPLATE_ACTIONS_BT_H_

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <actionlib/client/simple_action_client.h>
#include <vizzy_behavior_trees/util.hpp>
#include <vizzy_behavior_trees/conversions.hpp>
#include <map>
#include <vizzy_behavior_trees/rosbt_blackboard.hpp>

/*
Add your includes and action includes
*/

typedef actionlib::SimpleActionClient<YOUR_ACTION_SCOPE::ACTIONLIB_TEMPLATE> ACTIONLIB_TEMPLATE_Client;


class ACTIONLIB_TEMPLATE_BT : public BT::CoroActionNode
{
    public:

        /*This allows us to have multiple action names for
        the same kind of actionlib action (i.e. MoveBaseAction for more than one robot)
        and avoid creating duplicate action clients (i.e. using the same action in multiple
        parts of the behavior_tree).*/


        ACTIONLIB_TEMPLATE_Client* client_PTR;


        ACTIONLIB_TEMPLATE_BT(const std::string& name, const BT::NodeConfiguration& config)
            : CoroActionNode(name, config)
        {
            client_PTR = NULL;
        }

        static BT::PortsList providedPorts()
        {
            return{BT::InputPort<std::string>("action_name")
                   /*
                    Add extra input and output ports as you/the action needs
                    Note that,
                    ports can handle either strings, strings that represent
                    custom types (if you implemented their convertFromString method
                    as exemplified in conversions.hpp) or the type that you want!

                   ----- Example ports: -----

                   BT::InputPort<geometry_msgs::PoseStamped>("goal_pose"),
                   BT::InputPort<std::string>("frame_id"),
                   BT::OutputPort<geometry_msgs:Pose>("result")*/
                   };
        }

        BT::NodeStatus tick() override;
        void cleanup(bool halted);
        virtual void halt() override;

    private:
        std::atomic_bool _halt_requested;
        static std::map<std::string, std::shared_ptr<ACTIONLIB_TEMPLATE_Client>> _ACTIONLIB_TEMPLATE_Clients;
        static std::map<std::string, bool> _ACTIONLIB_TEMPLATE_ClientsInitializing;
};

#endif
