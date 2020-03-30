/*
  -----------------------------------------------------
  A template to create behavior tree actions to publish to ROS topics.
  Don't forget to replace PUBLISHER_TEMPLATE_BT with the name of your
  action!
*/

#ifndef PUBLISHER_TEMPLATE_ACTIONS_HPP_
#define PUBLISHER_TEMPLATE_ACTIONS_HPP_

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <vizzy_behavior_trees/util.hpp>
#include <ros/ros.h>
#include <vizzy_behavior_trees/rosbt_blackboard.hpp>


/*Add the necessary publisher includes here*/
/*--- Example --- */
/*#include <std_msgs/Float64.h>*/

using namespace BT;

class PUBLISHER_TEMPLATE_BT : public BT::SyncActionNode
{
    public:
        PUBLISHER_TEMPLATE_BT(const std::string& name, const BT::NodeConfiguration& config);

        /*Some needed class attributes here*/

        static BT::PortsList providedPorts()
        {
            return{BT::InputPort<std::string>("topic")
                   /*Some extra ports here*/
                   /*BT::InputPort<double>("angle"),
                     BT::InputPort<int>("make_special")*/
                   };
        }

        ros::NodeHandle nh_;

        BT::NodeStatus tick() override;

    private:
        static std::map<std::string, ros::Publisher> _publishers;
};


#endif
