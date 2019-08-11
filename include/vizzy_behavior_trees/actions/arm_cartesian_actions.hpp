#ifndef ARM_CARTESIAN_ACTIONS_BT_HPP_
#define ARM_CARTESIAN_ACTIONS_BT_HPP_

#include <behaviortree_cpp/behavior_tree.h>
#include <actionlib/client/simple_action_client.h>
#include "geometry_msgs/PoseStamped.h"
#include <vizzy_msgs/CartesianAction.h>
#include <vizzy_msgs/CartesianGoal.h>
#include <vizzy_behavior_trees/util.hpp>
#include <vizzy_behavior_trees/conversions.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vizzy_behavior_trees/rosbt_blackboard.hpp>


typedef actionlib::SimpleActionClient<vizzy_msgs::CartesianAction> CartesianClient;


class CartesianActionBT : public BT::CoroActionNode
{
    public:

        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener;

        //DEBUG
        ros::Publisher pub_origintal;
        ros::Publisher pub_fixed;


        CartesianClient* client_PTR;

        CartesianActionBT(const std::string& name, const BT::NodeConfiguration& config)
            : CoroActionNode(name, config), tfListener(tfBuffer)
        {
            ros::NodeHandle nh;
            pub_origintal = nh.advertise<geometry_msgs::PoseStamped>("original", 1);
            pub_fixed = nh.advertise<geometry_msgs::PoseStamped>("fixed", 1);
            client_PTR = NULL;
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
        void cleanup(bool halted);

    private:
        std::atomic_bool _halt_requested;
};


#endif
