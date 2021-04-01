#include <vizzy_behavior_trees/actions/arm_cartesian_actions.hpp>
#include "tf/transform_datatypes.h"
#include <math.h>
#include "behaviortree_cpp_v3/bt_factory.h"
#define MAX_REACH 0.6


BT::NodeStatus CartesianActionBT::tick()
{

    vizzy_msgs::CartesianGoal goal;

    BT::Optional<geometry_msgs::PoseStamped> pose = getInput<geometry_msgs::PoseStamped>("pose");
    BT::Optional<std::string> tipo = getInput<std::string>("type");
    BT::Optional<std::string> frame_id = getInput<std::string>("frame_id");
    BT::Optional<std::string> action_name = getInput<std::string>("action_name");

    if(!tipo)
    {
        throw BT::RuntimeError("missing required inputs [type]: ",
                                tipo.error() );
    }if(!pose)
    {
        throw BT::RuntimeError("missing required inputs [pose]: ",
                                pose.error() );
    }if (!frame_id)
    {
        throw BT::RuntimeError("missing required inputs [frame_id]: ",
                                frame_id.error() );

    }if (!action_name)
    {
        throw BT::RuntimeError("missing required inputs [action_name]: ",
                                action_name.error() );
    }

    if(client_PTR == NULL)
    {
        client_PTR = RosBlackBoard::getActionClientOrInit<CartesianClient>(action_name.value(), this); 
        if(client_PTR == NULL)
            return BT::NodeStatus::FAILURE;
    }

    geometry_msgs::PoseStamped poseStamped = pose.value();

    if(poseStamped.header.frame_id == "")
    {
        goal.end_effector_pose.header.frame_id = frame_id.value();
    }else
    {
        goal.end_effector_pose.header.frame_id = poseStamped.header.frame_id;
    }

    if(tipo.value() == "CARTESIAN")
    {

        goal.type = goal.CARTESIAN;

        bool left;



        //Transform it to the appropriate shoulder frame to make necessary computations
        //Maximum arm length approx 0.40 cm. This avoids strange orientations of the end effector

        geometry_msgs::TransformStamped transformStamped;
        geometry_msgs::PoseStamped coordsInShoulder;

        ROS_ERROR_STREAM("Action name:" << action_name.value());

        if(action_name.value() != "/vizzy/right_arm_cartesian_controller/cartesian_action" 
            && action_name.value() != "/vizzy/left_arm_cartesian_controller/cartesian_action"){

            goal.end_effector_pose.pose = poseStamped.pose;

        }else{

            if(action_name.value() == "/vizzy/left_arm_cartesian_controller/cartesian_action")
            {
                try{
                    left = true;
                    transformStamped = tfBuffer.lookupTransform("l_shoulder_abduction_link", 
                        goal.end_effector_pose.header.frame_id, ros::Time(0));
                    tf2::doTransform(poseStamped, coordsInShoulder, transformStamped);
                }
                catch (tf2::TransformException &ex) {
                    ROS_WARN("%s",ex.what());
                    return BT::NodeStatus::FAILURE;
                }

                goal.end_effector_pose.header.frame_id = "l_shoulder_abduction_link";

            }else if(action_name.value() == "/vizzy/right_arm_cartesian_controller/cartesian_action")
            {
                try{
                    left = false;
                    transformStamped = tfBuffer.lookupTransform("r_shoulder_abduction_link", 
                        goal.end_effector_pose.header.frame_id, ros::Time(0));
                    tf2::doTransform(poseStamped, coordsInShoulder, transformStamped);
                }
                catch (tf2::TransformException &ex) {
                    ROS_WARN("%s",ex.what());
                    return BT::NodeStatus::FAILURE;
                }

                goal.end_effector_pose.header.frame_id = "r_shoulder_abduction_link";

            }


            //Compute the norm to check out if the robot can reach the goal
            double x, y, z;
            x = coordsInShoulder.pose.position.x;
            y = coordsInShoulder.pose.position.y;
            z = coordsInShoulder.pose.position.z;

            double norma = sqrt(x*x+y*y+z*z);

            x = x/norma;
            y = y/norma;
            z = z/norma;

            //If the robot cannot reach it...
            if(norma > MAX_REACH)
            {
                coordsInShoulder.pose.position.x = x*MAX_REACH;
                coordsInShoulder.pose.position.y = y*MAX_REACH;
                coordsInShoulder.pose.position.z = z*MAX_REACH;
            }

            //Compute the orientation.
            //Using axis-angle is easier: compute cross product between x and the point.
            //Compute the angle
            tf2::Quaternion orientation;
            tf2::Vector3 pt(x, y, z);
            tf2::Vector3 uni_x(1, 0, 0);

            tf2::Vector3 axis = pt.cross(uni_x);
            axis.normalize();
            
            double angle = pt.angle(uni_x);

            orientation.setRotation(axis, -angle);

            //Now that x axis is aligned with the goal, rotate around it o make sure that the palm is pointing towards
            //the center of the robot (hand z axis is aligned with base footprint y). If it isn't, rotate around x to
            //do so.


            tf2::convert(orientation, goal.end_effector_pose.pose.orientation);


            //Temporary. Check that poseStamped is on base_footprint and just doTransform with the inverse
            
            if(poseStamped.header.frame_id == "base_footprint")
            {

                geometry_msgs::PoseStamped goal_pose_on_basefootprint;
                geometry_msgs::TransformStamped invTransformStamped;

                tf2::Stamped<tf2::Transform> tf2TransformStamped;
                tf2::fromMsg(transformStamped, tf2TransformStamped);

                auto tf2InverseTransform = tf2TransformStamped.inverse();

                tf2::Stamped<tf2::Transform> tfInvT;
                tfInvT.setData(tf2InverseTransform);

                invTransformStamped = tf2::toMsg(tfInvT);
                
                tf2::doTransform(goal.end_effector_pose, goal_pose_on_basefootprint, invTransformStamped);
                
                tf2::Transform tf_goal_to_base;

            }

            double rx;

            if(left)
                rx = 0;
            else
                rx = 0;

            tf2::Quaternion rot_x;
            rot_x.setRPY(rx, 0, 0);

            orientation = rot_x*orientation;
            orientation.normalize();
            goal.end_effector_pose.pose.position = coordsInShoulder.pose.position;

            tf2::convert(orientation, goal.end_effector_pose.pose.orientation);

            pub_fixed.publish(goal.end_effector_pose);
            pub_origintal.publish(poseStamped);

        }

    }else if(tipo.value() == "HOME"){
        goal.type = goal.HOME;
    }else if(tipo.value() == "GRAB"){
        goal.type = goal.GRAB;
    }else if(tipo.value() == "RELEASE"){
        goal.type = goal.RELEASE;
    }else if(tipo.value() == "POINT"){
        goal.type = goal.POINT;
    }

    std::cout << "[Cartesian]: Started." << std::endl <<
        "Goal: " << goal << std::endl;


    client_PTR->isServerConnected();

    client_PTR->sendGoal(goal);

    auto move_state = client_PTR->getState();


    while(!move_state.isDone())
    {
        move_state = client_PTR->getState();
        setStatusRunningAndYield();
    }

    cleanup(false);


    if(client_PTR->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        std::cout << "[Cartesian]: ENDED." << std::endl <<
            "SUCCESS" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }else{
        std::cout << "[Cartesian]: ENDED." << std::endl <<
            "FAILURE" << std::endl;
        return BT::NodeStatus::FAILURE;
    }


}

void CartesianActionBT::cleanup(bool halted)
{
    if(halted && client_PTR != NULL)
    {
            vizzy_msgs::CartesianGoal goal;
            goal.type = goal.PREEMPT;
            client_PTR->sendGoal(goal);
    }

}

void CartesianActionBT::halt()
{
    std::cout << name() <<": Halted." << std::endl;
    cleanup(true);
    CoroActionNode::halt();
}
