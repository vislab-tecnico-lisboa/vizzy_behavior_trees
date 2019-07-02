#ifndef GET_GEOMETRY_MSGS_HPP_
#define GET_GEOMETRY_MSGS_HPP_


#include <behaviortree_cpp/behavior_tree.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/callback_queue.h>

using namespace BT; 

//PoseArray
class GetPoseArrayBT : public SyncActionNode
{

    public:

        geometry_msgs::PoseArray poseList;
        ros::Subscriber sub_;
        ros::CallbackQueue queue_;

        GetPoseArrayBT(const std::string& name, const NodeConfiguration& config);

    static PortsList providedPorts()
    {
        return { InputPort<std::string>("topic"), 
            OutputPort<geometry_msgs::PoseArray>("pose_array")};
    }

    NodeStatus tick() override;
    void callback(const geometry_msgs::PoseArray::ConstPtr &msg);

    

};


//SelectPoseStamped from a PoseArray
class SelectPoseStamped : public AsyncActionNode
{

    public:
        geometry_msgs::PoseStamped pose;

        SelectPoseStamped(const std::string& string, const NodeConfiguration& config)
        : AsyncActionNode(string, config){}


        static PortsList providedPorts()
        {
            return { InputPort<std::string>("position_in_list"), 
                InputPort<geometry_msgs::PoseArray>("pose_array"),
                OutputPort<geometry_msgs::PoseStamped>("pose_stamped")};
        }

        NodeStatus tick() override;
        void halt() override {};
};

#endif