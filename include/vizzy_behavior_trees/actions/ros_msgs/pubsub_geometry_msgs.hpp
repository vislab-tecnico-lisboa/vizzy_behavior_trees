#ifndef PUBSUB_GEOMETRY_MSGS_HPP_
#define PUBSUB_GEOMETRY_MSGS_HPP_


#include <behaviortree_cpp_v3/behavior_tree.h>
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


//SelectPose from a PoseArray
class SelectPose : public SyncActionNode
{

    public:
        geometry_msgs::PoseStamped pose;

        SelectPose(const std::string& string, const NodeConfiguration& config)
        : SyncActionNode(string, config){}


        static PortsList providedPorts()
        {
            return { InputPort<std::string>("position_in_list"), 
                InputPort<geometry_msgs::PoseArray>("pose_array"),
                OutputPort<geometry_msgs::PoseStamped>("pose_stamped")};
        }

        NodeStatus tick() override;
};

//Select field from PoseStamped
class SelectFieldFromPoseStamped : public SyncActionNode
{

    public:
        geometry_msgs::PoseStamped pose;

        SelectFieldFromPoseStamped(const std::string& string, const NodeConfiguration& config)
        : SyncActionNode(string, config){}


        static PortsList providedPorts()
        {
            return { InputPort<std::string>("field"), 
                OutputPort<double>("output_val"),
                InputPort<geometry_msgs::PoseStamped>("pose_stamped")};
        }

        NodeStatus tick() override;
};

//Publish PoseStamped message

class PubPoseStampedBT : public BT::SyncActionNode
{
    public:
        PubPoseStampedBT(const std::string& name, const BT::NodeConfiguration& config);

        static BT::PortsList providedPorts()
        {
            return{BT::InputPort<std::string>("topic"),
                   BT::InputPort<geometry_msgs::PoseStamped>("pose_stamped")};
        }

        ros::NodeHandle nh_;

        BT::NodeStatus tick() override;

    private:
        std::atomic_bool _halt_requested;
        static std::map<std::string, ros::Publisher> _publishers;
};


#endif
