#include <vizzy_behavior_trees/actions/ros_msgs/get_geometry_msgs.hpp>
#include <cmath>

bool comparePoseStamped(geometry_msgs::Pose i, geometry_msgs::Pose j)
{ 

    double n1 = std::sqrt(std::pow(i.position.x, 2) + std::pow(i.position.y, 2) + std::pow(i.position.z, 2));
    double n2 = std::sqrt(std::pow(j.position.x, 2) + std::pow(j.position.y, 2) + std::pow(j.position.z, 2));

    return (n1<n2); 
}


//PoseArray
void GetPoseArrayBT::callback(const geometry_msgs::PoseArray::ConstPtr &msg)
{
    //Organize array based on its norm, from smaller to bigger
    this->poseList.header = msg->header;
    this->poseList.poses = msg->poses;

    std::sort (this->poseList.poses.begin(), this->poseList.poses.end(), comparePoseStamped);

}


GetPoseArrayBT::GetPoseArrayBT(const std::string& name, const NodeConfiguration& config)
    : SyncActionNode(name, config)
    {
        ros::NodeHandle nh_;
        BT::Optional<std::string> topic = TreeNode::getInput<std::string>("topic");

        if(!topic){
            throw BT::RuntimeError("missing required inputs [topic]: ",
                                    topic.error()); 
        }

        ros::SubscribeOptions ops = ros::SubscribeOptions::create<geometry_msgs::PoseArray>
            (topic.value(), 1, boost::bind(&GetPoseArrayBT::callback, this, _1), ros::VoidPtr(), &queue_);

        sub_ = nh_.subscribe(ops);
    }

BT::NodeStatus GetPoseArrayBT::tick()
{

    if(sub_.getNumPublishers() < 1)
    {
        return BT::NodeStatus::FAILURE;
    }

    setStatus(BT::NodeStatus::RUNNING);
    queue_.callOne();


    auto result = setOutput("pose_array", this->poseList);

    if(!result)
    {
        return BT::NodeStatus::FAILURE;
    }

   return BT::NodeStatus::SUCCESS;
}

//PoseStamped
NodeStatus SelectPose::tick()
{

    BT::Optional<geometry_msgs::PoseArray> poses = getInput<geometry_msgs::PoseArray>("pose_array");
    BT::Optional<std::string> pos_in_list = getInput<std::string>("position_in_list");

    if(!poses)
    {

        throw BT::RuntimeError("missing required inputs [pose_array]: ",
                                poses.error()); 

    }if(!pos_in_list)
    {

        throw BT::RuntimeError("missing required inputs [position_in_list]: ",
                                pos_in_list.error()); 
    }

    setStatus(BT::NodeStatus::RUNNING);

    int position = std::atoi(pos_in_list.value().c_str());


    geometry_msgs::PoseArray list;

    list = poses.value();

    if(list.poses.size() <= position)
    {
        return BT::NodeStatus::FAILURE;
    }


    pose.header.frame_id = list.header.frame_id;
    pose.header.stamp = list.header.stamp;
    pose.pose = list.poses[position];

    auto result = setOutput("pose_stamped", pose);

    if(!result)
    {
        return BT::NodeStatus::FAILURE;
    }

   

    return BT::NodeStatus::SUCCESS;

}

//PoseStamped
NodeStatus SelectFieldFromPoseStamped::tick()
{

    BT::Optional<std::string> field = getInput<std::string>("field");
    BT::Optional<geometry_msgs::PoseStamped> pose = getInput<geometry_msgs::PoseStamped>("pose_stamped");

    if(!field)
    {

        throw BT::RuntimeError("missing required inputs [field]: ",
                                field.error()); 

    }if(!pose)
    {

        throw BT::RuntimeError("missing required inputs [pose]: ",
                                pose.error()); 
    }

    geometry_msgs::PoseStamped pose_val = pose.value();

    setStatus(BT::NodeStatus::RUNNING);
    
    double value;

    if(field.value() == "x")
    {
        value = pose_val.pose.position.x;
    }else if(field.value() == "y")
    {
        value = pose_val.pose.position.y;
    }else if(field.value() == "z")
    {
        value = pose_val.pose.position.z;
    }else
        return BT::NodeStatus::FAILURE;


    auto result = setOutput("output_val", value);

    if(!result)
    {
        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::SUCCESS;

}
