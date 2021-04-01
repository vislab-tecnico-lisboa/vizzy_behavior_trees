#include <vizzy_behavior_trees/actions/ros_srvs/empty_srv.hpp>

EmptySrvBT::EmptySrvBT(const std::string& name, const NodeConfiguration& config)
    : SyncActionNode(name, config)
    {
        ros::NodeHandle nh_;
        BT::Optional<std::string> service_name = TreeNode::getInput<std::string>("service_name");

        if(!service_name){
            throw BT::RuntimeError("missing required inputs [service_name]: ",
                                    service_name.error());
        }


        srv_ = nh_.serviceClient<std_srvs::Empty>(service_name.value());
    }

BT::NodeStatus EmptySrvBT::tick()
{
    std_srvs::Empty empty;
    if(srv_.call(empty))
    {
        return BT::NodeStatus::SUCCESS;
    }else{
        return BT::NodeStatus::FAILURE;
    }
}
