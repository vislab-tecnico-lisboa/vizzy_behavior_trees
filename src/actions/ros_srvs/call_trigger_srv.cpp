#include <vizzy_behavior_trees/actions/ros_srvs/trigger_srv.hpp>

TriggerSrvBT::TriggerSrvBT(const std::string& name, const NodeConfiguration& config)
    : SyncActionNode(name, config)
    {
        ros::NodeHandle nh_;
        BT::Optional<std::string> service_name = TreeNode::getInput<std::string>("service_name");

        if(!service_name){
            throw BT::RuntimeError("missing required inputs [service_name]: ",
                                    service_name.error());
        }


        srv_ = nh_.serviceClient<std_srvs::Trigger>(service_name.value());
    }

BT::NodeStatus TriggerSrvBT::tick()
{
    std_srvs::Trigger trigger;
    if(srv_.call(trigger))
    {
        if(trigger.response.success)
        {
            setOutput("message", trigger.response.message);
            return BT::NodeStatus::SUCCESS;
        }else{
            return BT::NodeStatus::FAILURE;
        }
    }else{
        return BT::NodeStatus::FAILURE;
    }
}
