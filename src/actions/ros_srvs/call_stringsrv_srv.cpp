#include <vizzy_behavior_trees/actions/ros_srvs/stringsrv_srv.hpp>

StringSrvBT::StringSrvBT(const std::string& name, const NodeConfiguration& config)
    : SyncActionNode(name, config)
    {
        ros::NodeHandle nh_;
        BT::Optional<std::string> service_name = TreeNode::getInput<std::string>("service_name");

        if(!service_name){
            throw BT::RuntimeError("missing required inputs [service_name]: ",
                                    service_name.error());
        }


        srv_ = nh_.serviceClient<vizzy_behavior_trees::StringSrv>(service_name.value());
    }

BT::NodeStatus StringSrvBT::tick()
{

    BT::Optional<std::string> cmd = getInput<std::string>("cmd");
    BT::Optional<std::string> arg1 = getInput<std::string>("arg1");
    BT::Optional<std::string> arg2 = getInput<std::string>("arg2");

    vizzy_behavior_trees::StringSrv ss;


    if(cmd)
    {
        ss.request.command = cmd.value();
    }

    if(arg1)
    {
        ss.request.arg1 = arg1.value();
    }

    if(arg2)
    {
        ss.request.arg2 = arg2.value();
    }

    if(srv_.call(ss))
    {
        if(ss.response.success)
        {
            setOutput("message", ss.response.message);
            return BT::NodeStatus::SUCCESS;
        }else{
            return BT::NodeStatus::FAILURE;
        }
    }else{
        return BT::NodeStatus::FAILURE;
    }
}
