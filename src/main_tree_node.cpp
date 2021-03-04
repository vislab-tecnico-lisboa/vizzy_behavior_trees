#include "ros/ros.h"
#include "std_msgs/String.h"
#include <behaviortree_cpp_v3/bt_factory.h>
#include <sstream>
#include <ros/package.h>

//Fancy logging for Groot
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

#include <vizzy_behavior_trees/rosbt_loadplugins.hpp>


#include "xmlrpcpp/XmlRpcException.h"
#include <xmlrpcpp/XmlRpcValue.h>


int main(int argc, char **argv)
{

  ros::init(argc, argv, "behavior_tree");
  ros::NodeHandle n;
  ros::NodeHandle nPriv("~");

  BT::BehaviorTreeFactory factory;


  std::string this_pkg_path = ros::package::getPath("vizzy_behavior_trees");

  ROSBT::registerFromROSPlugins(factory);

  std::string xmlPath;
  double rate;
  nPriv.param<std::string>("bt_xml", xmlPath, "");
  nPriv.param<double>("loop_rate", rate, 30);
  auto tree = factory.createTreeFromFile(xmlPath);
  BT::PublisherZMQ publisher_zmq(tree);

  ros::Rate loop_rate(rate);

  XmlRpc::XmlRpcValue vars_list;
  std::vector<std::string> vars;

  /*Initialize variables from file*/
  /*Thanks to this rosanswer: https://answers.ros.org/question/189299/getting-hierarchy-level-of-yaml-parameter/ */
  /*Not a straightforward library...*/

  if (nPriv.getParam("bt_vars", vars_list))
  {
    std::map<std::string, XmlRpc::XmlRpcValue>::iterator i;
    ROS_ASSERT(vars_list.getType() == XmlRpc::XmlRpcValue::TypeStruct);

    for(auto it : vars_list)
    {

      std::string var_name;
      std::string var_value;

      var_name = it.first;
      var_value = std::string(vars_list[it.first]);

      tree.rootBlackboard()->set(var_name, var_value);

      for(auto bb : tree.blackboard_stack)
      {
        bb->addSubtreeRemapping(var_name, var_name);
      }
    
    }

  }



  while (ros::ok())
  {
    tree.root_node->executeTick();
    loop_rate.sleep();
  }


  return 0;
}
