#include "ros/ros.h"
#include "std_msgs/String.h"
#include <behaviortree_cpp/bt_factory.h>
#include <sstream>
#include <ros/package.h>

//Includes for all the actions that Vizzy can do
#include <vizzy_msgs/GazeAction.h>
#include <vizzy_msgs/CartesianAction.h>

//Includes for nodes
#include <vizzy_behavior_trees/actions/speech_actions.hpp>
#include <vizzy_behavior_trees/actions/move_base_actions.hpp>
#include <vizzy_behavior_trees/actions/general.hpp>


//Fancy logging for Groot
#include "behaviortree_cpp/loggers/bt_zmq_publisher.h"


int main(int argc, char **argv)
{

  ros::init(argc, argv, "behavior_tree");
  ros::NodeHandle n;
  ros::NodeHandle nPriv("~");


  /*Action clients*/
  actionlib::SimpleActionClient<woz_dialog_msgs::SpeechAction> *speech_client;
  speech_client = new actionlib::SimpleActionClient<woz_dialog_msgs::SpeechAction>("nuance_speech_tts");
  ROS_INFO("Waiting for speech client server");
  speech_client->waitForServer();

  /****************/

  BT::BehaviorTreeFactory factory;

  BT::NodeBuilder builder_speech = [speech_client](const std::string& name, const BT::NodeConfiguration& config)
    {
        return std::make_unique<SpeechActionBT>( name, config, speech_client);
    };

  factory.registerBuilder<SpeechActionBT>("Speak", builder_speech);
  factory.registerNodeType<MoveBaseActionBT>("MoveBase");
  factory.registerNodeType<WaitForXSeconds>("WaitForXSeconds");

  std::string xmlPath;
  nPriv.param<std::string>("bt_xml", xmlPath, "");
  auto tree = factory.createTreeFromFile(xmlPath);
  BT::PublisherZMQ publisher_zmq(tree);


  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    tree.root_node->executeTick();
    loop_rate.sleep();
  }


  delete speech_client;

  return 0;
}