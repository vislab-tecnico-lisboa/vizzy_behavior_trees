#include "ros/ros.h"
#include "std_msgs/String.h"
#include <behaviortree_cpp/bt_factory.h>
#include <sstream>
#include <ros/package.h>


//Includes for nodes
#include <vizzy_behavior_trees/actions/speech_actions.hpp>
#include <vizzy_behavior_trees/actions/move_base_actions.hpp>
#include <vizzy_behavior_trees/actions/general.hpp>
#include <vizzy_behavior_trees/actions/charging_actions.hpp>
#include <vizzy_behavior_trees/actions/gaze_actions.hpp>
#include <vizzy_behavior_trees/actions/arm_cartesian_actions.hpp>
#include <vizzy_behavior_trees/actions/arm_routines.hpp>
#include <vizzy_behavior_trees/actions/ros_msgs/get_geometry_msgs.hpp>
#include <vizzy_behavior_trees/actions/ros_msgs/get_std_msgs.hpp>
#include <vizzy_behavior_trees/conditions/general.hpp>

//Fancy logging for Groot
#include "behaviortree_cpp/loggers/bt_zmq_publisher.h"


int main(int argc, char **argv)
{

  ros::init(argc, argv, "behavior_tree");
  ros::NodeHandle n;
  ros::NodeHandle nPriv("~");

  BT::BehaviorTreeFactory factory;

  factory.registerNodeType<SpeechActionBT>("Speak");
  factory.registerNodeType<MoveBaseActionBT>("MoveBase");
  factory.registerNodeType<WaitForXSeconds>("WaitForXSeconds");
  factory.registerNodeType<ChargeActionBT>("Charge");
  factory.registerNodeType<CartesianActionBT>("ArmCartesian");
  factory.registerNodeType<ArmRoutineBT>("ArmRoutine");
  factory.registerNodeType<CompareInt>("CompareInt");
  factory.registerNodeType<CompareDouble>("CompareDouble");
  factory.registerNodeType<CheckBool>("CheckBool");
  factory.registerNodeType<CheckChargingBT>("CheckCharging");
  factory.registerNodeType<CheckBatteryBT>("CheckBattery");
  factory.registerNodeType<GetInt16BT>("GetInt16");
  factory.registerNodeType<GetPoseArrayBT>("GetPoseArray");
  factory.registerNodeType<SelectPoseStamped>("SelectPoseStamped");
  factory.registerNodeType<GazeActionBT>("GazeAtTarget");

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


  return 0;
}
