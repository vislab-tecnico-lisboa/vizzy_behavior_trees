#include "behaviortree_cpp_v3/bt_factory.h"
#include <vizzy_behavior_trees/actions/speech_actions.hpp>
#include <vizzy_behavior_trees/actions/move_base_actions.hpp>
#include <vizzy_behavior_trees/actions/general.hpp>
#include <vizzy_behavior_trees/actions/charging_actions.hpp>
#include <vizzy_behavior_trees/actions/gaze_actions.hpp>
#include <vizzy_behavior_trees/actions/arm_cartesian_actions.hpp>
#include <vizzy_behavior_trees/actions/arm_routines.hpp>
#include <vizzy_behavior_trees/actions/ros_msgs/pubsub_geometry_msgs.hpp>
#include <vizzy_behavior_trees/actions/ros_msgs/get_std_msgs.hpp>
#include <vizzy_behavior_trees/conditions/general.hpp>
#include <vizzy_behavior_trees/actions/torso_actions.hpp>
#include <vizzy_behavior_trees/actions/ros_srvs/empty_srv.hpp>
#include <vizzy_behavior_trees/actions/ros_srvs/trigger_srv.hpp>


BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<ArmRoutineBT>("ArmRoutines");
  factory.registerNodeType<CartesianActionBT>("ArmCartesian");
  factory.registerNodeType<GazeActionBT>("GazeAtTarget");
  factory.registerNodeType<WaitForXSeconds>("WaitForXSeconds");
  factory.registerNodeType<TimerAction>("TimerAction");
  factory.registerNodeType<DebugAction>("DebugAction");
  factory.registerNodeType<GeneralActionBT>("GeneralActionlib");
  factory.registerNodeType<ChargeActionBT>("Charge");
  factory.registerNodeType<CheckChargingBT>("CheckCharging");
  factory.registerNodeType<CheckBatteryBT>("CheckBattery");
  factory.registerNodeType<MoveBaseActionBT>("MoveBase");
  factory.registerNodeType<GetPoseArrayBT>("GetPoseArray");
  factory.registerNodeType<SelectPose>("SelectPose");
  factory.registerNodeType<SelectFieldFromPoseStamped>("SelectFieldFromPoseStamped");
  factory.registerNodeType<GetInt16BT>("GetInt16");
  factory.registerNodeType<GetFloat64BT>("GetFloat64");
  factory.registerNodeType<SpeechActionBT>("Speak");
  factory.registerNodeType<TorsoRoutineBT>("MoveTorso");
  factory.registerNodeType<CompareInt>("CompareInt");
  factory.registerNodeType<CompareDouble>("CompareDouble");
  factory.registerNodeType<CheckBool>("CheckBool");
  factory.registerNodeType<EmptySrvBT>("CallEmptySrv");
  factory.registerNodeType<TriggerSrvBT>("CallTriggerSrv");
  factory.registerNodeType<PubPoseStampedBT>("PubPoseStamped");
}