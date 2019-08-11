#include <vizzy_behavior_trees/rosbt_blackboard.hpp>

Blackboard::Ptr RosBlackBoard::_publishers_bb = BT::Blackboard::create();
Blackboard::Ptr RosBlackBoard::_subscribers_bb = BT::Blackboard::create();
Blackboard::Ptr RosBlackBoard::_subs_callback_queue_bb = BT::Blackboard::create();
Blackboard::Ptr RosBlackBoard::_service_clients_bb = BT::Blackboard::create();
Blackboard::Ptr RosBlackBoard::_actionclients_bb = BT::Blackboard::create();