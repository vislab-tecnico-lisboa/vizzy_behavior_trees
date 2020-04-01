/*
  -----------------------------------------------------
  A template to create the original library's (by facontidavide) SyncActionNode actions
  Don't forget to replace SYNCACTION_TEMPLATE with the name of your
  action!
*/


#include <YOURPACKAGE/INCLUDE_DIR/syncaction_template.hpp>
#include "behaviortree_cpp_v3/bt_factory.h"

BT::NodeStatus SYNCACTION_TEMPLATE_BT::tick()
{

    SOME_TYPE some_var;
    if ( !getInput<SOME_TYPE>("some_input", some_var))
    {
        throw BT::RuntimeError("missing required input [some_input]");
    }

    /*Some code...*/

    if(SUCCESS_CONDITION)
    {
        return BT::NodeStatus::SUCCESS;

    }else{
        return BT::NodeStatus::FAILURE;
    }
}