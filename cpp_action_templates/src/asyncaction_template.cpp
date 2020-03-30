/*
  -----------------------------------------------------
  A template to create the original library's (by facontidavide) AsyncActionNode actions
  Don't forget to replace ASYNCACTION_TEMPLATE with the name of your
  action!

  This action creates a new thread! Avoid using it! Use CoroActionNode and SyncActionNode
  as much as possible instead of this one
*/


#include <YOURPACKAGE/INCLUDE_DIR/asyncaction_template.hpp>

BT::NodeStatus ASYNCACTION_TEMPLATE_BT::tick()
{

    SOME_TYPE some_var;
    if ( !getInput<SOME_TYPE>("some_input", some_var))
    {
        throw BT::RuntimeError("missing required input [some_input]");
    }

    /*Some code...*/

    _halt_requested.store(false);

    // It is up to you to check periodically _halt_requested and interrupt
    // this tick() if it is true.

    while (SOME_CONDITION)
    {
        /*Code*/
    }

    return _halt_requested ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
}

void ASYNCACTION_TEMPLATE_BT::halt()
{
    _halt_requested.store(true);
}