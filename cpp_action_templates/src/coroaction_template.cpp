/*
  -----------------------------------------------------
  A template to create the original library's (by facontidavide) CoroActionNode actions
  Don't forget to replace COROACTION_TEMPLATE with the name of your
  action!
*/

#include <YOURPACKAGE/INCLUDE_DIR/coroaction_template.hpp>
#include "behaviortree_cpp_v3/bt_factory.h"


BT::NodeStatus COROACTION_TEMPLATE_BT::tick()
{


    SOME_TYPE some_var = getInput<SOME_TYPE>("some_input");

    if(!some_var)
    {
        throw BT::RuntimeError("missing required inputs [some_input]: ",
                                some_var.error() );
    }



    while(!SOME_DONE_CONDITION)
    {
        /*Do one quick iteration of a possibly long algorithm here*/
        /*  CODE CODE CODE */

        /*The next command is fundamental. Returns RUNNING to the behavior tree and
        when this node receives another tick it resumes from here...*/
        setStatusRunningAndYield();
    }


    return BT::NodeStatus::SUCCESS;

}

void COROACTION_TEMPLATE_BT::cleanup(bool halted)
{
    if(halted)
    {
        /*If you need to clean something up if the action gets preempted
        this is the place!*/
    }

}

void COROACTION_TEMPLATE_BT::halt()
{
    std::cout << name() <<": Halted." << std::endl;
    cleanup(true);
    CoroActionNode::halt();
}