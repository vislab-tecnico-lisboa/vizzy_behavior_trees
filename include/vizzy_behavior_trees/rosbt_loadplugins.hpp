#ifndef ROSBT_LOAD_PLUGINS_H_
#define ROSBT_LOAD_PLUGINS_H_

/*This code originally from  v-lopez, proposed on this pull request: 
https://github.com/BehaviorTree/BehaviorTree.CPP/pull/53.

However since it is not working in the original library due package renaming
I'm adding and modifying it here.

Joao Avelino, 2020*/

#include "filesystem/path.h"
#include <ros/package.h>
#include <behaviortree_cpp_v3/utils/shared_library.h>
#include <behaviortree_cpp_v3/utils/shared_library.h>
#include <behaviortree_cpp_v3/xml_parsing.h>

namespace ROSBT
{
    

void registerFromROSPlugins(BT::BehaviorTreeFactory &factory);

} // namespace ROSBT

#endif