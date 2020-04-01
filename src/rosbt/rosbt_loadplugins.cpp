/*This code originally from  v-lopez, proposed on this pull request: 
https://github.com/BehaviorTree/BehaviorTree.CPP/pull/53.

However since it is not working in the original library due package renaming
I'm adding and modifying it here.

Joao Avelino, 2020*/

#include <vizzy_behavior_trees/rosbt_loadplugins.hpp>

#ifdef _WIN32
const char os_pathsep(';');   // NOLINT
#else
const char os_pathsep(':');   // NOLINT
#endif

using namespace BT;

namespace ROSBT
{

// This function is a copy from the one in class_loader_imp.hpp in ROS pluginlib
// package, licensed under BSD.
// https://github.com/ros/pluginlib
std::vector<std::string> getCatkinLibraryPaths()
{
    std::vector<std::string> lib_paths;
    const char* env = std::getenv("CMAKE_PREFIX_PATH");
    if (env)
    {
        const std::string env_catkin_prefix_paths(env);
        std::vector<BT::StringView> catkin_prefix_paths =
            splitString(env_catkin_prefix_paths, os_pathsep);
        for (BT::StringView catkin_prefix_path : catkin_prefix_paths)
        {
            BT::StringView lol;
            filesystem::path path(catkin_prefix_path.to_string());
            filesystem::path lib("lib");
            lib_paths.push_back((path / lib).str());
        }
    }
    return lib_paths;
}

void registerFromROSPlugins(BT::BehaviorTreeFactory &factory)
{
    std::vector<std::string> plugins;
    ros::package::getPlugins("vizzy_behavior_trees", "bt_lib_plugin", plugins, true);
    std::vector<std::string> catkin_lib_paths = getCatkinLibraryPaths();

    for (const auto& plugin : plugins)
    {
        auto filename = filesystem::path(plugin + BT::SharedLibrary::suffix());
        for (const auto& lib_path : catkin_lib_paths)
        {
            const auto full_path = filesystem::path(lib_path) / filename;
            if (full_path.exists())
            {
                std::cout << "Registering ROS plugins from " << full_path.str() << std::endl;
                factory.registerFromPlugin(full_path.str());
                break;
            }
        }
    }
}

} // namespace ROSBT