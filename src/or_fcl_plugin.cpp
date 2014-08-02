#include <boost/make_shared.hpp>
#include <openrave/plugin.h>
#include "FCLCollisionChecker.h"

using OpenRAVE::EnvironmentBasePtr;
using OpenRAVE::InterfaceBasePtr;
using OpenRAVE::InterfaceType;
using OpenRAVE::PT_CollisionChecker;
using OpenRAVE::PLUGININFO;
using or_fcl::FCLCollisionChecker;

InterfaceBasePtr CreateInterfaceValidated(
        InterfaceType type, std::string const &interface_name,
        std::istream &input, EnvironmentBasePtr env)
{
    if (type == PT_CollisionChecker && interface_name == "fcl") {
        return boost::make_shared<FCLCollisionChecker>(env);
    } else {
        return InterfaceBasePtr();
    }
}

void GetPluginAttributesValidated(PLUGININFO &info)
{
    info.interfacenames[PT_CollisionChecker].push_back("fcl");
}

RAVE_PLUGIN_API void DestroyPlugin()
{
}
