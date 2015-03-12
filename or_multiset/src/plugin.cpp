#include <openrave/openrave.h>
//#include <openrave/planner.h>
#include <openrave/plugin.h>
#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/Planner.h>
#include <ompl/base/ProblemDefinition.h>

#include <ompl_multiset/MultiSetPRM.h>

#include "params_checkmask.h"
#include "planner_checkmask.h"

void GetPluginAttributesValidated(OpenRAVE::PLUGININFO& info)
{
   info.interfacenames[OpenRAVE::PT_Planner].push_back("MultiSetPRM");
}

OpenRAVE::InterfaceBasePtr CreateInterfaceValidated(
   OpenRAVE::InterfaceType type,
   const std::string & interfacename,
   std::istream& sinput,
   OpenRAVE::EnvironmentBasePtr penv)
{
   if((type == OpenRAVE::PT_Planner) && (interfacename == "multisetprm"))
      return OpenRAVE::InterfaceBasePtr(new or_multiset::MultiSetPRM(penv));
   return OpenRAVE::InterfaceBasePtr();
}

OPENRAVE_PLUGIN_API void DestroyPlugin()
{
}
