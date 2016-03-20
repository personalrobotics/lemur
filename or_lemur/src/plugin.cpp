/*! \file plugin.cpp
 * \author Chris Dellin <cdellin@gmail.com>
 * \copyright 2015 Carnegie Mellon University
 * \copyright License: BSD
 */

#include <algorithm>

#include <boost/chrono.hpp>
#include <boost/graph/adjacency_list.hpp>

#include <openrave/openrave.h>
#include <openrave/plugin.h>

#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/Planner.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/datastructures/NearestNeighbors.h>

#include <pr_bgl/compose_property_map.hpp>
#include <pr_bgl/edge_indexed_graph.h>
#include <pr_bgl/overlay_manager.h>
#include <pr_bgl/string_map.h>
#include <pr_bgl/heap_indexed.h>

#include <ompl_lemur/BisectPerm.h>
#include <ompl_lemur/NearestNeighborsLinearBGL.h>
#include <ompl_lemur/Roadmap.h>
#include <ompl_lemur/TagCache.h>
#include <ompl_lemur/UtilityChecker.h>
#include <ompl_lemur/LEMUR.h>
#include <ompl_lemur/Family.h>
#include <ompl_lemur/FamilyUtilityChecker.h>
#include <ompl_lemur/FamilyTagCache.h>

#include <or_lemur/or_ompl_conversions.h>
#include <or_lemur/module_family.h>
#include <or_lemur/params_lemur.h>
#include <or_lemur/params_family.h>
#include <or_lemur/planner_lemur.h>
#include <or_lemur/planner_family.h>
#include <or_lemur/planner_cctimer.h>

namespace {

template <class C>
OpenRAVE::InterfaceBase * make(OpenRAVE::EnvironmentBasePtr env)
{
   return new C(env);
}

struct Type
{
   OpenRAVE::InterfaceType ortype;
   std::string name;
   OpenRAVE::InterfaceBase *(*make)(OpenRAVE::EnvironmentBasePtr env);
};

// list of interface types we can construct
static Type types[] =
{
   {OpenRAVE::PT_Module,  "Family",        &make<or_lemur::FamilyModule>},
   {OpenRAVE::PT_Planner, "LEMUR",         &make<or_lemur::LEMUR>},
   {OpenRAVE::PT_Planner, "FamilyPlanner", &make<or_lemur::FamilyPlanner>},
   {OpenRAVE::PT_Planner, "CCTimer",       &make<or_lemur::CCTimer>},
};
static int num_types = sizeof(types)/sizeof(types[0]);

} // anonymous namespace

void GetPluginAttributesValidated(OpenRAVE::PLUGININFO& info)
{
   for (Type * type=types; type-types<num_types; type++)
      info.interfacenames[type->ortype].push_back(type->name);
}

OpenRAVE::InterfaceBasePtr CreateInterfaceValidated(
   OpenRAVE::InterfaceType ortype, const std::string & name,
   std::istream& sinput, OpenRAVE::EnvironmentBasePtr env)
{
   for (Type * type=types; type-types<num_types; type++)
   {
      std::string low = type->name;
      std::transform(low.begin(), low.end(), low.begin(), ::tolower);
      if (ortype == type->ortype && name == low)
         return OpenRAVE::InterfaceBasePtr(type->make(env));
   }
   return OpenRAVE::InterfaceBasePtr();
}

OPENRAVE_PLUGIN_API void DestroyPlugin()
{
}
