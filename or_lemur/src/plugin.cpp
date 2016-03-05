/* File: plugin.cpp
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

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

#include <ompl_lemur/MultiSetRoadmap.h>
#include <ompl_lemur/Cache.h>
#include <ompl_lemur/MultiSetPRM.h>

#include <ompl_lemur/BisectPerm.h>
#include <ompl_lemur/NearestNeighborsLinearBGL.h>
#include <ompl_lemur/Roadmap.h>
#include <ompl_lemur/TagCache.h>
#include <ompl_lemur/UtilityChecker.h>
#include <ompl_lemur/LEMUR.h>
#include <ompl_lemur/Family.h>
#include <ompl_lemur/FamilyUtilityChecker.h>

#include <or_lemur/inter_link_checks.h>

#include <or_lemur/module_family.h>
#include <or_lemur/module_subset_manager.h>
#include <or_lemur/or_checker.h>
#include <or_lemur/planner_multiset_prm.h>
#include <or_lemur/params_lemur.h>
#include <or_lemur/params_family.h>
#include <or_lemur/planner_lemur.h>
#include <or_lemur/planner_family.h>
#include <or_lemur/planner_cctimer.h>

void GetPluginAttributesValidated(OpenRAVE::PLUGININFO& info)
{
   info.interfacenames[OpenRAVE::PT_Planner].push_back("MultiSetPRM");
   info.interfacenames[OpenRAVE::PT_Planner].push_back("LEMUR");
   info.interfacenames[OpenRAVE::PT_Planner].push_back("FamilyPlanner");
   info.interfacenames[OpenRAVE::PT_Planner].push_back("CCTimer");
   info.interfacenames[OpenRAVE::PT_Module].push_back("Family");
   info.interfacenames[OpenRAVE::PT_Module].push_back("SubsetManager");
}

OpenRAVE::InterfaceBasePtr CreateInterfaceValidated(
   OpenRAVE::InterfaceType type,
   const std::string & interfacename,
   std::istream& sinput,
   OpenRAVE::EnvironmentBasePtr penv)
{
   if((type == OpenRAVE::PT_Planner) && (interfacename == "multisetprm"))
      return OpenRAVE::InterfaceBasePtr(new or_lemur::MultiSetPRM(penv));
   if((type == OpenRAVE::PT_Planner) && (interfacename == "lemur"))
      return OpenRAVE::InterfaceBasePtr(new or_lemur::LEMUR(penv));
   if((type == OpenRAVE::PT_Planner) && (interfacename == "familyplanner"))
      return OpenRAVE::InterfaceBasePtr(new or_lemur::FamilyPlanner(penv));
   if((type == OpenRAVE::PT_Planner) && (interfacename == "cctimer"))
      return OpenRAVE::InterfaceBasePtr(new or_lemur::CCTimer(penv));
   if((type == OpenRAVE::PT_Module) && (interfacename == "family"))
      return OpenRAVE::InterfaceBasePtr(new or_lemur::FamilyModule(penv));
   if((type == OpenRAVE::PT_Module) && (interfacename == "subsetmanager"))
      return OpenRAVE::InterfaceBasePtr(new or_lemur::ModuleSubsetManager(penv));
   return OpenRAVE::InterfaceBasePtr();
}

OPENRAVE_PLUGIN_API void DestroyPlugin()
{
}
