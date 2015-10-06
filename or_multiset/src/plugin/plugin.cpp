/* File: plugin.cpp
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

#include <boost/graph/adjacency_list.hpp>

#include <openrave/openrave.h>
#include <openrave/plugin.h>

#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/Planner.h>
#include <ompl/base/ProblemDefinition.h>

#include <pr_bgl/edge_indexed_graph.h>
#include <pr_bgl/overlay_manager.h>
#include <pr_bgl/heap_indexed.h>

#include <ompl_multiset/MultiSetRoadmap.h>
#include <ompl_multiset/Cache.h>
#include <ompl_multiset/MultiSetPRM.h>

#include <ompl_multiset/BisectPerm.h>
#include <ompl_multiset/RoadmapGen.h>
#include <ompl_multiset/EffortModel.h>
#include <ompl_multiset/E8Roadmap.h>
#include <ompl_multiset/SimpleEffortModel.h>

#include <or_multiset/inter_link_checks.h>

#include "module_subset_manager.h"
#include "planner_multiset_prm.h"
#include "planner_e8roadmap.h"

void GetPluginAttributesValidated(OpenRAVE::PLUGININFO& info)
{
   info.interfacenames[OpenRAVE::PT_Planner].push_back("MultiSetPRM");
   info.interfacenames[OpenRAVE::PT_Planner].push_back("E8Roadmap");
   info.interfacenames[OpenRAVE::PT_Module].push_back("SubsetManager");
}

OpenRAVE::InterfaceBasePtr CreateInterfaceValidated(
   OpenRAVE::InterfaceType type,
   const std::string & interfacename,
   std::istream& sinput,
   OpenRAVE::EnvironmentBasePtr penv)
{
   if((type == OpenRAVE::PT_Planner) && (interfacename == "multisetprm"))
      return OpenRAVE::InterfaceBasePtr(new or_multiset::MultiSetPRM(penv));
   if((type == OpenRAVE::PT_Planner) && (interfacename == "e8roadmap"))
      return OpenRAVE::InterfaceBasePtr(new or_multiset::E8Roadmap(penv));
   if((type == OpenRAVE::PT_Module) && (interfacename == "subsetmanager"))
      return OpenRAVE::InterfaceBasePtr(new or_multiset::ModuleSubsetManager(penv));
   return OpenRAVE::InterfaceBasePtr();
}

OPENRAVE_PLUGIN_API void DestroyPlugin()
{
}
