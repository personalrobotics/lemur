/* File: e8fromfile.cpp
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

#include <fstream>

#include <boost/chrono.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/property_map/dynamic_property_map.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphml.hpp>

#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/Planner.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/datastructures/NearestNeighbors.h>

#include <pr_bgl/compose_property_map.hpp>
#include <pr_bgl/edge_indexed_graph.h>
#include <pr_bgl/heap_indexed.h>
#include <pr_bgl/string_map.h>
#include <pr_bgl/overlay_manager.h>
#include <ompl_lemur/util.h>
#include <ompl_lemur/rvstate_map_string_adaptor.h>
#include <ompl_lemur/BisectPerm.h>
#include <ompl_lemur/NearestNeighborsLinearBGL.h>
#include <ompl_lemur/Roadmap.h>
#include <ompl_lemur/EffortModel.h>
#include <ompl_lemur/LEMUR.h>
#include <ompl_lemur/Family.h>
#include <ompl_lemur/FamilyEffortModel.h>
#include <ompl_lemur/RoadmapFromFile.h>

#include <gtest/gtest.h>

#define XSTR(s) STR(s)
#define STR(s) # s

bool isvalid(const ompl::base::State * state)
{
   double * values = state->as<
      ompl::base::RealVectorStateSpace::StateType>()->values;
   if (values[0] < 0.5 && values[1] < 0.5)
      return false;
   else
      return true;
}

ompl::base::ScopedState<ompl::base::RealVectorStateSpace>
make_state(const ompl::base::StateSpacePtr space, double x, double y)
{
   ompl::base::ScopedState<ompl::base::RealVectorStateSpace>
      state(space);
   double * values = state->as<
      ompl::base::RealVectorStateSpace::StateType>()->values;
   values[0] = x;
   values[1] = y;
   return state;
}

ompl::base::ScopedState<ompl::base::RealVectorStateSpace>
get_path_state(boost::shared_ptr<ompl::geometric::PathGeometric> path, size_t idx)
{
   ompl::base::StateSpacePtr space = path->getSpaceInformation()->getStateSpace();
   ompl::base::ScopedState<ompl::base::RealVectorStateSpace>
      traj_state(space, path->getState(idx));
   return traj_state;
}

TEST(E8FromFileTestCase, E8FromFileTest)
{
   // state space
   boost::shared_ptr<ompl::base::RealVectorStateSpace> space(
      new ompl::base::RealVectorStateSpace(2));
   space->setBounds(0.0, 1.0);
   space->setLongestValidSegmentFraction(
      0.001 / space->getMaximumExtent());
   space->setup();
   
   // space info
   ompl::base::SpaceInformationPtr si(
      new ompl::base::SpaceInformation(space));
   si->setStateValidityChecker(isvalid);
   si->setup();
   
   // problem definition
   ompl::base::ProblemDefinitionPtr pdef(
      new ompl::base::ProblemDefinition(si));
   pdef->addStartState(make_state(space, 0.25, 0.75));
   pdef->setGoalState(make_state(space, 0.75, 0.25));
   
   // family
   ompl_lemur::Family family;
   family.subsets.insert(std::make_pair("space_si",
      ompl_lemur::Family::Subset(si, space->getLongestValidSegmentLength())));
   ompl_lemur::FamilyEffortModel fem(family);
   fem.set_target(si);
   
   // cache
   ompl_lemur::DummyTagCache<ompl_lemur::LEMUR::VIdxTagMap,ompl_lemur::LEMUR::EIdxTagsMap> tag_cache;
   
   // planner
   ompl::base::PlannerPtr planner(new ompl_lemur::LEMUR(space, fem, tag_cache));
   planner->as<ompl_lemur::LEMUR>()->setCoeffDistance(1.);
   planner->as<ompl_lemur::LEMUR>()->setCoeffCheckcost(0.);
   planner->as<ompl_lemur::LEMUR>()->setCoeffBatch(0.);
   
   // roadmap
   planner->as<ompl_lemur::LEMUR>()->registerRoadmapType<ompl_lemur::RoadmapFromFile>("FromFile");
   planner->as<ompl_lemur::LEMUR>()->setRoadmapType("FromFile");
   planner->params().setParam("roadmap.filename", XSTR(DATADIR) "/halton2d.xml");
   planner->params().setParam("roadmap.root_radius", "0.3");
   
   // solve
   planner->setProblemDefinition(pdef);
   ompl::base::PlannerStatus status = planner->solve(
      ompl::base::plannerNonTerminatingCondition());
   ASSERT_EQ(status, ompl::base::PlannerStatus::EXACT_SOLUTION);
   
   // check resulting path
   boost::shared_ptr<ompl::geometric::PathGeometric> path = 
      boost::dynamic_pointer_cast<ompl::geometric::PathGeometric>(
      pdef->getSolutionPath());
   ASSERT_TRUE(path);
   ASSERT_EQ(path->getStateCount(), 4);
   ASSERT_EQ(get_path_state(path,0), make_state(space, 0.25, 0.75));
   ASSERT_EQ(get_path_state(path,1), make_state(space, 0.40625, 14./27.));
   ASSERT_EQ(get_path_state(path,2), make_state(space, 0.68750, 13./27.));
   ASSERT_EQ(get_path_state(path,3), make_state(space, 0.75, 0.25));
}

int main(int argc, char **argv)
{
   testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
