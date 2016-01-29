/* File: family_effort_model.cpp
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

#include <set>
#include <boost/graph/adjacency_list.hpp>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <pr_bgl/heap_indexed.h>
#include <ompl_lemur/logic_engine.h>
#include <ompl_lemur/EffortModel.h>
#include <ompl_lemur/Family.h>
#include <ompl_lemur/FamilyEffortModel.h>

#include <gtest/gtest.h>

// (  ( s1[c=1] )  s2[c=2]  )
//                 ^ target
TEST(FamilyEffortModelCase, InclusionTest)
{
   // note that we don't set up space or si's for now
   
   ompl::base::StateSpacePtr space(
      new ompl::base::RealVectorStateSpace(1));
   ompl_lemur::Family family;
   
   ompl::base::SpaceInformationPtr si1(
      new ompl::base::SpaceInformation(space));
   family.subsets.insert(std::make_pair(
      "si1", ompl_lemur::Family::Subset(si1, 1.0)));
   
   ompl::base::SpaceInformationPtr si2(
      new ompl::base::SpaceInformation(space));
   family.subsets.insert(std::make_pair(
      "si2", ompl_lemur::Family::Subset(si2, 2.0)));
   
   family.inclusions.insert(ompl_lemur::Family::Inclusion("si1","si2"));
   
   ompl_lemur::FamilyEffortModel model(family);
   model.set_target(si2);
   
   ASSERT_TRUE(model.p_hat(0,0) == 1.0);
}

int main(int argc, char **argv)
{
   testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
