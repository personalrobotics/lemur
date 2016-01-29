/* File: logic.cpp
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

#include <cstdio>
#include <cstdlib>
#include <algorithm>
#include <vector>
#include <pr_bgl/heap_indexed.h>
#include <ompl_lemur/logic_engine.h>

#include <gtest/gtest.h>

TEST(LogicTestCase, IdentityTest)
{
   ompl_lemur::LogicEngine logic(1);
   logic.initialize();
   
   ompl_lemur::LogicEngine::Assignment a = logic.empty_assignment();
   ompl_lemur::LogicEngine::VarVal target;
   std::vector<double> costs;
   costs.push_back(1.0);
   
   std::vector<ompl_lemur::LogicEngine::AssignmentDelta> deltas;
   
   target.var = 0;
   target.value = true;
   deltas.clear();
   logic.cheapest(a, target, costs, deltas);
   ASSERT_TRUE(deltas.size() == 1);
   ASSERT_TRUE(deltas[0].new_varval.var == 0);
   ASSERT_TRUE(deltas[0].new_varval.value == true);
   ASSERT_TRUE(deltas[0].result.var_mask.size() == 1);
   ASSERT_TRUE(deltas[0].result.var_mask[0] == true);
   ASSERT_TRUE(deltas[0].result.values.size() == 1);
   ASSERT_TRUE(deltas[0].result.values[0] == true);
   
   target.var = 0;
   target.value = false;
   deltas.clear();
   logic.cheapest(a, target, costs, deltas);
   ASSERT_TRUE(deltas.size() == 1);
   ASSERT_TRUE(deltas[0].new_varval.var == 0);
   ASSERT_TRUE(deltas[0].new_varval.value == false);
   ASSERT_TRUE(deltas[0].result.var_mask.size() == 1);
   ASSERT_TRUE(deltas[0].result.var_mask[0] == true);
   ASSERT_TRUE(deltas[0].result.values.size() == 1);
   ASSERT_TRUE(deltas[0].result.values[0] == false);
}

// A -> B
TEST(LogicTestCase, ModusPonensTest)
{
   ompl_lemur::LogicEngine logic(2);
   std::vector<std::size_t> antecedents;
   antecedents.push_back(0);
   logic.add_conjunction_implication(antecedents, 1);
   logic.initialize();
   
   ompl_lemur::LogicEngine::Assignment a = logic.empty_assignment();
   ompl_lemur::LogicEngine::VarVal target;
   std::vector<double> costs;
   costs.push_back(1.0);
   costs.push_back(2.0);
   
   std::vector<ompl_lemur::LogicEngine::AssignmentDelta> deltas;
   
   target.var = 1;
   target.value = true;
   deltas.clear();
   logic.cheapest(a, target, costs, deltas);
   ASSERT_TRUE(deltas.size() == 1);
   ASSERT_TRUE(deltas[0].new_varval.var == 0);
   ASSERT_TRUE(deltas[0].new_varval.value == true);
   ASSERT_TRUE(deltas[0].result.var_mask.size() == 2);
   ASSERT_TRUE(deltas[0].result.var_mask[0] == true);
   ASSERT_TRUE(deltas[0].result.values.size() == 2);
   ASSERT_TRUE(deltas[0].result.values[0] == true);
}

int main(int argc, char **argv)
{
   testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
