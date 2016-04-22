/*! \file FamilyUtilityChecker.cpp
 * \author Chris Dellin <cdellin@gmail.com>
 * \copyright 2015 Carnegie Mellon University
 * \copyright License: BSD
 */

#include <map>
#include <set>
#include <vector>
#include <cstdio>
#include <boost/graph/adjacency_list.hpp>
#include <ompl/util/Console.h>
#include <ompl/base/StateValidityChecker.h>
#include <pr_bgl/heap_indexed.h>
#include <ompl_lemur/UtilityChecker.h>
#include <ompl_lemur/Family.h>
#include <ompl_lemur/FamilyUtilityChecker.h>

ompl_lemur::FamilyUtilityChecker::FamilyUtilityChecker(
      ompl::base::SpaceInformation * si, const Family & family):
   ompl_lemur::UtilityChecker(si), _family(family), _has_changed(true)
{
   initialize();
}

ompl_lemur::FamilyUtilityChecker::FamilyUtilityChecker(
      const ompl::base::SpaceInformationPtr & si, const Family & family):
   ompl_lemur::UtilityChecker(si), _family(family), _has_changed(true)
{
   initialize();
}

ompl_lemur::FamilyUtilityChecker::~FamilyUtilityChecker()
{
}

void ompl_lemur::FamilyUtilityChecker::initialize()
{
   // add subsets in order
   for (std::set<std::string>::const_iterator
        it=_family.sets.begin(); it!=_family.sets.end(); it++)
   {
      _sets.push_back(*it);
      //var_costs.push_back(it->second.check_cost);
   }
   
   // convert relations to conjunction implications
   for (std::set<Family::Relation>::const_iterator
        it=_family.relations.begin(); it!=_family.relations.end(); it++)
   {
      ConjunctionImplication x;
      // find antecedents
      for (std::set<std::string>::iterator
         ait=it->first.begin(); ait!=it->first.end(); ait++)
      {
         size_t var_antecedent;
         for (var_antecedent=0; var_antecedent<_sets.size(); var_antecedent++)
            if (_sets[var_antecedent] == *ait)
               break;
         if (!(var_antecedent < _sets.size()))
            throw std::runtime_error("relation references unknown subset!");
         x.antecedents.push_back(var_antecedent);
      }
      // find consequent
      size_t var_consequent;
      for (var_consequent=0; var_consequent<_sets.size(); var_consequent++)
         if (_sets[var_consequent] == it->second)
            break;
      if (!(var_consequent < _sets.size()))
         throw std::runtime_error("relation references unknown subset!");
      x.consequent = var_consequent;
      // add
      _conjunction_implications.push_back(x);
   }

   // compute truth table
   std::vector<bool> row(_sets.size(), false);
   for (;;)
   {
      // skip invalid truth table rows
      std::vector<ConjunctionImplication>::iterator implit;
      for (implit=_conjunction_implications.begin();
            implit!=_conjunction_implications.end(); implit++)
      {
         if (row[implit->consequent])
            continue;
         std::vector<std::size_t>::iterator anteit;
         for (anteit=implit->antecedents.begin();
               anteit!=implit->antecedents.end(); anteit++)
            if (!row[*anteit])
               break;
         if (anteit!=implit->antecedents.end())
            continue;
         break;
      }
      if (implit==_conjunction_implications.end())
         _truth_table.push_back(row);
      // increment
      std::size_t iconst;
      for (iconst=0; iconst<_sets.size(); iconst++)
      {
         row[iconst] = !row[iconst];
         if (row[iconst])
            break;
      }
      if (!(iconst<_sets.size()))
         break;
   }
   OMPL_INFORM("Truth table has %lu rows.", _truth_table.size());

   // start with the full unknown state, tag 0
   BeliefState state_initial;
   state_initial.first.resize(_sets.size(),false);
   state_initial.second.resize(_sets.size(),false);
   _belief_states.push_back(state_initial);
   _belief_state_map[state_initial] = 0;
   
   // corresponding non-computed policy
   _policy.push_back(BeliefStatePolicy());
}

void ompl_lemur::FamilyUtilityChecker::start_checking(std::string set_target,
   const std::map<std::string, ompl_lemur::FamilyUtilityChecker::SetChecker> & set_checkers)
{
   // TODO: if checkers and set target are the same,
   // no need to recompute!
   
   // convert target and checkers into var index
   _checkers.clear();
   _var_target = _sets.size();
   for (size_t var=0; var<_sets.size(); var++)
   {
      std::map<std::string, ompl_lemur::FamilyUtilityChecker::SetChecker>::const_iterator
         needle = set_checkers.find(_sets[var]);
      if (needle == set_checkers.end())
      {
         _checkers.clear();
         throw std::runtime_error("checker not foud for a set!");
      }
      _checkers.push_back(needle->second);
      if (_sets[var] == set_target)
         _var_target = var;
   }
   if (_var_target == _sets.size())
   {
      _checkers.clear();
      throw std::runtime_error("set target not found!");
   }
   
   // invalidate policies
   for (unsigned int ui=0; ui<_policy.size(); ui++)
   {
      _policy[ui].computed = false;
   }
   
   // notify planners that values must be recomputed
   _has_changed = true;
}

void ompl_lemur::FamilyUtilityChecker::stop_checking()
{
   _checkers.clear();
}

bool ompl_lemur::FamilyUtilityChecker::hasChanged()
{
   bool ret = _has_changed;
   _has_changed = false;
   return ret;
}
   
bool ompl_lemur::FamilyUtilityChecker::isKnown(size_t tag) const
{
   const BeliefState & bstate = _belief_states[tag];
   return bstate.first[_var_target];
}

bool ompl_lemur::FamilyUtilityChecker::isKnownInvalid(size_t tag) const
{
   const BeliefState & bstate = _belief_states[tag];
   if (!bstate.first[_var_target])
      return false;
   if (bstate.second[_var_target])
      return false;
   return true;
}

double ompl_lemur::FamilyUtilityChecker::getPartialEvalCost(size_t tag, const ompl::base::State * state) const
{
   const BeliefState & bstate = _belief_states[tag];
   if (bstate.first[_var_target])
      return 0.0;
   if (!_policy[tag].computed)
      compute_policy(tag);
   return _policy[tag].cost_to_go;
}

bool ompl_lemur::FamilyUtilityChecker::isValidPartialEval(size_t & tag, const ompl::base::State * state) const
{
   // continue going as planned!
   for (;;)
   {
      const BeliefState bstate = _belief_states[tag];
      if (!_policy[tag].computed)
         compute_policy(tag);
      
      bool result_desired = _policy[tag].result_desired;
      bool valid = _checkers[_policy[tag].iset].second->isValid(state);
      
      // update
      size_t tag_on_result = valid ? _policy[tag].tag_on_valid : _policy[tag].tag_on_invalid;
      if (!tag_on_result)
      {
         tag_on_result = tagIfSetKnown(tag, _policy[tag].iset, valid);
         
         // update tag_on_result
         if (valid)
            _policy[tag].tag_on_valid = tag_on_result;
         else
            _policy[tag].tag_on_invalid = tag_on_result;
      }
      
      tag = tag_on_result;
      
      // should we continue?
      if (valid != result_desired)
         return false;
      
      // are we done?
      if (isKnown(tag))
         return !isKnownInvalid(tag);
   }
}

// this is called if _policy[tag].computed is false
// right now, this is completely self-contained ...
void ompl_lemur::FamilyUtilityChecker::compute_policy(size_t tag) const
{
   const BeliefState & initial_bstate = _belief_states[tag];
   
   // TODO: do we already know for this tag?
   
   std::vector< BeliefState > my_states;
   std::map< BeliefState, size_t > my_state_map;
   
   // points back to initial_state
   // (parent_idx, (iset,result))
   std::vector< std::pair<size_t,std::pair<size_t,bool> > > my_parents;
   
   // do forward dijkstra's search
   // index: index into my_states
   // cost: cost so far (from belief_state)
   pr_bgl::heap_indexed<double> heap;
   
   BeliefState initial_bstate_copy = initial_bstate;
   my_states.push_back(initial_bstate_copy);
   my_parents.push_back(std::make_pair(0,std::make_pair(0,false)));
   my_state_map[initial_bstate] = 0;
   heap.insert(0, 0.0);
   
   size_t popped_idx;
   double popped_distance;
   bool goal_found = false;
   while (heap.size())
   {
      // pop
      popped_idx = heap.top_idx();
      popped_distance = heap.top_key();
      const BeliefState popped_bstate = my_states[popped_idx];
      heap.remove_min();
      
      // are we done??
      if (popped_bstate.first[_var_target] && popped_bstate.second[_var_target])
      {
         goal_found = true;
         break;
      }
      
      // consider all potential tests and results
      for (size_t iset=0; iset<_sets.size(); iset++)
      {
         // obviously, skip tests for states we already know
         if (popped_bstate.first[iset])
            continue;
         for (int iresult=0; iresult<2; iresult++)
         {
            bool result = (iresult==0) ? false : true;
            
            // new belief
            BeliefState new_bstate = popped_bstate;
            new_bstate.first[iset] = true; // known
            new_bstate.second[iset] = result;
            
            // compute implications
            // for each var, which consistent truth table rows have it true or false?
            std::vector<bool> seen_true(_sets.size(),false);
            std::vector<bool> seen_false(_sets.size(),false);
            for (size_t irow=0; irow<_truth_table.size(); irow++)
            {
               // is this row inconsistent? then skip
               size_t ivar_incon;
               for (ivar_incon=0; ivar_incon<_sets.size(); ivar_incon++)
                  if (new_bstate.first[ivar_incon] && new_bstate.second[ivar_incon] != _truth_table[irow][ivar_incon])
                     break;
               if (ivar_incon<_sets.size())
                  continue;
               // ok, this is consistent
               for (size_t ivar2=0; ivar2<_sets.size(); ivar2++)
               {
                  if (_truth_table[irow][ivar2])
                     seen_true[ivar2] = true;
                  else
                     seen_false[ivar2] = true;
               }
            }
            
            // update new_state with implied stuff
            for (size_t ivar2=0; ivar2<_sets.size(); ivar2++)
            {
               if (new_bstate.first[ivar2])
                  continue;
               if (seen_true[ivar2] && seen_false[ivar2])
                  continue;
               new_bstate.first[ivar2] = true;
               new_bstate.second[ivar2] = seen_true[ivar2];
            }
            
            // if this implies the target is false, then don't even bother!
            if (new_bstate.first[_var_target] && !new_bstate.second[_var_target])
               continue;
            
            // compute the cost to get to this vertex
            double new_distance = popped_distance + _checkers[iset].first;
            
            // de-duplicate new state, get its index
            size_t new_idx;
            std::map< BeliefState, size_t >::iterator needle = my_state_map.find(new_bstate);
            if (needle != my_state_map.end())
            {
               new_idx = needle->second;
               // if it's in the heap, then we might have found a new value for it!
               if (heap.contains(new_idx))
               {
                  if (new_distance < heap.key_of(new_idx))
                  {
                     heap.update(new_idx, new_distance);
                     my_parents[new_idx] = std::make_pair(popped_idx,std::make_pair(iset,result));
                  }
               }
            }
            else
            {
               // never seen before state;
               // add it to my_states with the correct cost,
               // and add it to the open list (heap)
               new_idx = my_states.size();
               my_states.push_back(new_bstate);
               my_parents.push_back(std::make_pair(popped_idx,std::make_pair(iset,result)));
               my_state_map[new_bstate] = new_idx;
               heap.insert(new_idx, new_distance); // check_cost
            }
         }
      }
   }
   
   if (!goal_found)
      throw std::runtime_error("borking (no path found?)");
   
   // walk backwards
   size_t child = popped_idx;
   while (my_parents[child].first)
      child = my_parents[child].first;

   _policy[tag].computed = true;
   _policy[tag].cost_to_go = popped_distance;
   _policy[tag].iset = my_parents[child].second.first;
   _policy[tag].result_desired = my_parents[child].second.second;
   _policy[tag].tag_on_valid = 0;
   _policy[tag].tag_on_invalid = 0;
}

size_t ompl_lemur::FamilyUtilityChecker::getSetIndex(const std::string & set_name) const
{
   size_t iset;
   for (iset=0; iset<_sets.size(); iset++)
      if (_sets[iset] == set_name)
         break;
   if (iset<_sets.size())
      return iset;
   else
      throw std::runtime_error("set not found!");
}

size_t ompl_lemur::FamilyUtilityChecker::tagIfSetKnown(size_t tag_in, size_t iset, bool value) const
{
   const BeliefState bstate = _belief_states[tag_in];
   
   // compute resulting belief (it could be new!)
   BeliefState new_bstate = bstate;
   new_bstate.first[iset] = true; // known
   new_bstate.second[iset] = value;
   
   // compute implications
   // for each var, which consistent truth table rows have it true or false?
   std::vector<bool> seen_true(_sets.size(),false);
   std::vector<bool> seen_false(_sets.size(),false);
   for (size_t irow=0; irow<_truth_table.size(); irow++)
   {
      // is this row inconsistent? then skip
      size_t ivar_incon;
      for (ivar_incon=0; ivar_incon<_sets.size(); ivar_incon++)
         if (new_bstate.first[ivar_incon] && new_bstate.second[ivar_incon] != _truth_table[irow][ivar_incon])
            break;
      if (ivar_incon<_sets.size())
         continue;
      // ok, this is consistent
      for (size_t ivar2=0; ivar2<_sets.size(); ivar2++)
      {
         if (_truth_table[irow][ivar2])
            seen_true[ivar2] = true;
         else
            seen_false[ivar2] = true;
      }
   }
   
   // update new_state with implied stuff
   for (size_t ivar2=0; ivar2<_sets.size(); ivar2++)
   {
      if (new_bstate.first[ivar2])
         continue;
      if (seen_true[ivar2] && seen_false[ivar2])
         continue;
      new_bstate.first[ivar2] = true;
      new_bstate.second[ivar2] = seen_true[ivar2];
   }
   
   // check if it's in our set of existing tags ...
   // get the new tag in any case
   std::map< BeliefState, size_t >::iterator needle = _belief_state_map.find(new_bstate);
   if (needle != _belief_state_map.end())
   {
      return needle->second;
   }
   else
   {
      size_t new_tag = _belief_states.size();
      _belief_states.push_back(new_bstate);
      _belief_state_map[new_bstate] = new_tag;
      _policy.push_back(BeliefStatePolicy());
      return new_tag;
   }
}
