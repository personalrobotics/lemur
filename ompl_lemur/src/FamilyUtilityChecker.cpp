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
   
   Vertex v = add_vertex(_g);
   _g[v].knowns.resize(_sets.size(),false);
   _g[v].values.resize(_sets.size(),false);
   
   for (size_t vidx=0; vidx<num_vertices(_g); vidx++)
   {
      Vertex v_source = vertex(vidx, _g);
      for (size_t ivar=0; ivar<_sets.size(); ivar++)
      {
         if (_g[v_source].knowns[ivar])
            continue;
         Edge edges[2]; // for later cross-linking
         for (int ival=0; ival<2; ival++)
         {
            bool value = (ival==0) ? false : true;
            
            // what would this imply?
            std::vector<bool> implied_knowns = _g[v_source].knowns;
            std::vector<bool> implied_values = _g[v_source].values;
            implied_knowns[ivar] = true;
            implied_values[ivar] = value;
            
            // what rows are consistent?
            std::vector<bool> seen_true(_sets.size(),false);
            std::vector<bool> seen_false(_sets.size(),false);
            for (size_t irow=0; irow<_truth_table.size(); irow++)
            {
               // is this row inconsistent? then skip
               size_t ivar_incon;
               for (ivar_incon=0; ivar_incon<_sets.size(); ivar_incon++)
                  if (implied_knowns[ivar_incon] && implied_values[ivar_incon] != _truth_table[irow][ivar_incon])
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
            
            // update implied stuff
            for (size_t ivar2=0; ivar2<_sets.size(); ivar2++)
            {
               if (implied_knowns[ivar2])
                  continue;
               if (seen_true[ivar2] && seen_false[ivar2])
                  continue;
               implied_knowns[ivar2] = true;
               implied_values[ivar2] = seen_true[ivar2];
            }
            
            // does this implied vertex already exist?
            VertexIter vi, vi_end;
            for (boost::tie(vi,vi_end)=vertices(_g); vi!=vi_end; ++vi)
            {
               if (_g[*vi].knowns != implied_knowns)
                  continue;
               size_t i_mismatch;
               for (i_mismatch=0; i_mismatch<_sets.size(); i_mismatch++)
               {
                  if (!implied_knowns[i_mismatch])
                     continue;
                  if (_g[*vi].values[i_mismatch] != implied_values[i_mismatch])
                     break;
               }
               if (!(i_mismatch<_sets.size()))
                  break;
            }
            Vertex v_implied;
            if (vi!=vi_end)
               v_implied = *vi;
            else
            {
               v_implied = add_vertex(_g);
               _g[v_implied].knowns = implied_knowns;
               _g[v_implied].values = implied_values;
            }
            
            // add new edge
            edges[ival] = add_edge(v_source, v_implied, _g).first;
            _g[edges[ival]].var = ivar;
            _g[edges[ival]].value = value;
            //_g[edges[ival]].check_cost = _var_costs[ivar];
         }
         _g[edges[0]].edge_other = edges[1];
         _g[edges[1]].edge_other = edges[0];
      }
   }
   
   OMPL_INFORM("Family graph has %lu vertices.", num_vertices(_g));
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
   
   // copy check costs into edges
   EdgeIter ei, ei_end;
   for (boost::tie(ei,ei_end)=edges(_g); ei!=ei_end; ++ei)
      _g[*ei].check_cost = _checkers[_g[*ei].var].first; // check_cost
   
   // do manual reverse dijkstra's algorithm
   pr_bgl::heap_indexed<double> heap;
   VertexIter vi, vi_end;
   for (boost::tie(vi,vi_end)=vertices(_g); vi!=vi_end; ++vi)
   {
      if (_g[*vi].knowns[_var_target] && _g[*vi].values[_var_target])
      {
         _g[*vi].cost_to_go = 0.0;
         _g[*vi].checks_to_go = 0;
         size_t vidx = get(get(boost::vertex_index,_g), *vi);
         heap.insert(vidx, 0.0);
      }
      else
      {
         _g[*vi].cost_to_go = std::numeric_limits<double>::infinity();
         _g[*vi].checks_to_go = std::numeric_limits<size_t>::max();
      }
   }
   while (heap.size())
   {
      // pop
      Vertex v = vertex(heap.top_idx(), _g);
      heap.remove_min();
      InEdgeIter ei, ei_end;
      for (boost::tie(ei,ei_end)=in_edges(v,_g); ei!=ei_end; ++ei)
      {
         assert(target(*ei,_g)==v);
         Vertex v_source = source(*ei,_g);
         double cost_source = _g[v].cost_to_go + _g[*ei].check_cost;
         if (!(cost_source < _g[v_source].cost_to_go))
            continue;
         _g[v_source].cost_to_go = cost_source;
         _g[v_source].checks_to_go = _g[v].checks_to_go + 1;
         _g[v_source].edge_next = *ei;
         size_t vidx_source = get(get(boost::vertex_index,_g), v_source);
         if (heap.contains(vidx_source))
            heap.update(vidx_source, cost_source);
         else
            heap.insert(vidx_source, cost_source);
      }
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
   Vertex v = vertex(tag, _g);
   if (_g[v].checks_to_go == 0)
      return true;
   if (_g[v].checks_to_go == std::numeric_limits<size_t>::max())
      return true;
   return false;
}

bool ompl_lemur::FamilyUtilityChecker::isKnownInvalid(size_t tag) const
{
   Vertex v = vertex(tag, _g);
   if (_g[v].checks_to_go == std::numeric_limits<size_t>::max())
      return true;
   return false;
}

double ompl_lemur::FamilyUtilityChecker::getPartialEvalCost(size_t tag, const ompl::base::State * state) const
{
   Vertex v = vertex(tag, _g);
   return _g[v].cost_to_go;
}

bool ompl_lemur::FamilyUtilityChecker::isValidPartialEval(size_t & tag, const ompl::base::State * state) const
{
   Vertex v = vertex(tag, _g);
   assert(_g[v].checks_to_go != 0);
   assert(_g[v].checks_to_go != std::numeric_limits<size_t>::max());
   bool ret = true;
   while (_g[v].checks_to_go)
   {
      Edge e = _g[v].edge_next;
      //printf("checking against ivar=%lu desired=%c ...\n",
      //   _g[e].var, _g[e].value ? 'T' : 'F');
      bool valid = _checkers[_g[e].var].second->isValid(state);
      if (valid != _g[e].value)
      {
         v = target(_g[e].edge_other, _g);
         ret = false;
         break;
      }
      v = target(e, _g);
   }
   tag = get(get(boost::vertex_index,_g), v);
   return ret;
}
