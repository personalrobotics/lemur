/* File: FamilyEffortModel.h
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

namespace ompl_lemur
{

// computed from a (fixed) family snapshot
// serves as an ensemble effort model
// on states, with statuses

// tag=0 assumed to mean unknown!

// eventually, should be able to reset with a new family definition!
class FamilyEffortModel : public EffortModel
{
public:
   
   typedef boost::adjacency_list<boost::vecS,boost::vecS,boost::bidirectionalS>::edge_descriptor PreEdge;
   struct VProps
   {
      std::vector<bool> knowns;
      std::vector<bool> values;
      // below is recalculated every time target si changes!
      double cost_to_go;
      size_t checks_to_go; // if 0, edge_next is garbage!
      PreEdge edge_next;
   };
   struct EProps
   {
      std::size_t var;
      bool value;
      double check_cost;
      PreEdge edge_other;
   };
   typedef boost::adjacency_list<
      boost::vecS, // Edgelist ds, for per-vertex out-edges
      boost::vecS, // VertexList ds, for vertex set
      boost::bidirectionalS, // type of graph
      VProps, // internal (bundled) vertex properties
      EProps // internal (bundled) edge properties
      > Graph;
   typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
   typedef boost::graph_traits<Graph>::vertex_iterator VertexIter;
   typedef boost::graph_traits<Graph>::edge_descriptor Edge;
   typedef boost::graph_traits<Graph>::edge_iterator EdgeIter;
   typedef boost::graph_traits<Graph>::in_edge_iterator InEdgeIter;
   
   const Family family;
   
   // compound sentences (permanent)
   // A1 n A2 n A3 => C
   struct ConjunctionImplication
   {
      std::vector<std::size_t> antecedents;
      std::size_t consequent;
   };
   std::vector<ConjunctionImplication> conjunction_implications;
   
   std::vector< std::vector<bool> > truth_table;
   
   Graph g;
   
   // keyed byvar
   std::vector< std::pair<std::string,Family::Subset> > subsets;
   std::vector<double> var_costs;
   
   ompl::base::SpaceInformationPtr si_target;
   size_t var_target;
   //LogicEngine::VarVal varval_target;
   
   bool si_is_new;
   
   FamilyEffortModel(const Family & in_family):
      family(in_family), si_is_new(false)
   {
      // check that states are the same for all subsets?
      
      // add subsets in order
      for (std::map<std::string, Family::Subset>::const_iterator
           it=family.subsets.begin(); it!=family.subsets.end(); it++)
      {
         subsets.push_back(*it);
         var_costs.push_back(it->second.check_cost);
      }
      
      // convert inclusions/intersections to implications
      for (std::set<Family::Inclusion>::const_iterator
           it=family.inclusions.begin(); it!=family.inclusions.end(); it++)
      {
         size_t var_subset;
         size_t var_superset;
         for (var_subset=0; var_subset<subsets.size(); var_subset++)
            if (subsets[var_subset].first == it->subset)
               break;
         for (var_superset=0; var_superset<subsets.size(); var_superset++)
            if (subsets[var_superset].first == it->superset)
               break;
         if (!(var_subset<subsets.size()) || !(var_superset<subsets.size()))
            throw std::runtime_error("inclusion references unknown subset!");
         ConjunctionImplication x;
         x.antecedents.push_back(var_subset);
         x.consequent = var_superset;
         conjunction_implications.push_back(x);
      }
      
      for (std::set<Family::Intersection>::const_iterator
           it=family.intersections.begin(); it!=family.intersections.end(); it++)
      {
         // find variable index of subset
         size_t var_subset;
         for (var_subset=0; var_subset<subsets.size(); var_subset++)
            if (subsets[var_subset].first == it->subset)
               break;
         if (!(var_subset<subsets.size()))
            throw std::runtime_error("intersection references unknown subset!");
         // one conimp is sup1 n sup2 n sup3 -> sub
         ConjunctionImplication conimp_supstosub;
         // iterate through all supersets
         for (std::set<std::string>::iterator supit=it->supersets.begin(); supit!=it->supersets.end(); supit++)
         {
            size_t var_superset;
            for (var_superset=0; var_superset<subsets.size(); var_superset++)
               if (subsets[var_superset].first == *supit)
                  break;
            if (!(var_superset<subsets.size()))
               throw std::runtime_error("intersection references unknown subset!");
            // one conimp sub -> supX
            ConjunctionImplication conimp_subtosup;
            conimp_subtosup.antecedents.push_back(var_subset);
            conimp_subtosup.consequent = var_superset;
            conjunction_implications.push_back(conimp_subtosup);
            // add to supstosub conimp as well
            conimp_supstosub.antecedents.push_back(var_superset);
         }
         conimp_supstosub.consequent = var_subset;
         conjunction_implications.push_back(conimp_supstosub);
      }
      
      // compute truth table
      std::vector<bool> row(subsets.size(), false);
      for (;;)
      {
         // skip invalid truth table rows
         std::vector<ConjunctionImplication>::iterator implit;
         for (implit=conjunction_implications.begin();
               implit!=conjunction_implications.end(); implit++)
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
         if (implit==conjunction_implications.end())
            truth_table.push_back(row);
         // increment
         std::size_t iconst;
         for (iconst=0; iconst<subsets.size(); iconst++)
         {
            row[iconst] = !row[iconst];
            if (row[iconst])
               break;
         }
         if (!(iconst<subsets.size()))
            break;
      }
      printf("truth table has %lu rows\n", truth_table.size());
      
      printf("constructing family graph ...\n");
      
      Vertex v = add_vertex(g);
      g[v].knowns.resize(subsets.size(),false);
      g[v].values.resize(subsets.size(),false);
      
      for (size_t vidx=0; vidx<num_vertices(g); vidx++)
      {
         Vertex v_source = vertex(vidx, g);
         for (size_t ivar=0; ivar<subsets.size(); ivar++)
         {
            if (g[v_source].knowns[ivar])
               continue;
            Edge edges[2]; // for later cross-linking
            for (int ival=0; ival<2; ival++)
            {
               bool value = (ival==0) ? false : true;
               
               // what would this imply?
               std::vector<bool> implied_knowns = g[v_source].knowns;
               std::vector<bool> implied_values = g[v_source].values;
               implied_knowns[ivar] = true;
               implied_values[ivar] = value;
               
               // what rows are consistent?
               std::vector<bool> seen_true(subsets.size(),false);
               std::vector<bool> seen_false(subsets.size(),false);
               for (size_t irow=0; irow<truth_table.size(); irow++)
               {
                  // is this row inconsistent? then skip
                  size_t ivar_incon;
                  for (ivar_incon=0; ivar_incon<subsets.size(); ivar_incon++)
                     if (implied_knowns[ivar_incon] && implied_values[ivar_incon] != truth_table[irow][ivar_incon])
                        break;
                  if (ivar_incon<subsets.size())
                     continue;
                  // ok, this is consistent
                  for (size_t ivar2=0; ivar2<subsets.size(); ivar2++)
                  {
                     if (truth_table[irow][ivar2])
                        seen_true[ivar2] = true;
                     else
                        seen_false[ivar2] = true;
                  }
               }
               
               // update implied stuff
               for (size_t ivar2=0; ivar2<subsets.size(); ivar2++)
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
               for (boost::tie(vi,vi_end)=vertices(g); vi!=vi_end; ++vi)
               {
                  if (g[*vi].knowns != implied_knowns)
                     continue;
                  size_t i_mismatch;
                  for (i_mismatch=0; i_mismatch<subsets.size(); i_mismatch++)
                  {
                     if (!implied_knowns[i_mismatch])
                        continue;
                     if (g[*vi].values[i_mismatch] != implied_values[i_mismatch])
                        break;
                  }
                  if (!(i_mismatch<subsets.size()))
                     break;
               }
               Vertex v_implied;
               if (vi!=vi_end)
                  v_implied = *vi;
               else
               {
                  v_implied = add_vertex(g);
                  g[v_implied].knowns = implied_knowns;
                  g[v_implied].values = implied_values;
               }
               
               // add new edge
               edges[ival] = add_edge(v_source, v_implied, g).first;
               g[edges[ival]].var = ivar;
               g[edges[ival]].value = value;
               g[edges[ival]].check_cost = var_costs[ivar];
            }
            g[edges[0]].edge_other = edges[1];
            g[edges[1]].edge_other = edges[0];
         }
      }
      
      printf("graph has %lu vertices!\n", num_vertices(g));
   }
   
   // this should clear any caches if its different!
   // custom, not called by e8roadmap planner
   void set_target(ompl::base::SpaceInformationPtr si)
   {
      if (si == si_target)
         return;
      
      si_is_new = true;
      
      // set si_target and var_target
      printf(">> incoming target si: %p\n", si.get());
      si_target = si;
      for (var_target=0; var_target<subsets.size(); var_target++)
         if (subsets[var_target].second.si == si_target)
            break;
      if (!(var_target<subsets.size()))
         throw std::runtime_error("target references unknown subset!");
      
      // do manual reverse dijkstra's algorithm
      pr_bgl::heap_indexed<double> heap;
      VertexIter vi, vi_end;
      for (boost::tie(vi,vi_end)=vertices(g); vi!=vi_end; ++vi)
      {
         if (g[*vi].knowns[var_target] && g[*vi].values[var_target])
         {
            g[*vi].cost_to_go = 0.0;
            g[*vi].checks_to_go = 0;
            size_t vidx = get(get(boost::vertex_index,g), *vi);
            heap.insert(vidx, 0.0);
         }
         else
         {
            g[*vi].cost_to_go = std::numeric_limits<double>::infinity();
            g[*vi].checks_to_go = std::numeric_limits<size_t>::max();
         }
      }
      while (heap.size())
      {
         // pop
         Vertex v = vertex(heap.top_idx(), g);
         heap.remove_min();
         InEdgeIter ei, ei_end;
         for (boost::tie(ei,ei_end)=in_edges(v,g); ei!=ei_end; ++ei)
         {
            assert(target(*ei,g)==v);
            Vertex v_source = source(*ei,g);
            double cost_source = g[v].cost_to_go + g[*ei].check_cost;
            if (!(cost_source < g[v_source].cost_to_go))
               continue;
            g[v_source].cost_to_go = cost_source;
            g[v_source].checks_to_go = g[v].checks_to_go + 1;
            g[v_source].edge_next = *ei;
            size_t vidx_source = get(get(boost::vertex_index,g), v_source);
            if (heap.contains(vidx_source))
               heap.update(vidx_source, cost_source);
            else
               heap.insert(vidx_source, cost_source);
         }
      }
   }
   
   bool has_changed()
   {
      bool ret = si_is_new;
      si_is_new = false;
      return ret;
   }
   
   // p_hat is the sum of the check costs of the cheapest way to show target
   double p_hat(size_t tag, const ompl::base::State * state)
   {
      Vertex v = vertex(tag, g);
      return g[v].cost_to_go;
   }
   
   double x_hat(size_t tag, const ompl::base::State * state)
   {
      Vertex v = vertex(tag, g);
      if (g[v].checks_to_go == std::numeric_limits<size_t>::max())
         return std::numeric_limits<double>::infinity();
      return 0.;
   }
   
   // perform checks for this state (may not make target T or F!)
   // returns true if everything went as planned (so that target is T!)
   bool eval_partial(size_t & tag, const ompl::base::State * state)
   {
      Vertex v = vertex(tag, g);
      assert(g[v].checks_to_go != 0);
      assert(g[v].checks_to_go != std::numeric_limits<size_t>::max());
      bool ret = true;
      while (g[v].checks_to_go)
      {
         Edge e = g[v].edge_next;
         //printf("checking against ivar=%lu desired=%c ...\n",
         //   g[e].var, g[e].value ? 'T' : 'F');
         bool valid = subsets[g[e].var].second.si->isValid(state);
         if (valid != g[e].value)
         {
            v = target(g[e].edge_other, g);
            ret = false;
            break;
         }
         v = target(e, g);
      }
      tag = get(get(boost::vertex_index,g), v);
      return ret;
   }
   
   bool is_evaled(size_t tag)
   {
      Vertex v = vertex(tag, g);
      if (g[v].checks_to_go == 0)
         return true;
      if (g[v].checks_to_go == std::numeric_limits<size_t>::max())
         return true;
      return false;
   }
   
};

}
