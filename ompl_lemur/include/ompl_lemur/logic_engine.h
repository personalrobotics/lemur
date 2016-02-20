
namespace ompl_lemur
{

// custom propositional logic engine
// over a finite set of propositional variables (aka constants)
// assignment means partial assignment
class LogicEngine
{
public:
   std::size_t num_vars;
   bool initialized;
   LogicEngine(std::size_t num_vars):
      num_vars(num_vars),
      initialized(false)
   {
   }
   
   struct Assignment
   {
      std::vector<bool> var_mask; // true == is assigned
      std::vector<bool> values;
      bool operator==(const Assignment & rhs) const
      {
         if (var_mask != rhs.var_mask)
            return false;
         if (values.size() != rhs.values.size())
            return false;
         for (std::size_t i=0; i<values.size(); i++)
         {
            if (!var_mask[i])
               continue;
            if (values[i] != rhs.values[i])
               return false;
         }
         return true;
      }
   };
   
   struct VarVal
   {
      std::size_t var;
      bool value;
   };
   struct AssignmentDelta
   {
      VarVal new_varval;
      Assignment result;
   };
   struct SearchVertex
   {
      double cost_sofar;
      AssignmentDelta delta;
      std::vector<bool> row_is_consistent;
      std::size_t parent_idx;
   };
   
   Assignment empty_assignment()
   {
      Assignment pa;
      pa.var_mask.resize(num_vars, false);
      pa.values.resize(num_vars, false);
      return pa;
   }
   
   // compound sentences (permanent)
   // A1 n A2 n A3 => C
   struct ConjunctionImplication
   {
      std::vector<std::size_t> antecedents;
      std::size_t consequent;
   };
   std::vector<ConjunctionImplication> conjunction_implications;
   void add_conjunction_implication(
      std::vector<std::size_t> antecedents, std::size_t consequent)
   {
      ConjunctionImplication x;
      x.antecedents = antecedents;
      x.consequent = consequent;
      conjunction_implications.push_back(x);
   }
   
   std::vector< std::vector<bool> > truth_table;
   
   bool initialize()
   {
      if (initialized)
         return false;
      
      // generate all possible truth table rows
      std::vector<bool> row(num_vars, false);
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
         for (iconst=0; iconst<num_vars; iconst++)
         {
            row[iconst] = !row[iconst];
            if (row[iconst])
               break;
         }
         if (!(iconst<num_vars))
            break;
      }
      
      
      initialized = true;
      return true;
   }
   
   void cheapest(const Assignment & current,
      const VarVal & target_assignment,
      const std::vector<double> & var_costs,
      std::vector<AssignmentDelta> & deltas)
   {
      // open and closed list
      std::vector<SearchVertex> generated;
      pr_bgl::heap_indexed<double> heap;
      
      SearchVertex v_start;
      v_start.cost_sofar = 0.0;
      v_start.delta.result = current;
      v_start.row_is_consistent.resize(truth_table.size());
      v_start.parent_idx = 0;
      for (std::size_t irow=0; irow<truth_table.size(); irow++)
      {
         // is this row inconsistent?
         std::size_t ic = 0;
         for (; ic<num_vars; ic++)
         {
            if (!current.var_mask[ic])
               continue;
            if (current.values[ic] != truth_table[irow][ic])
               break;
         }
         if (ic<num_vars)
            v_start.row_is_consistent[irow] = false;
         else
            v_start.row_is_consistent[irow] = true;
      }
      heap.insert(generated.size(), v_start.cost_sofar);
      generated.push_back(v_start);
      
      std::size_t top_idx;
      for (;;)
      {
         // grab min from heap
         if (!heap.size())
         {
            fprintf(stderr,"error, no path found!\n");
            abort();
         }
         top_idx = heap.top_idx();
         SearchVertex v_top = generated[top_idx];
         heap.remove_min();
         
         // is it the goal?
         if (v_top.delta.result.var_mask[target_assignment.var])
         {
            if (v_top.delta.result.values[target_assignment.var]
                  == target_assignment.value)
               break;
            continue; // well this definitely wont work
         }
         
         // generate successors
         for (std::size_t ic_new=0; ic_new<num_vars; ic_new++)
         {
            if (v_top.delta.result.var_mask[ic_new])
               continue;
            // what if we did this assignment?
            for (int valnum=0; valnum<2; valnum++)
            {
               bool value = valnum ? true : false;
               
               SearchVertex v_succ = v_top;
               v_succ.delta.new_varval.var = ic_new;
               v_succ.delta.new_varval.value = value;
               v_succ.cost_sofar += var_costs[ic_new];
               v_succ.parent_idx = top_idx;
               
               // remove inconsistent rows
               for (std::size_t irow=0; irow<truth_table.size(); irow++)
               {
                  if (!v_succ.row_is_consistent[irow])
                     continue;
                  if (truth_table[irow][ic_new] != value)
                     v_succ.row_is_consistent[irow] = false;
               }
               // find first consistent row
               // if they're all inconsistent, then we're done
               std::size_t irow_first;
               for (irow_first=0; irow_first<truth_table.size(); irow_first++)
                  if (v_succ.row_is_consistent[irow_first])
                     break;
               if (!(irow_first<truth_table.size()))
                  continue;
               
               // determine new (more specific) implied partial assignment
               v_succ.delta.result.var_mask[ic_new] = true;
               v_succ.delta.result.values[ic_new] = value;
               for (std::size_t ic=0; ic<num_vars; ic++)
               {
                  if (v_succ.delta.result.var_mask[ic])
                     continue;
                  bool candidate_value = truth_table[irow_first][ic];
                  std::size_t irow_different;
                  for (irow_different=irow_first+1; irow_different<truth_table.size(); irow_different++)
                  {
                     if (!v_succ.row_is_consistent[irow_different])
                        continue;
                     if (truth_table[irow_different][ic] != candidate_value)
                        break;
                  }
                  if (irow_different<truth_table.size())
                     continue;
                  v_succ.delta.result.var_mask[ic] = true;
                  v_succ.delta.result.values[ic] = candidate_value;
               }
               
               // does this partial assignment already exist?
               std::size_t gi;
               for (gi=0; gi<generated.size(); gi++)
                  if (v_succ.delta.result == generated[gi].delta.result)
                     break;
               if (gi<generated.size()) // already there!
               {
                  if (v_succ.cost_sofar < generated[gi].cost_sofar)
                  {
                     generated[gi].cost_sofar = v_succ.cost_sofar;
                     generated[gi].parent_idx = top_idx;
                     heap.update(gi, v_succ.cost_sofar);
                  }
               }
               else
               {
                  heap.insert(generated.size(), v_succ.cost_sofar);
                  generated.push_back(v_succ);
               }
            }
         }
      }
      
      std::vector<std::size_t> path_idxs;
      path_idxs.push_back(top_idx);
      while (path_idxs.back())
         path_idxs.push_back(generated[path_idxs.back()].parent_idx);
      std::reverse(path_idxs.begin(), path_idxs.end());
      
      for (std::size_t ip=1; ip<path_idxs.size(); ip++)
         deltas.push_back(generated[path_idxs[ip]].delta);
   }
   
};

} // ompl_lemur
