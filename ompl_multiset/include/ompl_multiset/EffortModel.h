/* File: EffortModel.h
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

namespace ompl_multiset
{

class EffortModel
{
public:
   // if true, planner must recalculate wlazy for all edges
   virtual bool has_changed() = 0;
   
   virtual double p_hat(size_t tag, const ompl::base::State * state) = 0;
   virtual double x_hat(size_t tag, const ompl::base::State * state) = 0;
   
   // returns true if everything went as planned (so that target is T!)
   virtual bool eval_partial(size_t & tag, const ompl::base::State * state) = 0;
   
   virtual bool is_evaled(size_t tag) = 0;
};

class TagCache
{
public:

   virtual void load_vertex(size_t v_index, size_t & v_tag) = 0;
   virtual void load_edge(size_t e_index, std::vector< size_t > & e_tags) = 0;
   
   virtual void save_vertex(size_t v_index, size_t & v_tag) = 0;
   virtual void save_edge(size_t e_index, std::vector< size_t > & e_tags) = 0;
};

} // namespace ompl_multiset
