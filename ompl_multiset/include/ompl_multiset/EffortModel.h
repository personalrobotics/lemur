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
   virtual bool has_changed() = 0;
   virtual double p_hat(size_t tag, ompl::base::State * state) = 0;
   virtual double x_hat(size_t tag) = 0;
   virtual bool eval_partial(size_t & tag, ompl::base::State * state) = 0;
   virtual bool is_evaled(size_t tag) = 0;
};

} // namespace ompl_multiset
