/* File: SimpleEffortModel.h
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

namespace ompl_lemur
{

class SimpleEffortModel : public ompl_lemur::EffortModel
{
public:

   ompl::base::SpaceInformationPtr si;
   double check_cost;
   bool has_changed_called;
   
   SimpleEffortModel(
      ompl::base::SpaceInformationPtr si,
      double check_cost):
      si(si),
      check_cost(check_cost),
      has_changed_called(false)
   {
   }
   
   ~SimpleEffortModel()
   {
   }
   
   bool has_changed()
   {
      if (has_changed_called)
         return false;
      has_changed_called = true;
      return true;
   }
   
   // tag=0: unknown
   // tag=1: known-valid
   // tag=2: known-invalid
   
   double p_hat(size_t tag, const ompl::base::State * state)
   {
      if (tag == 0)
         return check_cost;
      else
         return 0.;
   }
   
   double x_hat(size_t tag, const ompl::base::State * state)
   {
      switch (tag)
      {
      case 0: return 0.0;
      case 1: return 0.0;
      case 2: return std::numeric_limits<double>::infinity();
      default: throw std::runtime_error("unknown tag!");
      }
   }
   
   bool eval_partial(size_t & tag, const ompl::base::State * state)
   {
      if (tag != 0)
         throw std::runtime_error("eval_partial called on known edge!");
      bool valid = si->isValid(state);
      if (valid)
      {
         tag = 1;
         return true;
      }
      else
      {
         tag = 2;
         return false;
      }
   }
   
   bool is_evaled(size_t tag)
   {
      if (tag == 0)
         return false;
      else
         return true;
   }
   
};

} // namespace ompl_lemur
