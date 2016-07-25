/*! \file FamilyLazyUtilityChecker.h
 * \author Chris Dellin <cdellin@gmail.com>
 * \copyright 2015 Carnegie Mellon University
 * \copyright License: BSD
 */

namespace ompl_lemur
{

/*! \brief Use a Family as a UtilityChecker.
 * 
 * This class implements an optimistic search over a family of sets.
 * Internally, this class effectively computes a graph of known states
 * over all N sets (naively 3^N states, modulo any relations).
 * For a given target set and costed checkers, it computes the
 * optimal set of checks (and their results) that would imply
 * membership in the target set.
 * 
 * This now implements lazy computation of tags / optimistic plans.
 * 
 * This used to be called `FamilyEffortModel`.
 */
class FamilyUtilityChecker : public UtilityChecker
{
public:
   
   FamilyUtilityChecker(ompl::base::SpaceInformation * si, const Family & family);
   FamilyUtilityChecker(const ompl::base::SpaceInformationPtr & si, const Family & family);
   ~FamilyUtilityChecker();
   
   // called by constructor
   void initialize();

public: // set by calling code (e.g. or_lemur::FamilyPlanner)

   typedef std::pair<double, ompl::base::StateValidityCheckerPtr> SetChecker;
   
   void start_checking(std::string set_target, const std::map<std::string, SetChecker> & set_checkers);
   void stop_checking();

public: // used internally and by ompl_lemur::FamilyTagCache

   // this will throw if set not found!
   size_t getSetIndex(const std::string & set_name) const;

   size_t numTags() const { return _belief_states.size(); }
   
   // this may add a new set (changing the result of numTags())
   size_t tagIfSetKnown(size_t tag_in, size_t iset, bool value) const;

public: // used by planner

   bool hasChanged();
   
   bool isKnown(size_t tag) const;
   
   bool isKnownInvalid(size_t tag) const;
   
   double getPartialEvalCost(size_t tag, const ompl::base::State * state) const;
   
   bool isValidPartialEval(size_t & tag, const ompl::base::State * state) const;
   
private:
   const Family _family;
   bool _has_changed;
   
   // view into the family
   std::vector<std::string> _sets;;
   
   // compound sentences (permanent)
   // A1 n A2 n A3 => C
   struct ConjunctionImplication
   {
      std::vector<std::size_t> antecedents;
      std::size_t consequent;
   };
   std::vector<ConjunctionImplication> _conjunction_implications;
   
   std::vector< std::vector<bool> > _truth_table;
   
// these are set by start_checking() and persist until stop_checkint()

   size_t _var_target;
   
   // subsumes var_costs and subsets[g[e].var].second.si->isValid()
   std::vector<SetChecker> _checkers;
   
   // these are only the states that are on optimal paths
   // to the target from existing states!
   typedef std::pair< std::vector<bool>, std::vector<bool> > BeliefState;
   mutable std::vector< BeliefState > _belief_states;
   mutable std::map< BeliefState, size_t > _belief_state_map;
   
   // this is cost and target dependent stuff:
   struct BeliefStatePolicy
   {
      bool computed;
      double cost_to_go;
      size_t iset;
      bool result_desired;
      size_t tag_on_valid; // 0 if not encountered yet
      size_t tag_on_invalid; // 0 if not encountered yet
      BeliefStatePolicy(): computed(false) {}
   };
   // this always must have the same length as _belief_states
   // (even if they are not computed yet)
   mutable std::vector< BeliefStatePolicy > _policy;
   
   // this is called if _policy[tag].computed is false
   void compute_policy(size_t tag) const;
};

#ifdef OMPL_LEMUR_HAS_BOOSTSMARTPTRS
typedef boost::shared_ptr<FamilyUtilityChecker> FamilyUtilityCheckerPtr;
#else
typedef std::shared_ptr<FamilyUtilityChecker> FamilyUtilityCheckerPtr;
#endif

} // namespace ompl_lemur
