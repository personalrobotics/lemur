/*! \file UtilityChecker.h
 * \author Chris Dellin <cdellin@gmail.com>
 * \copyright 2015 Carnegie Mellon University
 * \copyright License: BSD
 */

namespace ompl_lemur
{

/*! \brief Type of state validity checker which enables partial
 *         evaluation of tagged states.
 * 
 * This used to be called `EffortModel`.
 */
class UtilityChecker : public ompl::base::StateValidityChecker
{
public:
   UtilityChecker(ompl::base::SpaceInformation * si):
      ompl::base::StateValidityChecker(si)
   {
   }
   
   UtilityChecker(const ompl::base::SpaceInformationPtr & si):
      ompl::base::StateValidityChecker(si)
   {
   }
   
   virtual ~UtilityChecker()
   {
   }
   
   /*! \brief Determine if the underlying utility model has changed
    *         since the last call to hasChanged().
    * 
    * If true, planner must recalculate wlazy for all edges.
    * 
    * Was `EffortModel::has_changed()`.
    */
   virtual bool hasChanged() = 0;
   
   /*! \brief Determine whether a state with tag `tag` is known
    *         either valid or invalid.
    * 
    * Was `EffortModel::is_evaled()`.
    */
   virtual bool isKnown(size_t tag) const = 0;
   
   /*! \brief Determine whether a state with tag `tag` is known
    *         to be invalid.
    * 
    * Was `EffortModel::x_hat()`.
    */
   virtual bool isKnownInvalid(size_t tag) const = 0;
   
   /*! \brief Determines the optimistic planning cost entailed to call
    *         isValidPartialEval().
    * 
    * This is optimistic (if everything goes right for next call to
    * isValidPartialEval())
    * 
    * Was `EffortModel::p_hat()`.
    */
   virtual double getPartialEvalCost(size_t tag, const ompl::base::State * state) const = 0;
   
   /*! \brief Conduct the optimistic set of evaluations.
    * 
    * Returns true if everything went as planned (so that target is T!).
    *
    * If false, not necessary known invalid!
    *
    * This updates the `tag` argument.
    * 
    * Was `EffortModel::eval_partial()`.
    */
   virtual bool isValidPartialEval(size_t & tag, const ompl::base::State * state) const = 0;
   
   /*! \brief Tag-ignorant validity check wrapper.
    * 
    * Users of UtilityChecker will usually not call this, because they
    * want to take advantage of the tag mechanism instead.
    */
   virtual bool isValid(const ompl::base::State * state) const
   {
      size_t tag = 0;
      while (!isKnown(tag))
         isValidPartialEval(tag, state);
      return !isKnownInvalid(tag);
   }
};

#ifdef OMPL_LEMUR_HAS_BOOSTSMARTPTRS
typedef boost::shared_ptr<UtilityChecker> UtilityCheckerPtr;
#else
typedef std::shared_ptr<UtilityChecker> UtilityCheckerPtr;
#endif


/*! \brief Adaptor to use a basic ompl::base::StateValidityChecker
 *         as a UtilityChecker.
 * 
 * This assumes that each check is constant planning cost, which
 * defaults to 1.0.
 * 
 * This used to be called `SimpleEffortModel`.
 */
class BinaryUtilityChecker : public UtilityChecker
{
public:
   
   enum
   {
      TAG_UNKNOWN = 0,
      TAG_KNOWN_VALID = 1,
      TAG_KNOWN_INVALID = 2
   };

   bool _has_changed;
   ompl::base::StateValidityCheckerPtr _wrapped;
   double _check_cost;
   
   BinaryUtilityChecker(ompl::base::SpaceInformation * si,
      ompl::base::StateValidityCheckerPtr wrapped,
      double check_cost=1.0);
   BinaryUtilityChecker(const ompl::base::SpaceInformationPtr & si,
      ompl::base::StateValidityCheckerPtr wrapped,
      double check_cost=1.0);
   ~BinaryUtilityChecker();
   
   bool hasChanged();
   bool isKnown(size_t tag) const;
   bool isKnownInvalid(size_t tag) const;
   double getPartialEvalCost(size_t tag, const ompl::base::State * state) const;
   bool isValidPartialEval(size_t & tag, const ompl::base::State * state) const;
};

#ifdef OMPL_LEMUR_HAS_BOOSTSMARTPTRS
typedef boost::shared_ptr<BinaryUtilityChecker> BinaryUtilityCheckerPtr;
#else
typedef std::shared_ptr<BinaryUtilityChecker> BinaryUtilityCheckerPtr;
#endif

} // namespace ompl_lemur
