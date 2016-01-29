/* File: aborting_space_information.h
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

namespace ompl_lemur
{

class AbortingValidityChecker: public ompl::base::StateValidityChecker
{
public:
   AbortingValidityChecker(const ompl::base::SpaceInformationPtr &si):
      ompl::base::StateValidityChecker(si)
   {
   }
   bool isValid(const ompl::base::State * state) const
   {
      throw std::runtime_error("AbortingValidityChecker isValid() called!");
   }
};

ompl::base::SpaceInformationPtr get_aborting_space_information(ompl::base::StateSpacePtr space)
{
   ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(space));
   si->setStateValidityChecker(
      ompl::base::StateValidityCheckerPtr(new AbortingValidityChecker(si))
   );
   si->setup();
   return si;
}

} // namespace ompl_lemur
