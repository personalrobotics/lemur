/*! \file UtilityChecker.cpp
 * \author Chris Dellin <cdellin@gmail.com>
 * \copyright 2015 Carnegie Mellon University
 * \copyright License: BSD
 */

#include <stdexcept>
#include <ompl/base/StateValidityChecker.h>
#include <ompl_lemur/UtilityChecker.h>

ompl_lemur::BinaryUtilityChecker::BinaryUtilityChecker(
      ompl::base::SpaceInformation * si,
      ompl::base::StateValidityCheckerPtr wrapped,
      double check_cost):
   ompl_lemur::UtilityChecker(si),
   _has_changed(true), _wrapped(wrapped), _check_cost(check_cost)
{   
}

ompl_lemur::BinaryUtilityChecker::BinaryUtilityChecker(
      const ompl::base::SpaceInformationPtr & si,
      ompl::base::StateValidityCheckerPtr wrapped,
      double check_cost):
   ompl_lemur::UtilityChecker(si),
   _has_changed(true), _wrapped(wrapped), _check_cost(check_cost)
{
}
   
ompl_lemur::BinaryUtilityChecker::~BinaryUtilityChecker()
{
}

bool ompl_lemur::BinaryUtilityChecker::hasChanged()
{
   bool ret = _has_changed;
   _has_changed = false;
   return ret;
}

bool ompl_lemur::BinaryUtilityChecker::isKnown(size_t tag) const
{
   switch (tag)
   {
      case TAG_UNKNOWN:
         return false;
      case TAG_KNOWN_VALID:
      case TAG_KNOWN_INVALID:
         return true;
      default:
         throw std::runtime_error("unknown tag!");
   }
}

bool ompl_lemur::BinaryUtilityChecker::isKnownInvalid(size_t tag) const
{
   switch (tag)
   {
      case TAG_UNKNOWN:
      case TAG_KNOWN_VALID:
         return false;
      case TAG_KNOWN_INVALID:
         return true;
      default:
         throw std::runtime_error("unknown tag!");
   }
}

double ompl_lemur::BinaryUtilityChecker::getPartialEvalCost(size_t tag, const ompl::base::State * state) const
{
   switch (tag)
   {
      case TAG_UNKNOWN:
         return _check_cost;
      case TAG_KNOWN_VALID:
      case TAG_KNOWN_INVALID:
         return 0.0;
      default:
         throw std::runtime_error("unknown tag!");
   }
}

bool ompl_lemur::BinaryUtilityChecker::isValidPartialEval(size_t & tag, const ompl::base::State * state) const
{
   if (tag != TAG_UNKNOWN)
      throw std::runtime_error("isValidPartialEval called on known edge!");
   bool is_valid = _wrapped->isValid(state);
   tag = is_valid ? TAG_KNOWN_VALID : TAG_KNOWN_INVALID;
   return is_valid;
}
