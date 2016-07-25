/*! \file SamplerGenMonkeyPatch.h
 * \author Chris Dellin <cdellin@gmail.com>
 * \copyright 2015 Carnegie Mellon University
 * \copyright License: BSD
 */

/* requires:
#include <ompl_lemur/config.h>
#include <ompl/base/StateSampler.h>
 */

namespace ompl_lemur
{

#ifdef OMPL_LEMUR_HAS_BOOSTSMARTPTRS
boost::mt19937 &
#else
std::mt19937 &
#endif
SamplerGenMonkeyPatch(ompl::base::StateSamplerPtr sampler);

void StateSamplerSetSeed(ompl::base::StateSamplerPtr sampler, uint32_t seed);

} // namespace ompl_lemur
