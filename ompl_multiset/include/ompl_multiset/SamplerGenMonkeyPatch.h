/* File: SamplerGenMonkeyPatch.h
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: None
 */

/* requires:
#include <ompl/base/StateSampler.h>
 */

namespace ompl_multiset
{

boost::mt19937 & SamplerGenMonkeyPatch(ompl::base::StateSamplerPtr sampler);

} // namespace ompl_multiset
