/* File: SamplerGenMonkeyPatch.h
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

/* requires:
#include <ompl/base/StateSampler.h>
 */

namespace ompl_lemur
{

boost::mt19937 & SamplerGenMonkeyPatch(ompl::base::StateSamplerPtr sampler);

} // namespace ompl_lemur
