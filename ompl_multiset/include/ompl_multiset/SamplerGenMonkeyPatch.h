/* requires:
#include <ompl/base/StateSampler.h>
 */

namespace ompl_multiset
{

boost::mt19937 & SamplerGenMonkeyPatch(ompl::base::StateSamplerPtr sampler);

} // namespace ompl_multiset
