/*! \file SamplerGenMonkeyPatch.cpp
 * \author Chris Dellin <cdellin@gmail.com>
 * \copyright 2015 Carnegie Mellon University
 * \copyright License: BSD
 */

#include <sstream>
#include <ompl/base/StateSampler.h>
#include <ompl_lemur/config.h>
#include <ompl_lemur/SamplerGenMonkeyPatch.h>

// from Johannes Schaub - litb
// http://bloglitb.blogspot.de/2011/12/access-to-private-members-safer.html

namespace {
   
   // this defines a friend function
   // that can be called by ADL using the tag type
   template<typename Tag, typename Tag::type M>
   struct Rob
   { 
      friend typename Tag::type get(Tag)
      {
         return M;
      }
   };

   // tag used to access StateSampler::rng_
   struct StateSampler_rng
   { 
      typedef ompl::RNG ompl::base::StateSampler::*type;
      friend type get(StateSampler_rng);
   };
   template struct Rob<StateSampler_rng, &ompl::base::StateSampler::rng_>;

   // tag used to access RNG::generator_
   struct RNG_generator
   {
#ifdef OMPL_LEMUR_HAS_BOOSTSMARTPTRS
      typedef boost::mt19937 ompl::RNG::*type;
#else
      typedef std::mt19937 ompl::RNG::*type;
#endif
      friend type get(RNG_generator);
   };
   template struct Rob<RNG_generator, &ompl::RNG::generator_>;
   
} // anonymous namespace

#ifdef OMPL_LEMUR_HAS_BOOSTSMARTPTRS
boost::mt19937 &
#else
std::mt19937 &
#endif
ompl_lemur::SamplerGenMonkeyPatch(ompl::base::StateSamplerPtr sampler)
{
   return (*sampler).*get(StateSampler_rng()).*get(RNG_generator());
}

void ompl_lemur::StateSamplerSetSeed(ompl::base::StateSamplerPtr sampler, uint32_t seed)
{
#ifdef OMPL_LEMUR_HAS_BOOSTSMARTPTRS
   SamplerGenMonkeyPatch(sampler) = boost::mt19937(seed);
#else
   SamplerGenMonkeyPatch(sampler) = std::mt19937(seed);
#endif
}
