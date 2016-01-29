/* File: SamplerGenMonkeyPatch.cpp
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

#include <sstream>
#include <ompl/base/StateSampler.h>
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
      typedef boost::mt19937 ompl::RNG::*type;
      friend type get(RNG_generator);
   };
   template struct Rob<RNG_generator, &ompl::RNG::generator_>;
   
} // anonymous namespace

boost::mt19937 & ompl_lemur::SamplerGenMonkeyPatch(
   ompl::base::StateSamplerPtr sampler)
{
   return (*sampler).*get(StateSampler_rng()).*get(RNG_generator());
}
