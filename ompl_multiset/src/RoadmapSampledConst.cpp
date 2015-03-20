/* File: RoadmapSampledConst.cpp
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: None
 */

#include <vector>
#include <ompl/base/StateSampler.h>
#include <ompl/base/StateSpace.h>
#include <ompl_multiset/SamplerGenMonkeyPatch.h>
#include <ompl_multiset/Roadmap.h>
#include <ompl_multiset/RoadmapSampledConst.h>
#include <ompl_multiset/util.h>

using namespace ompl_multiset::util; // for sf, volume_n_ball

ompl_multiset::RoadmapSampledConst::RoadmapSampledConst(
      const ompl::base::StateSpacePtr space,
      unsigned int seed,
      unsigned int batch_n,
      double radius):
   Roadmap(space),
   sampler(space->allocStateSampler()),
   batch_n(batch_n),
   radius(radius)
{
   this->id = sf("class=RoadmapSampledConst seed=%u batch_n=%u radius=%s",
      seed, batch_n, double_to_text(radius).c_str());
   ompl_multiset::SamplerGenMonkeyPatch(sampler) = boost::mt19937(seed);
}

ompl_multiset::RoadmapSampledConst::~RoadmapSampledConst()
{
}

std::string ompl_multiset::RoadmapSampledConst::get_id()
{
   return this->id;
}

void ompl_multiset::RoadmapSampledConst::subgraphs_limit(
   unsigned int * num_subgraphs)
{
   *num_subgraphs = 0; // no limit
}

void ompl_multiset::RoadmapSampledConst::subgraphs_generate(
   unsigned int num_subgraphs)
{
   while (this->subgraphs.size() < num_subgraphs)
   {
      // add a new denser subgraph
      // with batch_n new vertices!
      
      unsigned int new_n = this->vertices.size() + this->batch_n;
      while (this->vertices.size() < new_n)
      {
         unsigned int inew = this->vertices.size();
         //printf("adding vertex %u ...\n", inew);
         
         // sample a new vertex
         ompl::base::State * snew = this->space->allocState();
         this->sampler->sampleUniform(snew);
         this->vertices.push_back(snew);
         
         // make edges to all other vertices
         for (unsigned int i=0; i<inew; i++)
         {
            double dist = this->space->distance(snew, this->vertices[i]);
            if (this->radius < dist)
               continue;
            
            this->edges.push_back(std::make_pair(i,inew));
         }
      }
      
      this->subgraphs.push_back(SubGraph(
         this->vertices.size(),
         this->edges.size(),
         this->radius));
   }
}

void ompl_multiset::RoadmapSampledConst::generator_save(
   std::string & data)
{
   std::stringstream ss;
   ss << ompl_multiset::SamplerGenMonkeyPatch(sampler);
   data = ss.str();
}

void ompl_multiset::RoadmapSampledConst::generator_load(
   std::string & data)
{
   std::stringstream ss(data);
   ss >> ompl_multiset::SamplerGenMonkeyPatch(sampler);
}
