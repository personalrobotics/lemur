/* File: RoadmapSampledConst.cpp
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

#include <vector>
#include <ompl/base/StateSampler.h>
#include <ompl/base/StateSpace.h>
#include <ompl_multiset/SamplerGenMonkeyPatch.h>
#include <ompl_multiset/MultiSetRoadmap.h>
#include <ompl_multiset/MultiSetRoadmapSampledConst.h>
#include <ompl_multiset/util.h>

using namespace ompl_multiset::util; // for sf, volume_n_ball

ompl_multiset::MultiSetRoadmapSampledConst::MultiSetRoadmapSampledConst(
      const ompl::base::StateSpacePtr space,
      unsigned int seed,
      unsigned int batch_n,
      double radius):
   MultiSetRoadmap(space),
   sampler(space->allocStateSampler()),
   batch_n(batch_n),
   radius(radius)
{
   this->id = sf("class=MultiSetRoadmapSampledConst seed=%u batch_n=%u radius=%s",
      seed, batch_n, double_to_text(radius).c_str());
   ompl_multiset::SamplerGenMonkeyPatch(sampler) = boost::mt19937(seed);
}

ompl_multiset::MultiSetRoadmapSampledConst::~MultiSetRoadmapSampledConst()
{
}

std::string ompl_multiset::MultiSetRoadmapSampledConst::get_id()
{
   return this->id;
}

void ompl_multiset::MultiSetRoadmapSampledConst::subgraphs_limit(
   unsigned int * num_subgraphs)
{
   *num_subgraphs = 0; // no limit
}

void ompl_multiset::MultiSetRoadmapSampledConst::subgraphs_generate(
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

void ompl_multiset::MultiSetRoadmapSampledConst::generator_save(
   std::string & data)
{
   std::stringstream ss;
   ss << ompl_multiset::SamplerGenMonkeyPatch(sampler);
   data = ss.str();
}

void ompl_multiset::MultiSetRoadmapSampledConst::generator_load(
   std::string & data)
{
   std::stringstream ss(data);
   ss >> ompl_multiset::SamplerGenMonkeyPatch(sampler);
}
