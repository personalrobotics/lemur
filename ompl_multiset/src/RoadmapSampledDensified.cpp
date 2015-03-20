/* File: RoadmapSampledDensified.cpp
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: None
 */

#include <vector>
#include <ompl/base/StateSampler.h>
#include <ompl/base/StateSpace.h>
#include <ompl_multiset/SamplerGenMonkeyPatch.h>
#include <ompl_multiset/Roadmap.h>
#include <ompl_multiset/RoadmapSampledDensified.h>
#include <ompl_multiset/util.h>

using namespace ompl_multiset::util; // for sf, volume_n_ball, double_to_text

ompl_multiset::RoadmapSampledDensified::RoadmapSampledDensified(
      const ompl::base::StateSpacePtr space,
      unsigned int seed,
      unsigned int batch_n,
      double gamma_rel):
   Roadmap(space),
   sampler(space->allocStateSampler()),
   batch_n(batch_n)
{
   this->id = sf("class=RoadmapSampledDensified seed=%u batch_n=%u gamma_rel=%s",
      seed, batch_n, double_to_text(gamma_rel).c_str());
   ompl_multiset::SamplerGenMonkeyPatch(sampler) = boost::mt19937(seed);
   this->d = this->space->getDimension();
   double pow_inside = (1.0 + 1.0/this->d)
      * this->space->getMeasure()/volume_n_ball(this->d);
   this->gamma = gamma_rel * 2.0 * pow(pow_inside, 1.0/this->d);
}

ompl_multiset::RoadmapSampledDensified::~RoadmapSampledDensified()
{
}

std::string ompl_multiset::RoadmapSampledDensified::get_id()
{
   return this->id;
}

void ompl_multiset::RoadmapSampledDensified::subgraphs_limit(
   unsigned int * num_subgraphs)
{
   *num_subgraphs = 0; // no limit
}

void ompl_multiset::RoadmapSampledDensified::subgraphs_generate(
   unsigned int num_subgraphs)
{
   while (this->subgraphs.size() < num_subgraphs)
   {
      // add a new denser subgraph
      // with batch_n new vertices!
      
      // compute radius
      unsigned int new_n = this->vertices.size() + this->batch_n;
      double radius = this->gamma * pow(log(new_n)/new_n, 1.0/this->d);
      printf("using radius=%.30f\n", radius);
      
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
            if (radius < dist)
               continue;
            
            this->edges.push_back(std::make_pair(i,inew));
         }
      }
      
      this->subgraphs.push_back(SubGraph(
         this->vertices.size(),
         this->edges.size(),
         radius));
   }
}

void ompl_multiset::RoadmapSampledDensified::generator_save(
   std::string & data)
{
   std::stringstream ss;
   ss << ompl_multiset::SamplerGenMonkeyPatch(sampler);
   data = ss.str();
}

void ompl_multiset::RoadmapSampledDensified::generator_load(
   std::string & data)
{
   std::stringstream ss(data);
   ss >> ompl_multiset::SamplerGenMonkeyPatch(sampler);
}
