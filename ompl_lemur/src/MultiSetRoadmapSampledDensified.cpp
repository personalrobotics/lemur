/* File: RoadmapSampledDensified.cpp
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

#include <vector>
#include <ompl/base/StateSampler.h>
#include <ompl/base/StateSpace.h>
#include <ompl_lemur/SamplerGenMonkeyPatch.h>
#include <ompl_lemur/MultiSetRoadmap.h>
#include <ompl_lemur/MultiSetRoadmapSampledDensified.h>
#include <ompl_lemur/util.h>

using namespace ompl_lemur::util; // for sf, volume_n_ball, double_to_text

ompl_lemur::MultiSetRoadmapSampledDensified::MultiSetRoadmapSampledDensified(
      const ompl::base::StateSpacePtr space,
      unsigned int seed,
      unsigned int batch_n,
      double gamma_rel):
   MultiSetRoadmap(space),
   sampler(space->allocStateSampler()),
   batch_n(batch_n)
{
   this->id = sf("class=MultiSetRoadmapSampledDensified seed=%u batch_n=%u gamma_rel=%s",
      seed, batch_n, double_to_text(gamma_rel).c_str());
   ompl_lemur::SamplerGenMonkeyPatch(sampler) = boost::mt19937(seed);
   this->d = this->space->getDimension();
   double pow_inside = (1.0 + 1.0/this->d)
      * this->space->getMeasure()/volume_n_ball(this->d);
   this->gamma = gamma_rel * 2.0 * pow(pow_inside, 1.0/this->d);
}

ompl_lemur::MultiSetRoadmapSampledDensified::~MultiSetRoadmapSampledDensified()
{
}

std::string ompl_lemur::MultiSetRoadmapSampledDensified::get_id()
{
   return this->id;
}

void ompl_lemur::MultiSetRoadmapSampledDensified::subgraphs_limit(
   unsigned int * num_subgraphs)
{
   *num_subgraphs = 0; // no limit
}

void ompl_lemur::MultiSetRoadmapSampledDensified::subgraphs_generate(
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

void ompl_lemur::MultiSetRoadmapSampledDensified::generator_save(
   std::string & data)
{
   std::stringstream ss;
   ss << ompl_lemur::SamplerGenMonkeyPatch(sampler);
   data = ss.str();
}

void ompl_lemur::MultiSetRoadmapSampledDensified::generator_load(
   std::string & data)
{
   std::stringstream ss(data);
   ss >> ompl_lemur::SamplerGenMonkeyPatch(sampler);
}
