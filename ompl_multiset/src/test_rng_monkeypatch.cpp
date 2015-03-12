#include <cstdio>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl_multiset/SamplerGenMonkeyPatch.h>
#include <ompl_multiset/Roadmap.h>
#include <ompl_multiset/RoadmapSampledDensified.h>
#include <ompl_multiset/Cache.h>

#if 0
#  include <boost/random/lagged_fibonacci.hpp>
#  include <boost/random/uniform_int.hpp>
#endif

int main()
{
#if 0
   {
      boost::lagged_fibonacci607 sGen_(1);
      boost::uniform_int<> sDist_(1, 1000000000);
      boost::variate_generator<boost::lagged_fibonacci607&, boost::uniform_int<> > s_(sGen_, sDist_);
      printf("seed generated: %u\n", s_());
      printf("seed generated: %u\n", s_());
      printf("seed generated: %u\n", s_());
      printf("seed generated: %u\n", s_());
      printf("seed generated: %u\n", s_());
      return 0;
   }
#endif   
   
#if 1
   {
      ompl::base::StateSpacePtr space(new ompl::base::RealVectorStateSpace(7));
      ompl::base::RealVectorBounds bounds(7);
      bounds.setLow(0,  0.52359877559829881566); bounds.setHigh(0, 5.75958653158128708327);
      bounds.setLow(1, -1.97222205475359246840); bounds.setHigh(1, 1.97222205475359246840);
      bounds.setLow(2, -2.74016692563109742764); bounds.setHigh(2, 2.74016692563109742764);
      bounds.setLow(3, -0.87266462599716476678); bounds.setHigh(3, 3.14159265358979311600);
      bounds.setLow(4, -4.79965544298440605075); bounds.setHigh(4, 1.30899693899574720568);
      bounds.setLow(5, -1.57079632679489655800); bounds.setHigh(5, 1.57079632679489655800);
      bounds.setLow(6, -3.00196631343024700200); bounds.setHigh(6, 3.00196631343024700200);
      space->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);
      
      ompl::RNG::setSeed(1);
      ompl::base::StateSamplerPtr sampler(space->allocStateSampler());
      sampler = space->allocStateSampler();
      sampler = space->allocStateSampler();
      
      // do some sampling
      ompl::base::State * s_new = space->allocState();
      double * q;
      
      sampler->sampleUniform(s_new); 
      q = s_new->as<ompl::base::RealVectorStateSpace::StateType>()->values;
      printf("sampled state (random): %f %f %f %f %f %f %f\n",
         q[0], q[1], q[2], q[3], q[4], q[5], q[6]);
      
      return 0;
   }
#endif
   
#if 0
   {
      ompl::base::StateSpacePtr space(new ompl::base::RealVectorStateSpace(7));
      ompl::base::RealVectorBounds bounds(7);
      bounds.setLow(0,  0.52359877559829881566); bounds.setHigh(0, 5.75958653158128708327);
      bounds.setLow(1, -1.97222205475359246840); bounds.setHigh(1, 1.97222205475359246840);
      bounds.setLow(2, -2.74016692563109742764); bounds.setHigh(2, 2.74016692563109742764);
      bounds.setLow(3, -0.87266462599716476678); bounds.setHigh(3, 3.14159265358979311600);
      bounds.setLow(4, -4.79965544298440605075); bounds.setHigh(4, 1.30899693899574720568);
      bounds.setLow(5, -1.57079632679489655800); bounds.setHigh(5, 1.57079632679489655800);
      bounds.setLow(6, -3.00196631343024700200); bounds.setHigh(6, 3.00196631343024700200);
      space->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);
      
      ompl_multiset::RoadmapSampledDensified roadmap(space, 419884521, 1000, 0.5279455036166703);
      roadmap.subgraphs_generate(1);
      double * q = roadmap.vertices[0]->as<ompl::base::RealVectorStateSpace::StateType>()->values;
      printf("first sampled state (random): %f %f %f %f %f %f %f\n",
         q[0], q[1], q[2], q[3], q[4], q[5], q[6]);
      return 0;
   }
#endif
   
   
   
   
   
   
   
   printf("starting test_rng_monkeypatch ...\n");
   
   ompl::base::StateSpacePtr space(new ompl::base::RealVectorStateSpace(2));
   /* state space set bounds */
   {
      ompl::base::RealVectorBounds bounds(2);
      bounds.setLow(0, 0.0); bounds.setHigh(0, 1.0);
      bounds.setLow(1, 0.0); bounds.setHigh(1, 1.0);
      space->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);
   }
   ompl::base::StateSamplerPtr sampler(space->allocStateSampler());
   
   // do some sampling
   ompl::base::State * s_new = space->allocState();
   double * q;
   
   sampler->sampleUniform(s_new); 
   q = s_new->as<ompl::base::RealVectorStateSpace::StateType>()->values;
   printf("sampled state (random): %f %f\n", q[0], q[1]);
   
   // monkey patch sampler
   //ompl_multiset::SamplerRNGMonkeyPatch(sampler, 2);
   
   ompl_multiset::SamplerGenMonkeyPatch(sampler) = boost::mt19937(2);
   
   sampler->sampleUniform(s_new); 
   q = s_new->as<ompl::base::RealVectorStateSpace::StateType>()->values;
   printf("sampled state (should be 0.435995 0.185082): %f %f\n", q[0], q[1]);
   
   
#if 0
   ompl_multiset::RoadmapSampledDensified roadmap(space, 2, 10, 1.1);
   roadmap.subgraphs_generate(3);
   for (unsigned int gi=0; gi<3; gi++)
      printf("subgraph[%u] has %u verts and %u edges!\n", gi,
         roadmap.subgraphs[gi].first, roadmap.subgraphs[gi].second);
   
   q = roadmap.vertices[29]->as<ompl::base::RealVectorStateSpace::StateType>()->values;
   printf("vertex 29 is %f, %f\n", q[0], q[1]);
#endif

#if 1
   {
      ompl_multiset::RoadmapSampledDensified roadmap1(space, 2, 10, 1.1);
      roadmap1.subgraphs_generate(2);
      
      ompl_multiset::Cache * c = ompl_multiset::cache_create("mycache");
      c->roadmap_save(&roadmap1);
      delete c;
   }
   
   printf("everything destructed!\n");
   
   {
      ompl_multiset::RoadmapSampledDensified roadmap2(space, 2, 10, 1.1);
      
      ompl_multiset::Cache * c = ompl_multiset::cache_create("mycache");
      c->roadmap_load(&roadmap2);
      delete c;
      
      roadmap2.subgraphs_generate(3);
      q = roadmap2.vertices[29]->as<ompl::base::RealVectorStateSpace::StateType>()->values;
      printf("vertex 29 is %f, %f\n", q[0], q[1]);
   }
#endif
   
   

   
   
   
   
   return 0;
}
