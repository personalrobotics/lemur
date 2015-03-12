#include <cstdarg>
#include <cfloat>
#include <vector>
#include <ompl/base/StateSampler.h>
#include <ompl/base/StateSpace.h>
#include <ompl_multiset/SamplerGenMonkeyPatch.h>
#include <ompl_multiset/Roadmap.h>
#include <ompl_multiset/RoadmapSampledDensified.h>

namespace
{

std::string sf(const char * fmt, ...)
{
  va_list ap;
  va_start(ap, fmt);
  int size = vsnprintf(0, 0, fmt, ap);
  va_end(ap);
  char * buf = new char[size+1];
  va_start(ap, fmt);
  vsnprintf(buf, size+1, fmt, ap);
  va_end(ap);
  std::string ret = std::string(buf);
  delete[] buf;
  return ret;
}
   
// volume of an n-ball
// https://en.wikipedia.org/wiki/Volume_of_an_n-ball
double volume_n_ball(unsigned int n)
{
   unsigned int k = n / 2;
   if (n % 2 == 0)
   {
      // even
      double ret = pow(M_PI,k);
      for (unsigned int i=2; i<=k; i++)
         ret /= i;
      return ret;
   }
   else
   {
      // odd
      double ret = 2. * pow(4.*M_PI, k);
      for (unsigned int i=k+1; i<=n; i++)
         ret /= i;
      return ret;
   }
}

std::string double_to_text(double in)
{
   char buf[2048];
   int min = 0;
   int max = 2047;
   // invariant: min DOESNT WORK, max DOES WORK
   // validate invariants
   sprintf(buf, "%.*f", min, in);
   if (in == strtod(buf,0))
      return std::string(buf);
   sprintf(buf, "%.*f", max, in);
   if (in != strtod(buf,0))
      return std::string(buf);
   // binary search
   for (;;)
   {
      int diff = max - min;
      if (diff == 1)
         break;
      int test = min + diff/2;
      sprintf(buf, "%.*f", test, in);
      if (in == strtod(buf,0))
         max = test;
      else
         min = test;
   }
   sprintf(buf, "%.*f", max, in);
   return std::string(buf);
}
   
} // anonymous namespace

ompl_multiset::RoadmapSampledDensified::RoadmapSampledDensified(
      const ompl::base::StateSpacePtr space,
      unsigned int seed,
      unsigned int batch_n,
      double gamma_rel): // > 1!
   Roadmap(space),
   sampler(space->allocStateSampler()),
   batch_n(batch_n)
{
   ompl_multiset::SamplerGenMonkeyPatch(sampler) = boost::mt19937(seed);
   this->d = this->space->getDimension();
   double pow_inside = (1.0 + 1.0/this->d)
      * this->space->getMeasure()/volume_n_ball(this->d);
   this->gamma = gamma_rel * 2.0 * pow(pow_inside, 1.0/this->d);
   this->id = sf("class=RoadmapSampledDensified seed=%u batch_n=%u gamma_rel=%s",
      seed, batch_n, double_to_text(gamma_rel).c_str());
}

std::string ompl_multiset::RoadmapSampledDensified::get_id()
{
   return this->id;
}


ompl_multiset::RoadmapSampledDensified::~RoadmapSampledDensified()
{
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
      
      this->subgraphs.push_back(std::make_pair(
         this->vertices.size(),
         this->edges.size()));
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






#if 0

void ompl_multiset::SampledDensifiedRoadmap::read(FILE * fp)
{
   
}

void ompl_multiset::SampledDensifiedRoadmap::write(FILE * fp)
{
   unsigned int serlen = this->space->getSerializationLength();
   
   unsigned char * ser = (unsigned char *) malloc(serlen);
   
   // write vertices
   for (unsigned int vi=0; vi<this->vertices.size(); vi++)
   {
      Vertex & v = this->vertices[vi];
      this->space->serialize(ser, v.state);
      fprintf(fp, "vertex %u ", vi);
      for (unsigned int i=0; i<serlen; i++)
         fprintf(fp, "%02x", ser[i]);
      fprintf(fp, "\n");
   }
   
   free(ser);
   
   // write edges
   for (unsigned int vi=0; vi<this->vertices.size(); vi++)
   {
      Vertex & v = this->vertices[vi];
      fprintf(fp, "edges %u", vi);
      for (unsigned int ei=0; ei<v.edges.size(); ei++)
         fprintf(fp, " %u", v.edges[ei]);
      fprintf(fp, "\n");
   }
}

void ompl_multiset::SampledDensifiedRoadmap::vertex(
   unsigned int vi, ompl::base::State * s)
{
   while (this->vertices.size() <= vi)
      this->add_batch();
   
   if (!s)
      return;
   Vertex & v = this->vertices[vi];
   this->space->copyState(s, v.state);
}

void ompl_multiset::SampledDensifiedRoadmap::edges_from(
   unsigned int vi_source,
   unsigned int vi_to,
   std::vector<unsigned int>::iterator * v_begin,
   std::vector<unsigned int>::iterator * v_end)
{
   while (this->vertices.size() <= vi_source)
      this->add_batch();
   while (this->vertices.size() <= vi_to)
      this->add_batch();
   
   Vertex & v = this->vertices[vi_source];
   
   *v_begin = v.edges.begin();
   *v_end = v.edges.end();
}

void ompl_multiset::SampledDensifiedRoadmap::add_batch()
{
   // compute r
   unsigned int new_n = this->n + this->batch_n;
   double radius = this->gamma * pow(log(new_n)/new_n, 1.0/this->d);
   printf("using radius: %.30f\n", radius);
   
   while (this->vertices.size() < new_n)
   {
      unsigned int ia = this->vertices.size();
      printf("adding vertex %u ...\n", ia);
      
      // extend vertices array
      this->vertices.resize(ia + 1);
      Vertex & va = this->vertices[ia];
      
      // sample a new state
      va.state = this->space->allocState();
      this->sampler->sampleUniform(va.state);
      
      // make edges
      for (unsigned int ib=0; ib<ia; ib++)
      {
         Vertex & vb = this->vertices[ib];
         
         double dist = this->space->distance(va.state, vb.state);
         if (radius < dist)
            continue;
         
         va.edges.push_back(ib);
         vb.edges.push_back(ia);
      }
   }
   
   this->n = new_n;
}

#endif
