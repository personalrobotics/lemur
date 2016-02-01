/* File: RoadmapRGGDens.h
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

/* requires:
#include <ompl_lemur/SamplerGenMonkeyPatch.h>
*/

namespace ompl_lemur
{

// for now this is an r-disk prm,
// uniform milestone sampling with given seed,
// uses the space's default sampler
template <class RoadmapArgs>
class RoadmapRGGDens : public Roadmap<RoadmapArgs>
{
   typedef boost::graph_traits<typename RoadmapArgs::Graph> GraphTypes;
   typedef typename GraphTypes::vertex_descriptor Vertex;
   typedef typename GraphTypes::edge_descriptor Edge;
   
   // set on construction
   unsigned int _dim;
   ompl::base::RealVectorBounds _bounds;
   
   // params
   unsigned int _num_per_batch;
   double _radius_first_batch;
   unsigned int _seed;
   
   // set on initialization
   double _gamma;
   ompl::base::StateSamplerPtr _sampler;
   
public:
   RoadmapRGGDens(RoadmapArgs & args):
      Roadmap<RoadmapArgs>(args, "RGGDens", 0),
      _dim(0),
      _bounds(0),
      _num_per_batch(0),
      _radius_first_batch(0.0),
      _seed(0),
      _sampler(this->space->allocStateSampler())
   {
      // check that we're in a real vector state space
      if (this->space->getType() != ompl::base::STATE_SPACE_REAL_VECTOR)
         throw std::runtime_error("NewRoadmapRGGDens only supports rel vector state spaces!");
      _dim = this->space->getDimension();
      if (0 == ompl_lemur::util::get_prime(_dim-1))
         throw std::runtime_error("not enough primes hardcoded!");
      ompl::base::StateSpacePtr myspace(this->space);
      _bounds = myspace->as<ompl::base::RealVectorStateSpace>()->getBounds();
      
      this->template declareParam<unsigned int>("num_per_batch", this,
         &RoadmapRGGDens<RoadmapArgs>::setNumPerBatch,
         &RoadmapRGGDens<RoadmapArgs>::getNumPerBatch);
      this->template declareParam<double>("radius_first_batch", this,
         &RoadmapRGGDens::setRadiusFirstBatch,
         &RoadmapRGGDens::getRadiusFirstBatch);
      this->template declareParam<unsigned int>("seed", this,
         &RoadmapRGGDens::setSeed,
         &RoadmapRGGDens::getSeed);
   }
   
   void setNumPerBatch(unsigned int num_per_batch)
   {
      if (num_per_batch == _num_per_batch)
         return;
      if (this->initialized)
         throw std::runtime_error("cannot set num_per_batch, already initialized!");
      _num_per_batch = num_per_batch;
   }
   
   unsigned int getNumPerBatch() const
   {
      return _num_per_batch;
   }
   
   void setRadiusFirstBatch(double radius_first_batch)
   {
      if (radius_first_batch == _radius_first_batch)
         return;
      if (this->initialized)
         throw std::runtime_error("cannot set radius_first_batch, already initialized!");
      _radius_first_batch = radius_first_batch;
   }
   
   double getRadiusFirstBatch() const
   {
      return _radius_first_batch;
   }
   
   void setSeed(unsigned int seed)
   {
      if (seed == _seed)
         return;
      if (this->initialized)
         throw std::runtime_error("cannot set seed, already initialized!");
      _seed = seed;
   }
   
   unsigned int getSeed() const
   {
      return _seed;
   }

   void initialize()
   {
      if (_num_per_batch == 0)
         throw std::runtime_error("cannot initialize, num not set!");
      if (_radius_first_batch == 0.0)
         throw std::runtime_error("cannot initialize, radius_first_batch not set!");
      
      _gamma = _radius_first_batch / pow(log(_num_per_batch)/_num_per_batch, 1./_dim);
      printf("RoadmapRGGDens calculated gamma=%f\n", _gamma);
      
      ompl_lemur::SamplerGenMonkeyPatch(_sampler) = boost::mt19937(_seed);
      
      this->initialized = true;
   }
   
   void deserialize(const std::string & ser_data)
   {
      std::stringstream ss(ser_data);
      ss >> ompl_lemur::SamplerGenMonkeyPatch(_sampler);
   }
   
   // should be stateless
   double root_radius(std::size_t i_batch)
   {
      unsigned int n = (1 + i_batch) * _num_per_batch;
      return _gamma * pow(log(n)/n, 1./_dim);
   }
   
   // sets all of these maps
   // generates one additional batch
   void generate()
   {
      // compute radius
      double radius = root_radius(this->num_batches_generated);
      std::size_t n = (this->num_batches_generated+1) * _num_per_batch;
      for (std::size_t v_index=num_vertices(this->g); v_index<n; v_index++)
      {
         Vertex v_new = add_vertex(this->g);
         
         put(this->vertex_batch_map, v_new, this->num_batches_generated);
         put(this->is_shadow_map, v_new, false);
         
         // allocate a new state for this vertex
         put(this->state_map, v_new, this->space->allocState());
         ompl::base::State * v_state = get(this->state_map, v_new);
         _sampler->sampleUniform(v_state);
         this->nn->add(v_new);
                  
         // allocate new undirected edges
         std::vector<Vertex> vs_near;
         this->nn->nearestR(v_new, radius, vs_near);
         for (unsigned int ui=0; ui<vs_near.size(); ui++)
         {
            if (vs_near[ui] == v_new)
               continue;
            Edge e = add_edge(v_new, vs_near[ui], this->g).first;
            ompl::base::State * vnear_state = get(this->state_map,vs_near[ui]);
            put(this->distance_map, e, this->space->distance(v_state,vnear_state));
            put(this->edge_batch_map, e, this->num_batches_generated);
         }
      }
      this->num_batches_generated++;
   }
   
   void serialize(std::string & ser_data)
   {
      std::stringstream ss;
      ss << ompl_lemur::SamplerGenMonkeyPatch(_sampler);
      ser_data = ss.str();
   }
};

} // namespace ompl_lemur
