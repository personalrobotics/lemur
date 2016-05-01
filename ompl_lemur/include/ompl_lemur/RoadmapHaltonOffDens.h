/*! \file RoadmapHaltonOffDens.h
 * \author Chris Dellin <cdellin@gmail.com>
 * \copyright 2015 Carnegie Mellon University
 * \copyright License: BSD
 */

/* requires:
#include <ompl_lemur/SamplerGenMonkeyPatch.h>
*/

namespace ompl_lemur
{

template <class RoadmapArgs>
class RoadmapHaltonOffDens : public Roadmap<RoadmapArgs>
{
   typedef boost::graph_traits<typename RoadmapArgs::Graph> GraphTypes;
   typedef typename GraphTypes::vertex_descriptor Vertex;
   typedef typename GraphTypes::edge_descriptor Edge;
   
   // set on construction
   unsigned int _dim;
   ompl::base::RealVectorBounds _bounds;
   
   // params
   unsigned int _num_per_batch;
   double _gamma_factor;
   enum t_scaling
   {
      SCALING_NOTSET,
      SCALING_LOG_N,
      SCALING_LOGLOG_N,
      SCALING_1_N
   } _scaling;
   unsigned int _seed;
   bool _seed_set;
   
   // set on initialization
   double _gamma;
   ompl::base::ScopedState<ompl::base::RealVectorStateSpace> _offset_state;
   double * _offset_values;
   
public:
   RoadmapHaltonOffDens(RoadmapArgs & args):
      Roadmap<RoadmapArgs>(args, "HaltonOffDens", 0),
      _dim(0),
      _bounds(0),
      _num_per_batch(0),
      _gamma_factor(0.0),
      _scaling(SCALING_NOTSET),
      _seed(0),
      _seed_set(false),
      _offset_state(this->space)
   {
      // check that we're in a real vector state space
      if (this->space->getType() != ompl::base::STATE_SPACE_REAL_VECTOR)
         throw std::runtime_error("RoadmapHaltonOffDens only supports rel vector state spaces!");
      _dim = this->space->getDimension();
      if (0 == ompl_lemur::util::get_prime(_dim-1))
         throw std::runtime_error("not enough primes hardcoded!");
      ompl::base::StateSpacePtr myspace(this->space);
      _bounds = myspace->as<ompl::base::RealVectorStateSpace>()->getBounds();
      _offset_values = _offset_state.get()->as<ompl::base::RealVectorStateSpace::StateType>()->values;
      
      this->template declareParam<unsigned int>("num_per_batch", this,
         &RoadmapHaltonOffDens<RoadmapArgs>::setNumPerBatch,
         &RoadmapHaltonOffDens<RoadmapArgs>::getNumPerBatch);
      this->template declareParam<double>("gamma_factor", this,
         &RoadmapHaltonOffDens::setGammaFactor,
         &RoadmapHaltonOffDens::getGammaFactor);
      this->template declareParam<std::string>("scaling", this,
         &RoadmapHaltonOffDens::setScaling,
         &RoadmapHaltonOffDens::getScaling);
      this->template declareParam<unsigned int>("seed", this,
         &RoadmapHaltonOffDens::setSeed,
         &RoadmapHaltonOffDens::getSeed);
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
   
   void setGammaFactor(double gamma_factor)
   {
      if (gamma_factor == _gamma_factor)
         return;
      if (this->initialized)
         throw std::runtime_error("cannot set gamma_factor, already initialized!");
      _gamma_factor = gamma_factor;
   }
   
   double getGammaFactor() const
   {
      return _gamma_factor;
   }
   
   void setScaling(std::string str_scaling)
   {
      enum t_scaling scaling;
      if (str_scaling == "log_n")
         scaling=SCALING_LOG_N;
      else if (str_scaling == "loglog_n")
         scaling=SCALING_LOGLOG_N;
      else if (str_scaling == "1_n")
         scaling=SCALING_1_N;
      else
         throw std::runtime_error("cannot set scaling, unknown value!");
      if (scaling == _scaling)
         return;
      if (this->initialized)
         throw std::runtime_error("cannot set scaling, already initialized!");
      _scaling = scaling;
   }
   
   std::string getScaling() const
   {
      switch (_scaling)
      {
      case SCALING_LOG_N: return "log_n";
      case SCALING_LOGLOG_N: return "loglog_n";
      case SCALING_1_N: return "1_n";
      default: return "notset";
      }
   }
     
   
   void setSeed(unsigned int seed)
   {
      if (_seed_set && seed == _seed)
         return;
      if (this->initialized)
         throw std::runtime_error("cannot set seed, already initialized!");
      _seed = seed;
      _seed_set = true;
   }
   
   unsigned int getSeed() const
   {
      return _seed;
   }
   
   void initialize()
   {
      std::vector<std::string> missings;
      if (_num_per_batch == 0)
         missings.push_back("num_per_batch");
      if (_gamma_factor == 0.0)
         missings.push_back("gamma_factor");
      if (_scaling == SCALING_NOTSET)
         missings.push_back("scaling");
      if (!_seed_set)
         missings.push_back("seed");
      if (missings.size())
      {
         std::string str = "Cannot initialize, parameters not set:";
         for (unsigned int ui=0; ui<missings.size(); ui++)
            str += " " + missings[ui];
         throw std::runtime_error(str);
      }
      
      double frac = this->space->getMeasure() / ompl_lemur::util::volume_n_ball(_dim);
      _gamma = _gamma_factor * 2.0 * pow((1.+1./_dim) * frac, 1./_dim);
      
      ompl::base::StateSamplerPtr sampler(this->space->allocStateSampler());
      ompl_lemur::SamplerGenMonkeyPatch(sampler) = boost::mt19937(_seed);
      sampler->sampleUniform(_offset_state.get());
      
      this->initialized = true;
   }
   
   // nothing to deserialize!
   void deserialize(const std::string & ser_data)
   {
   }
   
   // should be stateless
   double root_radius(std::size_t i_batch)
   {
      std::size_t n = (i_batch+1) * _num_per_batch;
      switch (_scaling)
      {
      case SCALING_LOG_N:
         return _gamma * pow(log(n)/(1.*n), 1./_dim);
      case SCALING_LOGLOG_N:
         return _gamma * pow(log(log(n))/(1.*n), 1./_dim);
      case SCALING_1_N:
         return _gamma * pow(1./(1.*n), 1./_dim);
      default:
         break;
      }
      throw std::runtime_error("scaling not set!");
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
         double * values = v_state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
         for (unsigned int ui=0; ui<_dim; ui++)
         {
            double value = _offset_values[ui];
            value += (_bounds.high[ui] - _bounds.low[ui])
               * ompl_lemur::util::halton(ompl_lemur::util::get_prime(ui), v_index);
            if (_bounds.high[ui] < value)
               value -= (_bounds.high[ui] - _bounds.low[ui]);
            values[ui] = value;
         }
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
   
   // nothing to serialize (offset from seed, and num_generated known)
   void serialize(std::string & ser_data)
   {
   }
};

} // namespace ompl_lemur
