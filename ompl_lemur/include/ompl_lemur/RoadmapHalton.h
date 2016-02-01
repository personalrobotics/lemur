/* File: RoadmapHalton.h
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

namespace ompl_lemur
{

template <class RoadmapArgs>
class RoadmapHalton : public Roadmap<RoadmapArgs>
{
   typedef boost::graph_traits<typename RoadmapArgs::Graph> GraphTypes;
   typedef typename GraphTypes::vertex_descriptor Vertex;
   typedef typename GraphTypes::edge_descriptor Edge;
   
   // set on construction
   unsigned int _dim;
   ompl::base::RealVectorBounds _bounds;
   
   // params
   unsigned int _num;
   double _radius;
   
public:
   RoadmapHalton(RoadmapArgs & args):
      Roadmap<RoadmapArgs>(args, "Halton", 1),
      _dim(0),
      _bounds(0),
      _num(0),
      _radius(0.0)
   {
      // check that we're in a real vector state space
      if (this->space->getType() != ompl::base::STATE_SPACE_REAL_VECTOR)
         throw std::runtime_error("RoadmapHalton only supports rel vector state spaces!");
      _dim = this->space->getDimension();
      if (0 == ompl_lemur::util::get_prime(_dim-1))
         throw std::runtime_error("not enough primes hardcoded!");
      ompl::base::StateSpacePtr myspace(this->space);
      _bounds = myspace->as<ompl::base::RealVectorStateSpace>()->getBounds();
      
      this->template declareParam<unsigned int>("num", this,
         &RoadmapHalton<RoadmapArgs>::setNum,
         &RoadmapHalton<RoadmapArgs>::getNum);
      this->template declareParam<double>("radius", this,
         &RoadmapHalton::setRadius,
         &RoadmapHalton::getRadius);
   }
   
   void setNum(unsigned int num)
   {
      if (num == _num)
         return;
      if (this->initialized)
         throw std::runtime_error("cannot set num, already initialized!");
      _num = num;
   }
   
   unsigned int getNum() const
   {
      return _num;
   }
   
   void setRadius(double radius)
   {
      if (radius == _radius)
         return;
      if (this->initialized)
         throw std::runtime_error("cannot set radius, already initialized!");
      _radius = radius;
   }
   
   double getRadius() const
   {
      return _radius;
   }

   void initialize()
   {
      if (_num == 0)
         throw std::runtime_error("cannot initialize, num not set!");
      if (_radius == 0.0)
         throw std::runtime_error("cannot initialize, radius not set!");
      this->initialized = true;
   }
   
   void deserialize(const std::string & ser_data)
   {
      throw std::runtime_error("RoadmapHalton deserialize from ser_data not supported!");
   }
   
   // should be stateless
   double root_radius(std::size_t i_batch)
   {
      return _radius;
   }
   
   // sets all of these maps
   // generates one additional batch
   void generate()
   {
      if (this->max_batches < this->num_batches_generated + 1)
         throw std::runtime_error("this roadmap gen doesnt support that many batches!");
      for (std::size_t v_index=num_vertices(this->g); v_index<_num; v_index++)
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
            values[ui] = _bounds.low[ui] + (_bounds.high[ui] - _bounds.low[ui])
               * ompl_lemur::util::halton(
                  ompl_lemur::util::get_prime(ui), v_index);
         }
         this->nn->add(v_new);
                  
         // allocate new undirected edges
         std::vector<Vertex> vs_near;
         this->nn->nearestR(v_new, _radius, vs_near);
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
      throw std::runtime_error("RoadmapHalton serialize to ser_data not supported!");
   }
};

} // namespace ompl_lemur
