/* File: RoadmapAAGrid.h
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

namespace ompl_lemur
{

template <class RoadmapArgs>
class RoadmapAAGrid : public Roadmap<RoadmapArgs>
{
   typedef boost::graph_traits<typename RoadmapArgs::Graph> GraphTypes;
   typedef typename GraphTypes::vertex_descriptor Vertex;
   typedef typename GraphTypes::edge_descriptor Edge;
   
   // set on construction
   unsigned int _dim;
   ompl::base::RealVectorBounds _bounds;
   
   // params
   double _res;
   
public:
   RoadmapAAGrid(RoadmapArgs & args):
      Roadmap<RoadmapArgs>(args, "AAGrid", 1),
      _dim(0),
      _bounds(0),
      _res(0.0)
   {
      // check that we're in a real vector state space
      if (this->space->getType() != ompl::base::STATE_SPACE_REAL_VECTOR)
         throw std::runtime_error("RoadmapAAGrid only supports rel vector state spaces!");
      _dim = this->space->getDimension();
      ompl::base::StateSpacePtr myspace(this->space);
      _bounds = myspace->as<ompl::base::RealVectorStateSpace>()->getBounds();
      
      this->template declareParam<double>("res", this,
         &RoadmapAAGrid::setRes,
         &RoadmapAAGrid::getRes);
   }

   void setRes(double res)
   {
      if (res == _res)
         return;
      if (this->initialized)
         throw std::runtime_error("cannot set res, already initialized!");
      _res = res;
   }
   
   double getRes() const
   {
      return _res;
   }
   
   void initialize()
   {
      if (_res == 0.0)
         throw std::runtime_error("cannot initialize, res not set!");
      this->initialized = true;
   }
   
   void deserialize(const std::string & ser_data)
   {
      throw std::runtime_error("RoadmapAAGrid deserialize from ser_data not supported!");
   }
   
   // should be stateless
   double root_radius(std::size_t i_batch)
   {
      return _res;
   }
   
   // sets all of these maps
   // generates one additional batch
   void generate()
   {
      if (this->max_batches < this->num_batches_generated + 1)
         throw std::runtime_error("this roadmap gen doesnt support that many batches!");
      // ok, generate all nodes!
      // first, compute offset and number of vertices per dimension
      std::vector<double> dim_offsets;
      std::vector<std::size_t> dim_numverts;
      std::size_t total_numverts = 1;
      for (std::size_t idim=0; idim<_dim; idim++)
      {
         double dim_len = _bounds.high[idim] - _bounds.low[idim];
         std::size_t numverts = floor(0.5 + dim_len/_res);
         double offset = 0.5*(dim_len - (numverts-1)*_res);
         total_numverts *= numverts;
         dim_numverts.push_back(numverts);
         dim_offsets.push_back(offset);
         printf("dim:%lu numerts:%lu offset:%f\n",
            idim, numverts, offset);
      }
      // ok, add vertices, and keep all descriptors for now ...
      std::vector<Vertex> vertices(total_numverts);
      for (std::size_t ivert=0; ivert<total_numverts; ivert++)
      {
         Vertex v_new = add_vertex(this->g);
         vertices[ivert] = v_new;
         
         put(this->vertex_batch_map, v_new, 0);
         put(this->is_shadow_map, v_new, false);
         
         // allocate a new state for this vertex
         put(this->state_map, v_new, this->space->allocState());
         ompl::base::State * v_state = get(this->state_map, v_new);
         double * values = v_state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
         this->nn->add(v_new);
         
         std::size_t ivert_used = ivert;
         std::size_t dim_stride = 1;
         for (std::size_t idim=_dim-1; ; idim--) // termination condition at end!
         {
            // set state components
            // vert[x][y][z] = vert[x*(ny*nz)+y*(nz)+z]
            std::size_t idimvert = ivert_used % dim_numverts[idim];
            values[idim] = _bounds.low[idim] + dim_offsets[idim] + _res*idimvert;
            // add edge to previous vertex along this dim
            if (idimvert)
            {
               std::size_t ivert_dimprev = ivert - dim_stride;
               Edge e = add_edge(vertices[ivert_dimprev], v_new, this->g).first;
               put(this->distance_map, e, _res);
               put(this->edge_batch_map, e, 0);
            }
            // continue
            if (idim == 0) break;
            ivert_used /= dim_numverts[idim];
            dim_stride *= dim_numverts[idim];
         }
      }
      this->num_batches_generated++;
   }
   
   void serialize(std::string & ser_data)
   {
      throw std::runtime_error("RoadmapAAGrid serialize to ser_data not supported!");
   }
};

} // namespace ompl_lemur
