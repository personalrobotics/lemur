/* File: RoadmapAAGrid.h
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

namespace ompl_multiset
{

template <class RoadmapSpec>
class RoadmapAAGrid : public RoadmapSpec
{
   typedef typename RoadmapSpec::BaseGraph Graph;
   typedef typename RoadmapSpec::BaseVState VState;
   typedef typename RoadmapSpec::BaseEDistance EDistance;
   typedef typename RoadmapSpec::BaseVBatch VBatch;
   typedef typename RoadmapSpec::BaseEBatch EBatch;
   typedef typename RoadmapSpec::BaseVShadow VShadow;
   typedef typename RoadmapSpec::BaseNN NN;

   typedef boost::graph_traits<Graph> GraphTypes;
   typedef typename GraphTypes::vertex_descriptor Vertex;
   typedef typename GraphTypes::edge_descriptor Edge;
   
public:
   // input parameters
   const double res;
   
   RoadmapAAGrid(
      const ompl::base::StateSpacePtr space,
      double res):
      RoadmapSpec(space,1),
      res(res),
      dim(0),
      bounds(0),
      num_batches_generated(0),
      vertices_generated(0),
      edges_generated(0)
   {
      // check that we're in a real vector state space
      if (space->getType() != ompl::base::STATE_SPACE_REAL_VECTOR)
         throw std::runtime_error("RoadmapAAGrid only supports rel vector state spaces!");
      dim = space->getDimension();
      bounds = space->as<ompl::base::RealVectorStateSpace>()->getBounds();
      // check for infinite (no) bounds?
   }
   ~RoadmapAAGrid() {}
   
   std::size_t get_num_batches_generated()
   {
      return num_batches_generated;
   }
   
   double root_radius(std::size_t i_batch)
   {
      return res;
   }
   
   void generate(
      Graph & g,
      NN & nn,
      VState state_map,
      EDistance distance_map,
      VBatch vertex_batch_map,
      EBatch edge_batch_map,
      VShadow is_shadow_map)
   {
      if (this->max_batches < num_batches_generated + 1)
         throw std::runtime_error("this roadmap gen doesnt support that many batches!");
      // ok, generate all nodes!
      // first, compute offset and number of vertices per dimension
      std::vector<double> dim_offsets;
      std::vector<std::size_t> dim_numverts;
      std::size_t total_numverts = 1;
      for (std::size_t idim=0; idim<dim; idim++)
      {
         double dim_len = bounds.high[idim] - bounds.low[idim];
         std::size_t numverts = floor(0.5 + dim_len/res);
         double offset = 0.5*(dim_len - (numverts-1)*res);
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
         Vertex v_new = add_vertex(g);
         vertices[ivert] = v_new;
         
         put(vertex_batch_map, v_new, 0);
         put(is_shadow_map, v_new, false);
         
         // allocate a new state for this vertex
         put(state_map, v_new, this->space->allocState());
         ompl::base::State * v_state = get(state_map, v_new);
         double * values = v_state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
         std::size_t ivert_used = ivert;
         std::size_t dim_stride = 1;
         for (std::size_t idim=dim-1; ; idim--) // termination condition at end!
         {
            // set state components
            // vert[x][y][z] = vert[x*(ny*nz)+y*(nz)+z]
            std::size_t idimvert = ivert_used % dim_numverts[idim];
            values[idim] = bounds.low[idim] + dim_offsets[idim] + res*idimvert;
            // add edge to previous vertex along this dim
            if (idimvert)
            {
               std::size_t ivert_dimprev = ivert - dim_stride;
               Edge e = add_edge(vertices[ivert_dimprev], v_new, g).first;
               put(distance_map, e, res);
               put(edge_batch_map, e, 0);
            }
            // continue
            if (idim == 0) break;
            ivert_used /= dim_numverts[idim];
            dim_stride *= dim_numverts[idim];
         }
      }
      num_batches_generated++;
   }
   
   void serialize()
   {
   }
   
   void deserialize()
   {
   }
   
private:
   // from space
   std::size_t dim;
   ompl::base::RealVectorBounds bounds;
   // progress
   std::size_t num_batches_generated;
   std::size_t vertices_generated;
   std::size_t edges_generated;
};

} // namespace ompl_multiset
