/* File: RoadmapGenAAGrid.h
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

namespace ompl_multiset
{

template <class TypeSet>
class RoadmapGenAAGrid : public RoadmapGen<TypeSet>
{
   typedef boost::graph_traits<typename TypeSet::Graph> GraphTypes;
   typedef typename GraphTypes::vertex_descriptor Vertex;
   
public:
   RoadmapGenAAGrid(
      const ompl::base::StateSpacePtr space,
      const std::string args):
      RoadmapGen<TypeSet>(space,"RoadmapGenAAGrid",args,1),
      dim(0),
      bounds(0),
      num_subgraphs_generated(0),
      vertices_generated(0),
      edges_generated(0)
   {
      // check that we're in a real vector state space
      if (space->getType() != ompl::base::STATE_SPACE_REAL_VECTOR)
         throw std::runtime_error("RoadmapGenAAGrid only supports rel vector state spaces!");
      dim = space->getDimension();
      bounds = space->as<ompl::base::RealVectorStateSpace>()->getBounds();
      int ret = sscanf(args.c_str(), "res=%lf", &res);
      if (ret != 1)
         throw std::runtime_error("bad args to RoadmapGenAAGrid!");
      if (args != ompl_multiset::util::sf("res=%s",
         ompl_multiset::util::double_to_text(res).c_str()))
      {
         throw std::runtime_error("args not in canonical form!");
      }
      // check for infinite (no) bounds?
   }
   ~RoadmapGenAAGrid() {}
   
   std::size_t get_num_subgraphs_generated()
   {
      return num_subgraphs_generated;
   }
   
   void generate(
      typename TypeSet::Graph & g,
      std::size_t num_subgraphs_desired,
      typename TypeSet::StateMap state_map,
      typename TypeSet::DistanceMap distance_map,
      typename TypeSet::VertexSubgraphMap vertex_subgraph_map,
      typename TypeSet::EdgeSubgraphMap edge_subgraph_map,
      typename TypeSet::IsShadowMap is_shadow_map)
   {
      if (this->num_subgraphs < num_subgraphs_desired)
         throw std::runtime_error("this roadmap gen doesnt support that many subgraphs !");
      if (num_subgraphs_generated!=0 || num_subgraphs_desired!=1)
         return;
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
         
         put(vertex_subgraph_map, v_new, 0);
         put(is_shadow_map, v_new, false);
         
         // allocate a new state for this vertex
         get(state_map, v_new).reset(new typename TypeSet::StateContainer(this->space));
         ompl::base::State * v_state = get(state_map, v_new)->state;
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
               add_edge(vertices[ivert_dimprev], v_new, g);
            }
            // continue
            if (idim == 0) break;
            ivert_used /= dim_numverts[idim];
            dim_stride *= dim_numverts[idim];
         }
      }
      num_subgraphs_generated++;
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
   // from id
   double res;
   // progress
   std::size_t num_subgraphs_generated;
   std::size_t vertices_generated;
   std::size_t edges_generated;
};

} // namespace ompl_multiset
