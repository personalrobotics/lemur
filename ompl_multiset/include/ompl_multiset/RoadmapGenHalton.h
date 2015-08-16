/* File: RoadmapGenHalton.h
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

/* requires:
#include <ompl_multiset/SamplerGenMonkeyPatch.h>
*/

namespace ompl_multiset
{


int primes[] =
{
     2,   3,   5,   7,  11,  13,  17,  19,  23,  29,
    31,  37,  41,  43,  47,  53,  59,  61,  67,  71,
    73,  79,  83,  89,  97, 101, 103, 107, 109, 113,
   127, 131, 137, 139, 149, 151, 157, 163, 167, 173,
   179, 181, 191, 193, 197, 199, 211, 223, 227, 229,
   233, 239, 241, 251, 257, 263, 269, 271, 277, 281,
   283, 293, 307, 311, 313, 317, 331, 337, 347, 349,
   353, 359, 367, 373, 379, 383, 389, 397, 401, 409,
   419, 421, 431, 433, 439, 443, 449, 457, 461, 463,
   467, 479, 487, 491, 499, 503, 509, 521, 523, 541
};

// index is 0-indexed
double halton(int prime, int index)
{
   double sample = 0.0;
   double denom = prime;
   for (index++; index; index/=prime, denom*=prime)
      sample += (index % prime) / denom;
   return sample;
}



// for now this is an r-disk prm,
// uniform milestone sampling with given seed,
// uses the space's default sampler
//template <class Graph, class VertexIndexMap, class EdgeIndexMap//,
   //class StateMap, class SubgraphMap, class IsShadowMap, class DistanceMap
//   >
template <class TypeSet>
class RoadmapGenHalton : public RoadmapGen<TypeSet>
{
   typedef boost::graph_traits<typename TypeSet::Graph> GraphTypes;
   typedef typename GraphTypes::vertex_descriptor Vertex;
   typedef typename GraphTypes::vertex_iterator VertexIter;
   typedef typename GraphTypes::edge_descriptor Edge;
   typedef typename GraphTypes::edge_iterator EdgeIter;
   typedef typename GraphTypes::out_edge_iterator EdgeOutIter;
   typedef typename GraphTypes::in_edge_iterator EdgeInIter;
   
public:
   RoadmapGenHalton(
      const ompl::base::StateSpacePtr space,
      const std::string args):
      RoadmapGen<TypeSet>(space,"RoadmapGenHalton",args,1),
      dim(0),
      bounds(0),
      num_subgraphs_generated(0),
      vertices_generated(0),
      edges_generated(0)
   {
      // check that we're in a real vector state space
      if (space->getType() != ompl::base::STATE_SPACE_REAL_VECTOR)
         throw std::runtime_error("RoadmapGenHalton only supports rel vector state spaces!");
      dim = space->getDimension();
      if (sizeof(primes)/sizeof(primes[0]) < dim)
         throw std::runtime_error("too many dims, not enough primes hardcoded!");
      bounds = space->as<ompl::base::RealVectorStateSpace>()->getBounds();
      int ret = sscanf(args.c_str(), "n=%u radius=%lf", &n, &radius);
      if (ret != 2)
         throw std::runtime_error("bad args to RoadmapGenHalton!");
      if (args != ompl_multiset::util::sf("n=%u radius=%s",
         n, ompl_multiset::util::double_to_text(radius).c_str()))
      {
         throw std::runtime_error("args not in canonical form!");
      }
   }
   ~RoadmapGenHalton() {}
   
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
      // ok, generate n nodes!
      while (num_vertices(g) < n)
      {
         Vertex v_new = add_vertex(g);
         // set vertex index? (only for non-adjacecy list)
         
         put(vertex_subgraph_map, v_new, 0);
         put(is_shadow_map, v_new, false);
         
         // allocate a new state for this vertex
         //put(state_map, v_new,
         get(state_map, v_new).reset(
            new typename TypeSet::StateContainer(this->space)
         );
         ompl::base::State * v_state = get(state_map, v_new)->state;
         double * values = v_state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
         for (unsigned int ui=0; ui<dim; ui++)
            values[ui] = bounds.low[ui] + (bounds.high[ui] - bounds.low[ui]) * halton(primes[ui], vertices_generated);
                  
         // allocate new undirected edges
         for (unsigned int ui=0; ui<num_vertices(g)-1; ui++)
         {
            Vertex v_other = vertex(ui, g);
            double dist = this->space->distance(
               get(state_map, v_new)->state,
               get(state_map, v_other)->state);
            if (this->radius < dist)
               continue;
            Edge e = add_edge(v_new, v_other, g).first;
            put(distance_map, e, dist);
            put(edge_subgraph_map, e, 0);
            edges_generated++;
         }
         
         vertices_generated++;
      }
   }
   
   void serialize()
   {
   }
   
   void deserialize()
   {
   }
   
private:
   // from space
   unsigned int dim;
   ompl::base::RealVectorBounds bounds;
   // from id
   unsigned int n;
   double radius;
   // progress
   std::size_t num_subgraphs_generated;
   std::size_t vertices_generated;
   std::size_t edges_generated;
};

} // namespace ompl_multiset
