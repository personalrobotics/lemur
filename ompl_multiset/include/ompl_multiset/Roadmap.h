/* File: Roadmap.h
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

namespace ompl_multiset
{

template <typename Vertex>
class NNDummy
{
public:
   inline void nearestR(Vertex v_new, double radius, std::vector< std::pair<Vertex,double> > & vs_near) { abort(); }
};

template <class Graph, class VState>
class NNLinear
{
public:
   typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
   Graph & g;
   VState state_map;
   const ompl::base::StateSpacePtr space;
   NNLinear(Graph & g, VState state_map, const ompl::base::StateSpacePtr space):
      g(g), state_map(state_map), space(space)
   {
   }
   inline void nearestR(Vertex v_new, double radius, std::vector< std::pair<Vertex,double> > & vs_near)
   {
      vs_near.clear();
      for (unsigned int ui=0; ui<num_vertices(g); ui++)
      {
         Vertex v_other = vertex(ui, g);
         if (v_other == v_new)
            continue;
         double dist = this->space->distance(
            get(state_map, v_new)->state,
            get(state_map, v_other)->state);
         if (radius < dist)
            continue;
         vs_near.push_back(std::make_pair(v_other,dist));
      }
   }
   void sync() {}
};

template <class Graph, class VState>
class NNOmplBatched
{
public:
   typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
   Graph & g;
   VState state_map;
   const ompl::base::StateSpacePtr space;
   ompl::NearestNeighbors<Vertex> * ompl_nn;
   NNOmplBatched(Graph & g, VState state_map,
      const ompl::base::StateSpacePtr space,
      ompl::NearestNeighbors<Vertex> * ompl_nn):
      g(g), state_map(state_map), space(space), ompl_nn(ompl_nn)
   {
   }
   inline void nearestR(Vertex v_new, double radius, std::vector< std::pair<Vertex,double> > & vrads_near)
   {
      //printf("nearestR NNOmplBatched ...\n");
      vrads_near.clear();
      // add in from ompl_nn object
      std::vector<Vertex> vs_near;
      ompl_nn->nearestR(v_new, radius, vs_near);
      for (unsigned int ui=0; ui<vs_near.size(); ui++)
      {
         if (vs_near[ui] == v_new)
            continue;
         double dist = space->distance(
            get(state_map, v_new)->state,
            get(state_map, vs_near[ui])->state);
         vrads_near.push_back(std::make_pair(vs_near[ui],dist));
      }
      //printf("  found %lu from ompl_nn.\n", vrads_near.size());
      // add in additionals since last sync
      for (unsigned int ui=ompl_nn->size(); ui<num_vertices(g); ui++)
      {
         Vertex v_other = vertex(ui, g);
         if (v_other == v_new)
            continue;
         double dist = this->space->distance(
            get(state_map, v_new)->state,
            get(state_map, v_other)->state);
         if (radius < dist)
            continue;
         vrads_near.push_back(std::make_pair(v_other,dist));
      }
      //printf("  found %lu total.\n", vrads_near.size());
   }
   void sync()
   {
      printf("syncing NNOmplBatched ...\n");
      for (unsigned int ui=ompl_nn->size(); ui<num_vertices(g); ui++)
         ompl_nn->add(vertex(ui,g));
   }
};

// continuous space
// generates a possibly infinite roadmap given an ompl space
// note, generate() should tolerate the graph having unused but
// added vertices (e.g. from old/unapplied roots)
//template <class Graph, class VertexIndexMap, class EdgeIndexMap
   //,class StateMap, class BatchMap, class IsShadowMap, class DistanceMap
//   >
template <class Graph, class VState, class EDistance, class VBatch, class EBatch, class VShadow, class NN>
class Roadmap
{
public:
   typedef Graph BaseGraph;
   typedef VState BaseVState;
   typedef EDistance BaseEDistance;
   typedef VBatch BaseVBatch;
   typedef EBatch BaseEBatch;
   typedef VShadow BaseVShadow;
   typedef NN BaseNN;

   const ompl::base::StateSpacePtr space;
   const std::size_t max_batches; // 0 means inf
   
   // TODO: make this per-type, and move string parsing into RoadmapID
   Roadmap(const ompl::base::StateSpacePtr space, int max_batches):
      space(space), max_batches(max_batches)
   {
   }
   virtual ~Roadmap() {}
   
   virtual std::size_t get_num_batches_generated() = 0;
   
   virtual double root_radius(std::size_t i_batch) = 0;
   
   // sets all of these maps
   // generates one additional batch
   virtual void generate(
      Graph & g,
      NN & nn,
      VState state_map,
      EDistance distance_map,
      VBatch vertex_batch_map,
      EBatch edge_batch_map,
      VShadow is_shadow_map) = 0;
   
   virtual void serialize() = 0;
   virtual void deserialize() = 0;
};

} // namespace ompl_multiset
