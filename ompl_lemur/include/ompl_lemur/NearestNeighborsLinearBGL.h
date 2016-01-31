/* File: NearestNeighborsLinearBGL.h
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

namespace ompl_lemur
{

template <class Graph, class VState>
class NearestNeighborsLinearBGL
{
public:
   typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
   Graph & g;
   VState state_map;
   const ompl::base::StateSpacePtr space;
   NearestNeighborsLinearBGL(Graph & g, VState state_map, const ompl::base::StateSpacePtr space):
      g(g), state_map(state_map), space(space)
   {
   }
   inline void add(Vertex v_new) {}
   inline void nearestR(Vertex v_new, double radius, std::vector<Vertex> & vs_near)
   {
      vs_near.clear();
      for (unsigned int ui=0; ui<num_vertices(g); ui++)
      {
         Vertex v_other = vertex(ui, g);
         double dist = this->space->distance(
            get(state_map, v_new),
            get(state_map, v_other));
         if (radius < dist)
            continue;
         vs_near.push_back(v_other);
      }
   }
   void sync() {}
};

} // namespace ompl_lemur
