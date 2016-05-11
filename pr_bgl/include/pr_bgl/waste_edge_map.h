/*! \file waste_edge_map.h
 * \author Chris Dellin <cdellin@gmail.com>
 * \copyright 2015 Carnegie Mellon University
 * \copyright License: BSD
 * 
 * \brief Waste edge map (apply a vertex potential function to
 *        edge weights).
 */

namespace pr_bgl
{

// assumes vertex heuristic map is goal-directed (smaller at goal)
// that is, enew = eold + h(v_target) - h(v_source)
template <typename Graph, typename EdgeMap, typename HeurMap>
class waste_edge_map
{
public:
   typedef typename boost::property_traits<EdgeMap>::category category;
   typedef typename boost::property_traits<EdgeMap>::key_type key_type; 
   typedef typename boost::property_traits<EdgeMap>::value_type value_type;
   typedef typename boost::property_traits<EdgeMap>::reference reference;

   Graph & graph;
   const EdgeMap & edge_map;
   const HeurMap & heur_map;
   
   waste_edge_map(Graph & graph, const EdgeMap & edge_map, const HeurMap & heur_map):
      graph(graph), edge_map(edge_map), heur_map(heur_map)
   {
   }
};

template <typename Graph, typename EdgeMap, typename HeurMap>
waste_edge_map<Graph,EdgeMap,HeurMap>
make_waste_edge_map(Graph & graph, const EdgeMap & edge_map, const HeurMap & heur_map)
{
   return waste_edge_map<Graph,EdgeMap,HeurMap>(graph, edge_map, heur_map);
}

template <typename Graph, typename EdgeMap, typename HeurMap>
inline const double get(
   const waste_edge_map<Graph,EdgeMap,HeurMap> & map,
   const typename boost::graph_traits<Graph>::edge_descriptor & edge)
{
   return get(map.edge_map, edge)
      + get(map.heur_map, target(edge, map.graph))
      - get(map.heur_map, source(edge, map.graph));
}

} // namespace pr_bgl
