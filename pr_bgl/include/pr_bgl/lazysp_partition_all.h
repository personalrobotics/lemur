/* File: lazysp_partition_all.h
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

namespace pr_bgl
{

template <class Graph, class WLazyMap>
class lazysp_partition_all
{
public:
   typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
   typedef typename boost::graph_traits<Graph>::edge_descriptor Edge;
   WLazyMap w_lazy_map;
   const double len_ref;
   const Vertex v_start;
   const Vertex v_goal;
   
   lazysp_partition_all(WLazyMap w_lazy_map, double len_ref, Vertex v_start, Vertex v_goal):
      w_lazy_map(w_lazy_map), len_ref(len_ref), v_start(v_start), v_goal(v_goal)
   {
   }
   
   //emplate <class Graph>
   void get_to_evaluate(
      const Graph & g,
      const std::vector< std::pair<Edge,bool> > & path,
      std::vector<Edge> & to_evaluate)
   {
      typedef typename boost::property_map<Graph, boost::vertex_index_t>::type VerIndexMap;
      
      // coupling temps,outputs
      typedef pr_bgl::PairIndexMap<Vertex,VerIndexMap> VerPairIndexMap;
      typedef boost::vector_property_map<double,VerIndexMap> VerVector;
      typedef boost::vector_property_map<double,VerPairIndexMap> VerPairVector;
      VerVector temp1(boost::num_vertices(g),boost::get(boost::vertex_index,g));
      VerVector temp2(boost::num_vertices(g),boost::get(boost::vertex_index,g));
      VerPairVector coupling_map(
         boost::num_vertices(g)*boost::num_vertices(g),
         VerPairIndexMap(boost::get(boost::vertex_index,g),
         boost::num_vertices(g)));
      
      // compute coupling from scratch
      pr_bgl::partition_all(g, len_ref,
         w_lazy_map,
         coupling_map, temp1, temp2);
      double coupling_withall = coupling_map[std::make_pair(v_start,v_goal)];
      //printf("start-goal coupling: %.20f\n", coupling_withall);
      
      double score_best = 0.0;
      Edge e_best;
      for (unsigned int ui=0; ui<path.size(); ui++)
      {
         Edge e = path[ui].first;
         if (path[ui].second)
            continue;
         
         // compute leave-one-out coupling score
         double coupling_without = pr_bgl::partition_all_without_edge(g, v_start, v_goal,
            e, get(w_lazy_map,e)/len_ref, coupling_map);
         double score = 1.0 - coupling_without/coupling_withall;
         
         // is it better?
         if (score_best <= score)
         {
            e_best = e;
            score_best = score;
         }
      }
      to_evaluate.push_back(e_best);
   }
};

} // namespace pr_bgl
