/*! \file lazysp_selector_sp_indicator_probability.h
 * \author Chris Dellin <cdellin@gmail.com>
 * \copyright 2015 Carnegie Mellon University
 * \copyright License: BSD
 * 
 * \brief Indicator probability selector
 *        (pr_bgl::lazysp_selector_sp_indicator_probability)
 */

namespace pr_bgl
{

/*! \brief Shortest-path indicator probability selector
 *         for pr_bgl::lazysp.
 *
 * This selector simply invalidates non-evaluated edges with some
 * probability, runs a shortest-path search on the resulting graph,
 * and scores edges based on how often they are on the resulting
 * shortest path.
 */
template <class Graph, class WLazyMap, class IsEvaledMap>
class lazysp_selector_sp_indicator_probability
{
public:
   typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
   typedef typename boost::graph_traits<Graph>::edge_descriptor Edge;
   typedef typename boost::graph_traits<Graph>::edge_iterator EdgeIter;
   WLazyMap w_lazy_map;
   IsEvaledMap is_evaled_map;
   const int nsamps;
   const Vertex v_start;
   const Vertex v_goal;
   boost::mt19937 rand_gen;
   boost::uniform_01<> rand_dist;
   boost::variate_generator<boost::mt19937&, boost::uniform_01<> > rand_var;
   
   lazysp_selector_sp_indicator_probability(WLazyMap w_lazy_map, IsEvaledMap is_evaled_map,
         int nsamps, Vertex v_start, Vertex v_goal, unsigned int seed):
      w_lazy_map(w_lazy_map), is_evaled_map(is_evaled_map),
      nsamps(nsamps), v_start(v_start), v_goal(v_goal),
      rand_gen(seed), rand_var(rand_gen, rand_dist)
   {
   }
   
   void get_to_evaluate(
      const Graph & g,
      const std::vector< std::pair<Edge,bool> > & path,
      std::vector<Edge> & to_evaluate)
   {      
      std::map<Edge, int> edge_counts;
      for (std::pair<EdgeIter,EdgeIter> ep=edges(g); ep.first!=ep.second; ep.first++)
         edge_counts[*ep.first] = 0;
      
      //printf("sampling ...\n");
      for (int i=0; i<nsamps; i++)
      {
         // compute sampled edge weights
         std::map<Edge, double> w_sampled;
         for (std::pair<EdgeIter,EdgeIter> ep=edges(g); ep.first!=ep.second; ep.first++)
         {
            Edge e = *ep.first;
            if (get(is_evaled_map,e))
               w_sampled[e] = get(w_lazy_map,e);
            else
            {
               double randvalue = rand_var();
               if (0.5 < randvalue) // collides?
                  w_sampled[e] = std::numeric_limits<double>::max();
               else
                  //w_sampled[e] = get(w_lazy_map,e);
                  w_sampled[e] = 1.0 + randvalue / 0.5;
            }
         }
         
         // compute shortest path
         std::map<Vertex,double> wsampled_startdist;
         std::map<Vertex,Vertex> wsampled_preds;
         boost::dijkstra_shortest_paths(
            g,
            v_start, // source
            boost::make_assoc_property_map(wsampled_preds),
            boost::make_assoc_property_map(wsampled_startdist),
            boost::make_assoc_property_map(w_sampled),
            boost::get(boost::vertex_index, g),
            std::less<double>(), // compare
            boost::closed_plus<double>(std::numeric_limits<double>::max()), // combine
            std::numeric_limits<double>::max(),
            double(),
            boost::make_dijkstra_visitor(boost::null_visitor())
            );

         if (wsampled_startdist[v_goal] == std::numeric_limits<double>::max())
            continue;
         
         // add into edge_counts along shortest path
         Vertex v_cur = v_goal;
         while (v_cur != v_start)
         {
            Vertex v_prev = wsampled_preds[v_cur];
            Edge e = boost::edge(v_prev, v_cur, g).first;
            edge_counts[e]++;
            v_cur = v_prev;
         }
      }
      
      int count_best = 0;
      Edge e_best;
      for (unsigned int ui=0; ui<path.size(); ui++)
      {
         Edge e = path[ui].first;
         if (path[ui].second)
            continue;
         if (count_best <= edge_counts[e])
         {
            e_best = e;
            count_best = edge_counts[e];
         }
      }
      to_evaluate.push_back(e_best);
   }
   
   template <class Edge, class WeightType>
   void update_notify(Edge e, WeightType e_weight_old) {}
};

} // namespace pr_bgl
