/*! \file lazysp_selector_partition_all.h
 * \author Chris Dellin <cdellin@gmail.com>
 * \copyright 2015 Carnegie Mellon University
 * \copyright License: BSD
 * 
 * \brief Adaptors to use partition_all as a LazySP selector
 */

namespace pr_bgl
{

/*! \brief Adaptor to use non-matrix partition_all functions
 *         as a selector for pr_bgl::lazysp.
 * 
 * for now, this assumes g is an undirected graph
 *
 * we include a do_fake_roots parameter (default: false)
 * when this parameter is set,
 * then edges TO the start and edges FROM the goal are IGNORED
 * (this way, the oppposite edges can have 0 weight)
 * (this is accounted for on both initial add and incremental updates)
 *
 * also, if do_fake_roots is set,
 * then the first and last edges on the path are always
 * scored 1.0 (highest) and will therefore always be evaluated first
 */ 
template <class Graph, class WLazyMap>
class lazysp_selector_partition_all
{
public:
   typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
   typedef typename boost::graph_traits<Graph>::edge_descriptor Edge;
   typedef typename boost::graph_traits<Graph>::edge_iterator EdgeIter;
   
   typedef typename boost::property_map<Graph, boost::vertex_index_t>::type VerIndexMap;
   typedef pr_bgl::pair_index_map<Vertex,VerIndexMap> VerPairIndexMap;
   typedef boost::vector_property_map<double,VerIndexMap> VerVector;
   typedef boost::vector_property_map<double,VerPairIndexMap> VerPairVector;
   
   const Graph & g;
   WLazyMap w_lazy_map;
   const double len_ref;
   const Vertex v_start;
   const Vertex v_goal;
   const bool do_fake_roots;
   
   // coupling temps,outputs
   VerVector temp1;
   VerVector temp2;
   VerPairVector coupling_map;
   
   lazysp_selector_partition_all(
      const Graph & g,
      WLazyMap w_lazy_map, double len_ref,
      Vertex v_start, Vertex v_goal,
      bool do_fake_roots):
      g(g), w_lazy_map(w_lazy_map), len_ref(len_ref),
      v_start(v_start), v_goal(v_goal),
      do_fake_roots(do_fake_roots),
      temp1(num_vertices(g),get(boost::vertex_index,g)),
      temp2(num_vertices(g),get(boost::vertex_index,g)),
      coupling_map(
         num_vertices(g)*num_vertices(g),
         VerPairIndexMap(get(boost::vertex_index,g),num_vertices(g)))
   {
      printf("computing partition_all from scratch ...\n");
      
      // compute coupling from scratch
      pr_bgl::partition_all_init(g, coupling_map);
      
      std::pair<EdgeIter,EdgeIter> ep=edges(g);
      for (EdgeIter ei=ep.first; ei!=ep.second; ei++)
      {
         Vertex v_s = source(*ei, g);
         Vertex v_t = target(*ei, g);
         double weight_frac = get(w_lazy_map,*ei) / len_ref;
         
         if (!do_fake_roots || (v_t != v_start && v_s != v_goal))
         {
            // add forward edge s->t
            partition_all_update_directed_edge(g,
               v_s,
               v_t,
               weight_frac,
               true, // is_add
               coupling_map, temp1, temp2);
         }
         
         if (!do_fake_roots || (v_s != v_start && v_t != v_goal))
         {
            // add backwards edge t->s
            partition_all_update_directed_edge(g,
               v_t,
               v_s,
               weight_frac,
               true, // is_add
               coupling_map, temp1, temp2);
         }
      }
   }

   void get_to_evaluate(
      const Graph & g,
      const std::vector< std::pair<Edge,bool> > & path,
      std::vector<Edge> & to_evaluate)
   {
      double coupling_withall = coupling_map[std::make_pair(v_start,v_goal)];
      //printf("start-goal coupling: %e\n", coupling_withall);
      
      double score_best = 0.0;
      Edge e_best;
      bool found_best = false;
      for (unsigned int ui=0; ui<path.size(); ui++)
      {
         Edge e = path[ui].first;
         if (path[ui].second)
            continue;
         
         double score;
         if (do_fake_roots && (ui==0 || ui==path.size()-1))
            score = 1.0;
         else
         {
            // compute leave-one-out coupling score
            double coupling_without = pr_bgl::partition_all_without_edge(g, v_start, v_goal,
               e, get(w_lazy_map,e)/len_ref, coupling_map);
            //printf("  path[%u] edge coupling without: %e\n", ui, coupling_without);
            score = 1.0 - coupling_without/coupling_withall;
         }
         
         // is it better?
         if (score_best <= score)
         {
            e_best = e;
            score_best = score;
            found_best = true;
         }
      }
      if (!found_best)
         throw std::runtime_error("no best edge found!");
      to_evaluate.push_back(e_best);
   }
   
   template <class Edge, class WeightType>
   void update_notify(Edge e, WeightType e_weight_old)
   {
      //printf("accommodating updated edge ...\n");
      double e_weight_new = get(w_lazy_map, e);
      if (e_weight_new == e_weight_old)
         return;
      
      // remove in both directions (assume above check skips fake roots)
      Vertex v_s = source(e, g);
      Vertex v_t = target(e, g);
      
      double weight_frac_old = e_weight_old / len_ref;
      if (!do_fake_roots || (v_t != v_start && v_s != v_goal))
         partition_all_update_directed_edge(g, v_s, v_t, weight_frac_old, false, coupling_map, temp1, temp2);
      if (!do_fake_roots || (v_s != v_start && v_t != v_goal))
         partition_all_update_directed_edge(g, v_t, v_s, weight_frac_old, false, coupling_map, temp1, temp2);
      
      double weight_frac = get(w_lazy_map, e) / len_ref;
      if (!do_fake_roots || (v_t != v_start && v_s != v_goal))
         partition_all_update_directed_edge(g, v_s, v_t, weight_frac, true, coupling_map, temp1, temp2);
      if (!do_fake_roots || (v_s != v_start && v_t != v_goal))
         partition_all_update_directed_edge(g, v_t, v_s, weight_frac, true, coupling_map, temp1, temp2);
   }
};

/*! \brief Adaptor to use matrix partition_all functions
 *         as a LazySP selector
 */
template <class Graph, class WLazyMap>
class lazysp_partition_all_matrix
{
public:
   typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
   typedef typename boost::graph_traits<Graph>::edge_descriptor Edge;
   
   
   const Graph & g;
   WLazyMap w_lazy_map;
   const double len_ref;
   const Vertex v_start;
   const Vertex v_goal;
   const bool do_fake_roots;
   
   pr_bgl::partition_all_matrix & solver;
   
   lazysp_partition_all_matrix(
      const Graph & g,
      WLazyMap w_lazy_map, double len_ref,
      Vertex v_start, Vertex v_goal,
      bool do_fake_roots,
      pr_bgl::partition_all_matrix & solver):
      g(g), w_lazy_map(w_lazy_map), len_ref(len_ref),
      v_start(v_start), v_goal(v_goal),
      do_fake_roots(do_fake_roots),
      solver(solver)
   {
   }

   void get_to_evaluate(
      const Graph & g,
      const std::vector< std::pair<Edge,bool> > & path,
      std::vector<Edge> & to_evaluate)
   {
      double coupling_withall = solver.Z(v_start, v_goal);
      //printf("start-goal coupling: %e\n", coupling_withall);
      
      double score_best = 0.0;
      Edge e_best;
      bool found_best = false;
      for (unsigned int ui=0; ui<path.size(); ui++)
      {
         Edge e = path[ui].first;
         if (path[ui].second)
            continue;
         
         double score;
         if (do_fake_roots && (ui==0 || ui==path.size()-1))
            score = 1.0;
         else
         {
            double weight_frac = get(w_lazy_map,e)/len_ref;
            double coupling_without = solver.without_undirected(
               v_start, v_goal, source(e,g), target(e,g), weight_frac);
            
            //printf("  path[%u] edge coupling without: %e\n", ui, coupling_without);
            score = 1.0 - coupling_without/coupling_withall;
         }
         
         // is it better?
         if (score_best <= score)
         {
            e_best = e;
            score_best = score;
            found_best = true;
         }
      }
      if (!found_best)
         throw std::runtime_error("no best edge found!");
      to_evaluate.push_back(e_best);
   }
   
   template <class Edge, class WeightType>
   void update_notify(Edge e, WeightType e_weight_old)
   {
      //printf("accommodating updated edge ...\n");
      double e_weight_new = get(w_lazy_map, e);
      if (e_weight_new == e_weight_old)
         return;
      
      // remove in both directions (assume above check skips fake roots)
      Vertex va = source(e, g);
      Vertex vb = target(e, g);
      
      // remove in both directions
      double weight_frac_old = e_weight_old / len_ref;
      if (!do_fake_roots || (vb != v_start && va != v_goal))
         solver.remove_edge(va, vb, weight_frac_old);
      if (!do_fake_roots || (va != v_start && vb != v_goal))
         solver.remove_edge(vb, va, weight_frac_old);
      
      // add in both directions
      double weight_frac = get(w_lazy_map, e) / len_ref;
      if (!do_fake_roots || (vb != v_start && va != v_goal))
         solver.add_edge(va, vb, weight_frac);
      if (!do_fake_roots || (va != v_start && vb != v_goal))
         solver.add_edge(vb, va, weight_frac);
   }
};

} // namespace pr_bgl
