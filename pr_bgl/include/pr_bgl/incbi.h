
namespace pr_bgl
{

// for now this is not templated (requires typedefs above)
// uses my custom indexed heap implementation
// todo: between calls, can the graph can get bigger? can the roots change?
// todo: can i make this work for undirected graphs too?
template < class Graph, class WeightMap, class IndexMap >
class IncBi
{
public:

   typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
   typedef typename boost::graph_traits<Graph>::edge_descriptor Edge;
   typedef typename boost::graph_traits<Graph>::out_edge_iterator EdgeOutIter;
   typedef typename boost::graph_traits<Graph>::in_edge_iterator EdgeInIter;

   const Graph & m_g;
   Vertex m_vstart;
   Vertex m_vgoal;
   
   // invariant: consistent (rs vs ds and weights)
   std::vector<double> m_ds_backing;
   std::vector<double> m_rs_backing;
   std::vector<double> m_dg_backing;
   std::vector<double> m_rg_backing;
   
   typedef boost::iterator_property_map<
      std::vector<double>::iterator,
      typename boost::property_map<Graph, boost::vertex_index_t>::type,
      double, double& > VecDoublePropertyMap;
   VecDoublePropertyMap m_ds;
   VecDoublePropertyMap m_rs;
   VecDoublePropertyMap m_dg;
   VecDoublePropertyMap m_rg;
   IndexMap m_vimap;
   
   HeapIndexed m_queue_start;
   HeapIndexed m_queue_goal;
   HeapIndexed m_queue_conn; // contains all CONSISTENT non-inf nodes sorted by sum ds+dg
   
   WeightMap m_w;

   IncBi(const Graph& g, Vertex vstart, Vertex vgoal, WeightMap weight, IndexMap index_map):
      m_g(g), m_vstart(vstart), m_vgoal(vgoal), m_w(weight), m_vimap(index_map)
   {
      m_ds_backing.resize(boost::num_vertices(m_g), std::numeric_limits<double>::infinity());
      m_rs_backing.resize(boost::num_vertices(m_g), std::numeric_limits<double>::infinity());
      m_dg_backing.resize(boost::num_vertices(m_g), std::numeric_limits<double>::infinity());
      m_rg_backing.resize(boost::num_vertices(m_g), std::numeric_limits<double>::infinity());
      
      m_ds = boost::make_iterator_property_map(m_ds_backing.begin(), boost::get(boost::vertex_index, m_g));
      m_rs = boost::make_iterator_property_map(m_rs_backing.begin(), boost::get(boost::vertex_index, m_g));
      m_dg = boost::make_iterator_property_map(m_dg_backing.begin(), boost::get(boost::vertex_index, m_g));
      m_rg = boost::make_iterator_property_map(m_rg_backing.begin(), boost::get(boost::vertex_index, m_g));
   
      boost::put(m_rs, m_vstart, 0.0);
      m_queue_start.insert(boost::get(m_vimap, m_vstart), 0.0);
      boost::put(m_rg, m_vgoal, 0.0);
      m_queue_goal.insert(boost::get(m_vimap, m_vgoal), 0.0);
   }

   void notify_changed_edges(
      typename std::vector<Edge>::iterator begin,
      typename std::vector<Edge>::iterator end)
   {
      if (m_ds_backing.size() != boost::num_vertices(m_g))
         throw std::runtime_error("graph size changed!");

      for (typename std::vector<Edge>::iterator it=begin; it!=end; ++it)
      {
         Edge e = *it;
         
         // d-values dont change, but update r-values given new edge weight (and possibly update queues)
         // start side
         do
         {
            Vertex v_target = boost::target(e, m_g);
            unsigned int vidx_target = boost::get(m_vimap, v_target);
            double r_best = std::numeric_limits<double>::infinity();
            for (std::pair<EdgeInIter,EdgeInIter> ep=boost::in_edges(v_target,m_g); ep.first!=ep.second; ep.first++)
            {
               Edge e2 = *ep.first;
               double r
                  = boost::get(m_ds, boost::source(e2,m_g))
                  + boost::get(m_w, e2);
               if (r < r_best)
                  r_best = r;
            }
            if (boost::get(m_rs,v_target) == r_best)
               break;
            
            boost::put(m_rs, v_target, r_best);
            
            bool is_enqueued = m_queue_start.contains(vidx_target);
            bool is_consistent = (r_best == boost::get(m_ds,v_target));
            if (is_enqueued && is_consistent)
            {
               // vertex is newly start-consistent
               m_queue_start.remove(vidx_target);
               // add to conn queue if already goal-consistent and non-inf
               if (!m_queue_goal.contains(vidx_target))
               {
                  double d_sum = boost::get(m_ds,v_target) + boost::get(m_dg,v_target);
                  if (d_sum != std::numeric_limits<double>::infinity())
                     m_queue_conn.insert(vidx_target, d_sum);
               }
            }
            else if (!is_enqueued && !is_consistent)
            {
               // vertex is newly start-inconsistent
               m_queue_start.insert(vidx_target, std::min(r_best,boost::get(m_ds,v_target)));
               if (m_queue_conn.contains(vidx_target))
                  m_queue_conn.remove(vidx_target);
            }
            else if (is_enqueued && !is_consistent)
            {
               // vertex is still start-inconsistent
               m_queue_start.update(vidx_target, std::min(r_best,boost::get(m_ds,v_target)));
            }
         }
         while (0);
         
         // goal side
         do
         {
            Vertex v_source = boost::source(e, m_g);
            unsigned int vidx_source = boost::get(m_vimap, v_source);
            double r_best = std::numeric_limits<double>::infinity();
            for (std::pair<EdgeOutIter,EdgeOutIter> ep=boost::out_edges(v_source,m_g); ep.first!=ep.second; ep.first++)
            {
               Edge e2 = *ep.first;
               double r
                  = boost::get(m_w, e2)
                  + boost::get(m_dg, boost::target(e2,m_g));
               if (r < r_best)
                  r_best = r;
            }
            if (boost::get(m_rg,v_source) == r_best)
               break;
            
            boost::put(m_rg, v_source, r_best);
            
            bool is_enqueued = m_queue_goal.contains(vidx_source);
            bool is_consistent = (r_best == boost::get(m_dg,v_source));
            if (is_enqueued && is_consistent)
            {
               // vertex is newly goal-consistent
               m_queue_goal.remove(vidx_source);
               // add to conn queue if already start-consistent and non-inf
               if (!m_queue_start.contains(vidx_source))
               {
                  double d_sum = boost::get(m_ds,v_source) + boost::get(m_dg,v_source);
                  if (d_sum != std::numeric_limits<double>::infinity())
                     m_queue_conn.insert(vidx_source, d_sum);
               }
            }
            else if (!is_enqueued && !is_consistent)
            {
               // vertex is newly start-inconsistent
               m_queue_goal.insert(vidx_source, std::min(r_best,boost::get(m_dg,v_source)));
               if (m_queue_conn.contains(vidx_source))
                  m_queue_conn.remove(vidx_source);
            }
            else if (is_enqueued && !is_consistent)
            {
               // vertex is still start-inconsistent
               m_queue_goal.update(vidx_source, std::min(r_best,boost::get(m_dg,v_source)));
            }
         }
         while (0);
      }
   }
   
   template <class Visitor>
   void search(double beta_s, double beta_g, Visitor & visitor, std::list<Edge> & epath)
   {
      if (m_ds_backing.size() != boost::num_vertices(m_g))
         throw std::runtime_error("graph size changed!");
      
      epath.clear();
      
      for (;;)
      {
         if (!m_queue_start.size() || !m_queue_goal.size())
            throw std::runtime_error("one of our queues is empty! oh noes!\n");
         
         visitor.dump(*this);
         
         // should we terminate? (like from goldberg)
         bool terminate = false;
         do
         {
            if (!m_queue_conn.size()) break;
            Vertex v_conn = boost::vertex(m_queue_conn.top_idx(), m_g);
            if (m_queue_start.top_key() < boost::get(m_ds,v_conn)) break;
            if (m_queue_goal.top_key() < boost::get(m_dg,v_conn)) break;
            terminate = true;
         }
         while (0);
         if (terminate)
         {
            printf("SEARCH done! with key: %f\n", boost::vertex(m_queue_conn.top_idx(), m_g));
            break;
         }
         
         // the current inconsistent vertex
         size_t vidx_cur;
         Vertex v_cur;
         
         // pull from whichever queue is smaller
         if (beta_s * m_queue_start.top_key() <= beta_g * m_queue_goal.top_key())
         {
            vidx_cur = m_queue_start.top_idx();
            v_cur = boost::vertex(vidx_cur, m_g);
            printf("pulling from start queue, vidx=%u w key=%f!\n",
               vidx_cur, m_queue_start.top_key());
            m_queue_start.remove_min();
            
            if (boost::get(m_ds,v_cur) > boost::get(m_rs,v_cur))
            {
               /* currently overconsistent */
               
               // make vertex start-consistent
               double d = boost::get(m_rs,v_cur);
               boost::put(m_ds, v_cur, d);
               
               // newly add to conn queue if non-inf sum
               if (!m_queue_goal.contains(vidx_cur))
               {
                  double d_sum = boost::get(m_ds,v_cur) + boost::get(m_dg,v_cur);
                  if (d_sum != std::numeric_limits<double>::infinity())
                     m_queue_conn.insert(vidx_cur, d_sum);
               }
               
               // get all dependants (successors)
               // to ensure r invariant is still satisfied!
               // in this case, since our d got SMALLER,
               // the r-values of successors can only get SMALLER because of US
               for (std::pair<EdgeOutIter,EdgeOutIter> ep=boost::out_edges(v_cur,m_g); ep.first!=ep.second; ep.first++)
               {
                  Edge e = *ep.first;
                  
                  printf("found a successor!\n");
                  
                  double weight = boost::get(m_w, e);
                  printf("edge weight: %f\n", weight);
                  
                  double r_target_me = d + weight;
                  
                  Vertex v_target = boost::target(e, m_g);
                  
                  if (boost::get(m_rs,v_target) <= r_target_me)
                     continue; /* still correct */
                  
                  // make r correct
                  boost::put(m_rs,v_target, r_target_me);
                  
                  size_t vidx_target = boost::get(m_vimap, v_target);
                  bool is_enqueued = m_queue_start.contains(vidx_target);
                  bool should_be_enqueued = !(r_target_me == boost::get(m_ds,v_target));
                  if (is_enqueued && !should_be_enqueued)
                  {
                     // dependant is newly start-consistent
                     m_queue_start.remove(vidx_target);
                     // add to conn queue if already goal-consistent and non-inf
                     if (!m_queue_goal.contains(vidx_target))
                     {
                        double d_sum = boost::get(m_ds,v_target) + boost::get(m_dg,v_target);
                        if (d_sum != std::numeric_limits<double>::infinity())
                           m_queue_conn.insert(vidx_target, d_sum);
                     }
                  }
                  else if (!is_enqueued && should_be_enqueued)
                  {
                     // dependant is newly inconsistent
                     m_queue_start.insert(vidx_target, r_target_me);
                     if (m_queue_conn.contains(vidx_target))
                        m_queue_conn.remove(vidx_target);
                  }
                  else if (is_enqueued && should_be_enqueued)
                  {
                     // dependant is still inconsistent
                     m_queue_start.update(vidx_target, r_target_me);
                  }
               }
            }
            else
            {
               /* currently underconsistent */
               
               // make vertex overconsistent or consistent
               boost::put(m_ds, v_cur, std::numeric_limits<double>::infinity());
               
               // if now unconsistent, put back in the queue
               double r = boost::get(m_rs, v_cur);
               if (r != std::numeric_limits<double>::infinity())
                  m_queue_start.insert(vidx_cur, r);

               // this vertex is now either still inconsistent or consistent with inf sum,
               // so no need to add it to the conn queue
               
               // since my d-value is changed, update r-value of all successors who might have depended on me
               // and ensure their queue positions are correct
               // those whose r-values didnt depend solely on me wont change at all;
               // those that did will have r-values that will get higher (either to inf or not)
               for (std::pair<EdgeOutIter,EdgeOutIter> ep=boost::out_edges(v_cur,m_g); ep.first!=ep.second; ep.first++)
               {
                  Vertex v_succ = boost::target(*ep.first, m_g);
                  size_t vidx_succ = boost::get(m_vimap, v_succ);
                  
                  double r_best = std::numeric_limits<double>::infinity();
                  for (std::pair<EdgeInIter,EdgeInIter> ep2=boost::in_edges(v_succ,m_g); ep2.first!=ep2.second; ep2.first++)
                  {
                     Edge e = *ep2.first;
                     double r_me
                        = boost::get(m_ds,boost::source(e,m_g))
                        + boost::get(m_w, e);
                     if (r_me < r_best)
                        r_best = r_me;
                  }
                  if (boost::get(m_rs, v_succ) == r_best)
                     continue;

                  // r is incorrect (because of me); make r correct (it will get bigger, perhaps to inf)
                  boost::put(m_rs, v_succ, r_best);
                  
                  bool is_enqueued = m_queue_start.contains(vidx_succ);
                  bool is_consistent = (r_best == boost::get(m_ds,v_succ));
                  if (is_enqueued && is_consistent)
                  {
                     // vertex is newly start-consistent (higher r now matches current d)
                     m_queue_start.remove(vidx_succ);
                     // add to conn queue if already goal-consistent and non-inf
                     if (!m_queue_goal.contains(vidx_succ))
                     {
                        double d_sum = boost::get(m_ds,v_succ) + boost::get(m_dg,v_succ);
                        if (d_sum != std::numeric_limits<double>::infinity())
                           m_queue_conn.insert(vidx_succ, d_sum);
                     }
                  }
                  else if (!is_enqueued && !is_consistent)
                  {
                     // vertex is newly start-inconsistent
                     m_queue_start.insert(vidx_succ, std::min(r_best,boost::get(m_ds,v_succ)));
                     if (m_queue_conn.contains(vidx_succ))
                        m_queue_conn.remove(vidx_succ);
                  }
                  else if (is_enqueued && !is_consistent)
                  {
                     // vertex is still start-inconsistent
                     m_queue_start.update(vidx_succ, std::min(r_best,boost::get(m_ds,v_succ)));
                  }
               }
            }
         }
         else
         {
            vidx_cur = m_queue_goal.top_idx();
            v_cur = boost::vertex(vidx_cur, m_g);
            printf("pulling from goal queue, vidx=%u w key=%f!\n",
               vidx_cur, m_queue_goal.top_key());
            m_queue_goal.remove_min();
            
            if (boost::get(m_dg,v_cur) > boost::get(m_rg,v_cur))
            {
               /* currently overconsistent */
               
               // make vertex goal-consistent
               double d = boost::get(m_rg, v_cur);
               boost::put(m_dg, v_cur, d);
               
               // newly add to conn queue if non-inf sum
               if (!m_queue_start.contains(vidx_cur))
               {
                  double d_sum = boost::get(m_ds,v_cur) + boost::get(m_dg,v_cur);
                  if (d_sum != std::numeric_limits<double>::infinity())
                     m_queue_conn.insert(vidx_cur, d_sum);
               }
               
               // get all dependants (predecessors)
               // to ensure r invariant is still satisfied!
               // in this case, since our d got SMALLER,
               // the r-values of successors can only get SMALLER because of US
               for (std::pair<EdgeInIter,EdgeInIter> ep=boost::in_edges(v_cur,m_g); ep.first!=ep.second; ep.first++)
               {
                  Edge e = *ep.first;
                  
                  printf("found a predecessor!\n");
                  
                  double weight = boost::get(m_w, e);
                  printf("edge weight: %f\n", weight);
                  
                  double r_source_me = d + weight;
                  
                  Vertex v_source = boost::source(e, m_g);
                  
                  if (boost::get(m_rg, v_source) <= r_source_me)
                     continue; /* still correct */
                  
                  // make r correct
                  boost::put(m_rg, v_source, r_source_me);
                  
                  size_t vidx_source = boost::get(m_vimap, v_source);
                  bool is_enqueued = m_queue_goal.contains(vidx_source);
                  bool should_be_enqueued = !(r_source_me == boost::get(m_dg,v_source));
                  if (is_enqueued && !should_be_enqueued)
                  {
                     // dependant is newly goal-consistent
                     m_queue_goal.remove(vidx_source);
                     // add to conn queue if already start-consistent and non-inf
                     if (!m_queue_start.contains(vidx_source))
                     {
                        double d_sum = boost::get(m_ds,v_source) + boost::get(m_dg,v_source);
                        if (d_sum != std::numeric_limits<double>::infinity())
                           m_queue_conn.insert(vidx_source, d_sum);
                     }
                  }
                  else if (!is_enqueued && should_be_enqueued)
                  {
                     // dependant is newly inconsistent
                     m_queue_goal.insert(vidx_source, r_source_me);
                     if (m_queue_conn.contains(vidx_source))
                        m_queue_conn.remove(vidx_source);
                  }
                  else if (is_enqueued && should_be_enqueued)
                  {
                     // dependant is still inconsistent
                     m_queue_goal.update(vidx_source, r_source_me);
                  }
               }
            }
            else
            {
               /* currently underconsistent */
               
               // make vertex overconsistent or consistent
               boost::put(m_dg, v_cur, std::numeric_limits<double>::infinity());
               
               // if now unconsistent, put back in the queue
               double r = boost::get(m_rg, v_cur);
               if (r != std::numeric_limits<double>::infinity())
                  m_queue_goal.insert(vidx_cur, r);
               
               // this vertex is now either still inconsistent or consistent with inf sum,
               // so no need to add it to the conn queue
               
               // since my d-value is changed, update r-value of all predecessors who might have depended on me
               // and ensure their queue positions are correct
               for (std::pair<EdgeInIter,EdgeInIter> ep=boost::in_edges(v_cur,m_g); ep.first!=ep.second; ep.first++)
               {
                  Vertex v_pred = boost::source(*ep.first, m_g);
                  size_t vidx_pred = boost::get(m_vimap, v_pred);
                  
                  double r_best = std::numeric_limits<double>::infinity();
                  for (std::pair<EdgeOutIter,EdgeOutIter> ep2=boost::out_edges(v_pred,m_g); ep2.first!=ep2.second; ep2.first++)
                  {
                     Edge e = *ep2.first;
                     double r_me
                        = boost::get(m_dg,boost::target(e,m_g))
                        + boost::get(m_w, e);
                     if (r_me < r_best)
                        r_best = r_me;
                  }
                  if (boost::get(m_rg, v_pred) == r_best)
                     continue;

                  // r is incorrect (because of me); make r correct (it will get bigger, perhaps to inf)
                  boost::put(m_rg, v_pred, r_best);
                  
                  bool is_enqueued = m_queue_goal.contains(vidx_pred);
                  bool is_consistent = (r_best == boost::get(m_dg,v_pred));
                  if (is_enqueued && is_consistent)
                  {
                     // vertex is newly goal-consistent (higher r now matches current d)
                     m_queue_goal.remove(vidx_pred);
                     // add to conn queue if already start-consistent and non-inf
                     if (!m_queue_start.contains(vidx_pred))
                     {
                        double d_sum = boost::get(m_ds,v_pred) + boost::get(m_dg,v_pred);
                        if (d_sum != std::numeric_limits<double>::infinity())
                           m_queue_conn.insert(vidx_pred, d_sum);
                     }
                  }
                  else if (!is_enqueued && !is_consistent)
                  {
                     // vertex is newly start-inconsistent
                     m_queue_goal.insert(vidx_pred, std::min(r_best,boost::get(m_dg,v_pred)));
                     if (m_queue_conn.contains(vidx_pred))
                        m_queue_conn.remove(vidx_pred);
                  }
                  else if (is_enqueued && !is_consistent)
                  {
                     // vertex is still start-inconsistent
                     m_queue_goal.update(vidx_pred, std::min(r_best,boost::get(m_dg,v_pred)));
                  }
               }
            }
         }
      }
      
      // get path
      {
         Vertex v;
         // walk to start
         v = boost::vertex(m_queue_conn.top_idx(), m_g);
         printf("getting path to start ...\n");
         while (boost::get(m_ds, v))
         {
            printf("front vertex: %u, with d-value %f\n", boost::get(m_vimap,v), boost::get(m_ds,v));
            double ds_best = std::numeric_limits<double>::infinity();
            Edge e_best;
            for (std::pair<EdgeInIter,EdgeInIter> ep=boost::in_edges(v,m_g); ep.first!=ep.second; ep.first++)
            {
               Edge e = *ep.first;
               Vertex v_source = boost::source(e, m_g);
               double ds_me = boost::get(m_ds,v_source) + boost::get(m_w,e);
               printf("  option from vertex %u, d=%f (%f + %f)\n",
                  v_source, ds_me, boost::get(m_ds,v_source), boost::get(m_w,e));
               if (ds_me < ds_best)
               {
                  ds_best = ds_me;
                  e_best = e;
               }
            }
            epath.push_front(e_best);
            v = boost::source(e_best, m_g);
         }
         // walk to goal
         v = boost::vertex(m_queue_conn.top_idx(), m_g);
         printf("getting path to goal ...\n");
         while (boost::get(m_dg, v))
         {
            printf("front vertex: %u, with d-value %f\n", boost::get(m_vimap,v), boost::get(m_dg,v));
            double dg_best = std::numeric_limits<double>::infinity();
            Edge e_best;
            for (std::pair<EdgeOutIter,EdgeOutIter> ep=boost::out_edges(v,m_g); ep.first!=ep.second; ep.first++)
            {
               Edge e = *ep.first;
               Vertex v_target = boost::target(e, m_g);
               double dg_me = boost::get(m_w, e) + boost::get(m_dg,v_target);
               printf("  option from vertex %u, d=%f (%f + %f)\n",
                  v_target, dg_me, boost::get(m_dg,v_target), boost::get(m_w,e));
               if (dg_me < dg_best)
               {
                  dg_best = dg_me;
                  e_best = e;
               }
            }
            epath.push_back(e_best);
            v = boost::target(e_best, m_g);
         }
         printf("done getting path!\n");
      }
   }
};

} // namespace pr_bgl
