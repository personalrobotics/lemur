/*! \file path_generator.h
 * \author Chris Dellin <cdellin@gmail.com>
 * \copyright 2015 Carnegie Mellon University
 * \copyright License: BSD
 * 
 * \brief Generates all simple paths in non-decreasing order of length
 *        (pr_bgl::path_generator).
 */

/* requires:
#include <map>
#include <vector>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/reverse_graph.hpp>
#include <pr_bgl/compose_property_map.hpp>
#include <pr_bgl/rev_edge_map.h>
#include <pr_bgl/heap_indexed.h>
*/

namespace pr_bgl
{

/*! \brief Generates all simple paths in non-decreasing order of length.
 */
template <class Graph, class WeightMap>
class path_generator
{
private:
   typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
   typedef typename boost::graph_traits<Graph>::edge_descriptor Edge;
   typedef typename boost::graph_traits<Graph>::out_edge_iterator EdgeOutIter;
   typedef std::pair<double, double> Key; // (g+h, h), (path-sofar)
public:
   typedef std::pair<double, std::vector<Vertex> > Path;
   
private:
   // inputs
   const Graph & g;
   const Vertex v_start;
   const Vertex v_goal;
   const WeightMap weight_map;
   
   /*! \brief state */
   std::map<Vertex,double> goaldist;
   /*! \brief this is append-only, both the open and close lists */
   std::vector< Path > paths;
   /*! \brief this indexes into paths, is the open list */
   pr_bgl::heap_indexed< Key > queue;
   
   /*! \brief 1-size buffer, for peeking */
   Path next_path_buffer;
      
public:
   path_generator(const Graph & g, Vertex v_start, Vertex v_goal, WeightMap weight_map):
      g(g), v_start(v_start), v_goal(v_goal), weight_map(weight_map)
   {
      // find optimal actual path cost
      // run reversed dijkstra's to get goaldist (and preds) from w
      printf("running w_est reverse dijstra's ...\n");
      std::map<Vertex,Vertex> preds;
      boost::reverse_graph<Graph> rg(g);
      boost::dijkstra_shortest_paths(
         rg,
         v_goal, // source (actually dest of non-reversed graph)
         boost::make_assoc_property_map(preds),
         boost::make_assoc_property_map(goaldist),
         pr_bgl::make_compose_property_map(
            weight_map,
            pr_bgl::rev_edge_map<Graph>(rg)),
         boost::get(boost::vertex_index, g),
         std::less<double>(), // compare
         boost::closed_plus<double>(std::numeric_limits<double>::max()), // combine
         std::numeric_limits<double>::max(),
         double(),
         boost::make_dijkstra_visitor(boost::null_visitor())
      );
      // start with just the start vertex
      paths.push_back(Path(0.0, std::vector<Vertex>(1,v_start)));
      queue.insert(0, Key(goaldist[v_start],goaldist[v_start]));
      // keep buffer
      next_path_buffer = generate_path();
   }
   
   bool peek_next_exists()
   {
      return (0 < next_path_buffer.second.size());
   }
   
   double peek_length()
   {
      return next_path_buffer.first;
   }
   
   Path next_path()
   {
      Path ret = next_path_buffer;
      next_path_buffer = generate_path();
      return ret;
   }
   
private:
   /*! \brief returns an empty path if no more exist! */
   Path generate_path()
   {
      unsigned int ui;
      while (queue.size())
      {
         // remove top element from queue
         size_t top_idx = queue.top_idx();
         queue.remove_min();
         Path path = paths[top_idx];
         // is it a full path to the goal?
         if (path.second.back() == v_goal)
            return path;
         // iterate over all successors of the last vertex
         std::pair<EdgeOutIter, EdgeOutIter> out_edges
            = boost::out_edges(path.second.back(), g);
         for (; out_edges.first!=out_edges.second; out_edges.first++)
         {
            Vertex v_successor = boost::target(*out_edges.first, g);
            // ignore non-simple paths
            for (ui=0; ui<path.second.size(); ui++)
               if (path.second[ui] == v_successor)
                  break;
            if (ui<path.second.size())
               continue;
            // add new path to paths
            Path path_new = path;
            Edge e = *out_edges.first;
            path_new.first += boost::get(weight_map, e);
            path_new.second.push_back(v_successor);
            paths.push_back(path_new);
            queue.insert(paths.size()-1,
               Key(path_new.first+goaldist[v_successor],goaldist[v_successor]));
         }
      }
      // no path!
      return Path(std::numeric_limits<double>::max(), std::vector<Vertex>());
   }
};

} // namespace pr_bgl
