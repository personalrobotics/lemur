/* File: dijk.cpp
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

#include <algorithm> // for std::for_each
#include <cstdio>
#include <functional>
#include <queue>
#include <utility> // for std::pair
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <ompl/util/RandomNumbers.h>

struct VertexProperties
{
   std::vector<double> q;
};

struct EdgeProperties
{
   double dist;
};

typedef boost::adjacency_list<
   boost::vecS, // Edgelist ds, for per-vertex out-edges
   boost::vecS, // VertexList ds, for vertex set
   boost::undirectedS, // type of graph
   VertexProperties, EdgeProperties // bundled internal properties
   > Graph;

typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
typedef boost::graph_traits<Graph>::vertex_iterator VertexIter;
typedef boost::graph_traits<Graph>::edge_descriptor Edge;
typedef boost::graph_traits<Graph>::out_edge_iterator EdgeOutIter;

/* adds a vertex and edges rgg style */
Vertex rgg_add_vertex_at(Graph & g, double x, double y)
{
   Vertex v = boost::add_vertex(g);
   g[v].q.resize(2);
   g[v].q[0] = x;
   g[v].q[1] = y;
   /* iterate over all existing vertices, adding edges ... */
   for (std::pair<VertexIter,VertexIter> vp = boost::vertices(g);
      vp.first!=vp.second; vp.first++)
   {
      Vertex n = *vp.first;
      if (n == v)
         continue;
      double dist = sqrt(pow(g[n].q[0]-g[v].q[0],2.0) + pow(g[n].q[1]-g[v].q[1],2.0));
      if (0.2 < dist)
         continue;
      Edge e;
      bool success;
      boost::tie(e,success) = boost::add_edge(v, n, g);
      if (!success)
         throw "edge already exists!";
      g[e].dist = dist;
   }
   return v;
}

struct OpenType
{
   OpenType(double v_cost, Vertex v, Vertex v_parent, bool isbck=true):
      v_cost(v_cost), v(v), v_parent(v_parent), isbck(isbck) {}
   double v_cost;
   Vertex v;
   Vertex v_parent;
   bool isbck;
   bool operator < (const OpenType rhs) const
   {
     return v_cost > rhs.v_cost;
   }
};

void average(std::vector<double> & vec)
{
   double sum = 0.0;
   for (unsigned int i=0; i<vec.size(); i++)
      sum += vec[i];
   printf("%.9f\n", sum/vec.size());
}

int main()
{
   printf("starting checkmask main ...\n");
   
   printf("constructing an rgg ...\n");
   Graph g;
   Vertex v_start = rgg_add_vertex_at(g, 0.14644660940672627, 0.14644660940672627);
   Vertex v_goal = rgg_add_vertex_at(g, 0.8535533905932737, 0.8535533905932737);
   ompl::RNG::setSeed(1);
   ompl::RNG rng;
   for (int i=0; i<100; i++)
   {
      double x = rng.uniform01();
      double y = rng.uniform01();
      rgg_add_vertex_at(g, x, y);
   }
   
   printf("running sweet sweet dijkstra's!\n");
   std::vector<double> times_boost_uni;
   while (times_boost_uni.size() < 1000)
   {
      struct timespec tic, toc;
      clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &tic);
      std::map<Vertex, double> distmap;
      std::map<Vertex, Vertex> predmap;
      boost::dijkstra_shortest_paths(g, v_start,
         boost::weight_map(boost::get(&EdgeProperties::dist,g))
         .distance_map(boost::make_assoc_property_map(distmap))
         .predecessor_map(boost::make_assoc_property_map(predmap))
         );
      clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &toc);
      times_boost_uni.push_back((toc.tv_sec-tic.tv_sec) + 1.0e-9*(toc.tv_nsec-tic.tv_nsec));
#if 0
      printf("path from goal to start:\n");
      {
         Vertex v = v_goal;
         for (;;)
         {
            printf("vertex: %lu (%f,%f) cts:%f\n", v, g[v].q[0], g[v].q[1], distmap[v]);
            Vertex v_prev = predmap[v];
            if (v_prev == v)
               break;
            v = v_prev;
         }
      }
#endif
   }
   average(times_boost_uni);
   
   printf("my own implementation of unidirectional dijkstra's ...\n");
   std::vector<double> times_me_uni;
   while (times_me_uni.size() < 1000)
   {
      struct timespec tic, toc;
      clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &tic);
      std::map<Vertex, std::pair<double,Vertex> > closed;
      std::priority_queue<OpenType> open;
      open.push(OpenType(0.0, v_start, v_start));
      while (!open.empty())
      {
         // pop next
         OpenType top = open.top();
         open.pop();
         // already in closed list?
         if (closed.find(top.v) != closed.end())
            continue;
         // add to closed list!
         closed[top.v] = std::make_pair(top.v_cost, top.v_parent);
         // are we done?
         if (top.v == v_goal)
            break;
         // get all successors
         for (std::pair<EdgeOutIter,EdgeOutIter> ep = boost::out_edges(top.v, g);
            ep.first!=ep.second; ep.first++)
         {
            Edge e = *ep.first;
            Vertex v_succ = boost::target(e, g);
            if (v_succ == top.v)
               v_succ = boost::source(e, g);
            // already in closed list?
            if (closed.find(v_succ) != closed.end())
               continue;
            // add to open list
            open.push(OpenType(top.v_cost+g[e].dist, v_succ, top.v));
         }
      }
      clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &toc);
      times_me_uni.push_back((toc.tv_sec-tic.tv_sec) + 1.0e-9*(toc.tv_nsec-tic.tv_nsec));
#if 0
      printf("path from goal to start:\n");
      {
         Vertex v = v_goal;
         for (;;)
         {
            printf("vertex: %lu (%f,%f) cts:%f\n", v, g[v].q[0], g[v].q[1], closed[v].first);
            Vertex v_prev = closed[v].second;
            if (v_prev == v)
               break;
            v = v_prev;
         }
      }
#endif
   }
   average(times_me_uni);
   
   printf("doing some sweet bidirectional dijkstra's!\n");
   std::vector<double> times_me_bi;
   while (times_me_bi.size() < 1000)
   {
      struct timespec tic, toc;
      clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &tic);
      std::map<Vertex, std::pair<double,Vertex> > closed_fwd;
      std::map<Vertex, std::pair<double,Vertex> > closed_bck;
      std::priority_queue<OpenType> open;
      std::pair<bool, Vertex> connection = std::make_pair(false, v_start);
      open.push(OpenType(0.0, v_start, v_start, false));
      open.push(OpenType(0.0, v_goal, v_goal, true));
      while (!open.empty())
      {
         // pop next
         OpenType top = open.top();
         open.pop();
         // get appropriate closed list
         std::map<Vertex, std::pair<double,Vertex> > * closed_mine = &closed_fwd;
         std::map<Vertex, std::pair<double,Vertex> > * closed_other = &closed_bck;
         if (top.isbck)
         {
            closed_mine = &closed_bck;
            closed_other = &closed_fwd;
         }
         // already in closed list?
         if (closed_mine->find(top.v) != closed_mine->end())
            continue;
         // are we done?
         // (i think we should be done when we discover the first overlapping EDGE!!)
         if (closed_other->find(top.v_parent) != closed_other->end())
         {
            connection = std::make_pair(true, top.v_parent);
            break;
         }
         // add to closed list!
         (*closed_mine)[top.v] = std::make_pair(top.v_cost, top.v_parent);
         // get all successors
         for (std::pair<EdgeOutIter,EdgeOutIter> ep = boost::out_edges(top.v, g);
            ep.first!=ep.second; ep.first++)
         {
            Edge e = *ep.first;
            Vertex v_succ = boost::target(e, g);
            if (v_succ == top.v)
               v_succ = boost::source(e, g);
            // already in closed list?
            if (closed_mine->find(v_succ) != closed_mine->end())
               continue;
            // add to open list
            open.push(OpenType(top.v_cost+g[e].dist, v_succ, top.v, top.isbck));
         }
      }
      clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &toc);
      times_me_bi.push_back((toc.tv_sec-tic.tv_sec) + 1.0e-9*(toc.tv_nsec-tic.tv_nsec));
#if 0
      if (!connection.first)
      {
         printf("no connection found!\n");
         abort();
      }
      printf("path from connection to start:\n");
      {
         Vertex v = connection.second;
         for (;;)
         {
            printf("vertex: %lu (%f,%f) cts:%f\n", v, g[v].q[0], g[v].q[1], closed_fwd[v].first);
            Vertex v_prev = closed_fwd[v].second;
            if (v_prev == v)
               break;
            v = v_prev;
         }
      }
      printf("path from connection to goal:\n");
      {
         Vertex v = connection.second;
         for (;;)
         {
            printf("vertex: %lu (%f,%f) cts:%f\n", v, g[v].q[0], g[v].q[1], closed_bck[v].first);
            Vertex v_prev = closed_bck[v].second;
            if (v_prev == v)
               break;
            v = v_prev;
         }
      }
#endif
   }
   average(times_me_bi);
   
   
   
   
   return 0;
}
