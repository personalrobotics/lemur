/* File: lpastar.cpp
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

#include <cstdio>
#include <map>
#include <string>
#include <sstream>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphml.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

#include <pr_bgl/heap_indexed.h>
#include <pr_bgl/lpastar.h>

#include <gtest/gtest.h>

#define XSTR(s) STR(s)
#define STR(s) # s

struct grid_reader
{
   unsigned int nrows, ncols;
   unsigned int istart, jstart;
   unsigned int igoal, jgoal;
   std::vector< std::vector<bool> > rows_original;
   std::vector< std::vector<bool> > rows_changed;
   grid_reader(const char * filename)
   {
      std::ifstream fp;
      std::string line;
      int ret;
      int n;
      unsigned int irow, icol;
      fp.open(filename);
      std::getline(fp, line);
      ret = sscanf(line.c_str(), "%ux%u%n", &nrows, &ncols, &n);
      if (ret != 2 || n != line.size())
         throw std::runtime_error("graph file parse error");
      std::getline(fp, line);
      ret = sscanf(line.c_str(), "start %u,%u%n", &istart, &jstart, &n);
      if (ret != 2 || n != line.size())
         throw std::runtime_error("graph file parse error");
      std::getline(fp, line);
      ret = sscanf(line.c_str(), "goal %u,%u%n", &igoal, &jgoal, &n);
      if (ret != 2 || n != line.size())
         throw std::runtime_error("graph file parse error");
      rows_original.resize(nrows, std::vector<bool>(ncols));
      std::getline(fp, line);
      if (line != "original")
         throw std::runtime_error("graph file parse error");
      for (irow=0; irow<nrows; irow++)
      {
         if (!std::getline(fp, line) || line.size() != ncols)
            throw std::runtime_error("graph file parse error");
         for (icol=0; icol<ncols; icol++)
         switch (line[icol])
         {
         case '.': rows_original[irow][icol] = true; break;
         case '#': rows_original[irow][icol] = false; break;
         default: std::runtime_error("graph file parse error");
         }
      }
      rows_changed.resize(nrows, std::vector<bool>(ncols));
      std::getline(fp, line);
      if (line != "changed")
         throw std::runtime_error("graph file parse error");
      for (irow=0; irow<nrows; irow++)
      {
         if (!std::getline(fp, line) || line.size() != ncols)
            throw std::runtime_error("graph file parse error");
         for (icol=0; icol<ncols; icol++)
         switch (line[icol])
         {
         case '.': rows_changed[irow][icol] = true; break;
         case '#': rows_changed[irow][icol] = false; break;
         default: std::runtime_error("graph file parse error");
         }
      }
   }
};

template <class Graph, class CoordMap>
struct grid_heuristic
{
   typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
   const Graph & g;
   const CoordMap coord_map;
   const int igoal, jgoal;
   grid_heuristic(const Graph & g, Vertex v_goal, CoordMap coord_map):
      g(g), coord_map(coord_map),
      igoal(get(coord_map, v_goal).first),
      jgoal(get(coord_map, v_goal).second)
   {
   }
   unsigned int operator()(Vertex v)
   {
      int iv = get(coord_map, v).first;
      int jv = get(coord_map, v).second;
      return std::max(std::abs(iv - igoal), std::abs(jv - jgoal));
   }
};

template <class Graph>
class counting_visitor
{
public:
   typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
   typedef typename boost::graph_traits<Graph>::edge_descriptor Edge;
   std::set<Vertex> & vs_expanded;
   int & num_expansions;
   counting_visitor(std::set<Vertex> & vs_expanded, int & num_expansions):
      vs_expanded(vs_expanded), num_expansions(num_expansions)
   {
   }
   inline void examine_vertex(Vertex u, const Graph & g)
   {
      vs_expanded.insert(u);
      num_expansions++;
   }
};

template<class Vertex>
void print_gridworld(
   std::vector< std::vector<bool> > & rows,
   std::map<std::pair<unsigned int,unsigned int>, Vertex> & coord_to_v,
   std::map<Vertex,unsigned int> & v_gvalues,
   std::set<Vertex> & vs_expanded)
{
   unsigned int nrows = rows.size();
   unsigned int ncols = rows[0].size();
   printf("+");
   for (unsigned int icol=0; icol<ncols; icol++) printf("---");
   printf("-+\n");
   for (unsigned int irow=0; irow<nrows; irow++)
   {
      printf("| ");
      for (unsigned int icol=0; icol<ncols; icol++)
      {
         Vertex v = coord_to_v[std::make_pair(irow,icol)];
         bool v_free = rows[irow][icol];
         if (!v_free)
            printf("##");
         else if (v_gvalues[v] == std::numeric_limits<unsigned int>::max())
            printf("  ");
         else
            printf("%02u", v_gvalues[v]);
         if (vs_expanded.find(v) != vs_expanded.end())
            printf("*");
         else
            printf(" ");
      }
      printf("|\n");
   }
   printf("+");
   for (unsigned int icol=0; icol<ncols; icol++) printf("---");
   printf("-+\n");
}

TEST(LifelongPlanningAstarTestCase, LifelongPlanningAstarTest)
{
   grid_reader grid(
      XSTR(DATADIR) "/lpastar_fig1.txt");
   ASSERT_EQ(15, grid.nrows);
   ASSERT_EQ(20, grid.ncols);
   
   typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS> Graph;
   typedef boost::graph_traits<Graph> GraphTypes;
   typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
   typedef boost::graph_traits<Graph>::edge_descriptor Edge;
   typedef boost::graph_traits<Graph>::edge_iterator EdgeIter;
   
   Graph g;
   std::map<std::pair<unsigned int,unsigned int>, Vertex> coord_to_v;
   std::map<Vertex, std::pair<unsigned int,unsigned int> > v_coords;
   std::map<Edge, unsigned int> e_dists;
   std::map<Vertex,Vertex> v_preds;
   std::map<Vertex,unsigned int> v_gvalues;
   std::map<Vertex,unsigned int> v_rhsvalues;
   typedef boost::associative_property_map<
      std::map<Vertex, std::pair<unsigned int,unsigned int> > > MapVII;
   typedef boost::associative_property_map<std::map<Vertex,Vertex> > MapVV;
   typedef boost::associative_property_map<std::map<Vertex,unsigned int> > MapVI;
   typedef boost::associative_property_map<std::map<Edge,unsigned int> > MapEI;
   
   // add vertices
   for (unsigned int irow=0; irow<grid.nrows; irow++)
   for (unsigned int icol=0; icol<grid.ncols; icol++)
   {
      Vertex v = add_vertex(g);
      coord_to_v[std::make_pair(irow,icol)] = v;
      v_coords[v] = std::make_pair(irow,icol);
   }
   Vertex v_start = coord_to_v[std::make_pair( grid.istart,grid.jstart)];
   Vertex v_goal = coord_to_v[std::make_pair(grid.igoal,grid.jgoal)];
   
   /* add edges
    * each vertex creates these edges:
    *   o-
    *  /|\  */
   for (unsigned int irow=0; irow<grid.nrows; irow++)
   for (unsigned int icol=0; icol<grid.ncols; icol++)
   {
      Vertex v1 = coord_to_v[std::make_pair(irow,icol)];
      bool v1_free = grid.rows_original[irow][icol];
      std::vector< std::pair<unsigned int,unsigned int> > neighbors;
      // edge right
      if (icol+1 < grid.ncols)
         neighbors.push_back(std::make_pair(irow,icol+1));
      // edge down
      if (irow+1 < grid.nrows)
         neighbors.push_back(std::make_pair(irow+1,icol));
      // edge down right
      if (icol+1 < grid.ncols && irow+1 < grid.nrows)
         neighbors.push_back(std::make_pair(irow+1,icol+1));
      // edge down left
      if (0 < icol && irow+1 < grid.nrows)
         neighbors.push_back(std::make_pair(irow+1,icol-1));
      for (unsigned int ui=0; ui<neighbors.size(); ui++)
      {
         Vertex v2 = coord_to_v[neighbors[ui]];
         bool v2_free = grid.rows_original[neighbors[ui].first][neighbors[ui].second];
         Edge e = add_edge(v1, v2, g).first;
         e_dists[e] = (v1_free && v2_free) ? 1 : UINT_MAX;
      }
   }
   
   // create algorithm instance
   std::set<Vertex> vs_expanded;
   int num_expansions = 0;
   pr_bgl::lpastar<Graph,
      grid_heuristic<Graph,MapVII>,
      counting_visitor<Graph>,
      MapVV, MapVI, MapVI, MapEI,
      boost::property_map<Graph, boost::vertex_index_t>::type,
      std::less<unsigned int>, boost::closed_plus<unsigned int>,
      unsigned int, unsigned int
   > lpastar(g, v_start, v_goal,
      grid_heuristic<Graph,MapVII>(g, v_goal, MapVII(v_coords)),
      counting_visitor<Graph>(vs_expanded,num_expansions),
      MapVV(v_preds), MapVI(v_gvalues), MapVI(v_rhsvalues), MapEI(e_dists),
      get(boost::vertex_index, g), // index_map
      std::less<unsigned int>(), // compare
      boost::closed_plus<unsigned int>(std::numeric_limits<unsigned int>::max()), // combine
      std::numeric_limits<unsigned int>::max(), 0, // cost inf, zero
      0 // goal_margin
   );

   // run search
   lpastar.compute_shortest_path();
   print_gridworld(grid.rows_original, coord_to_v, v_gvalues, vs_expanded);
   printf("num_expansions: %d (%lu vertices)\n", num_expansions, vs_expanded.size());
   ASSERT_EQ(37, num_expansions);
   ASSERT_EQ(37, vs_expanded.size());
   ASSERT_EQ(13, v_gvalues[v_goal]);
   
   // ok, alert lpastar to all changed edges
   std::set<Edge> edel;
   EdgeIter ei, ei_end;
   for (boost::tie(ei,ei_end)=edges(g); ei!=ei_end; ei++)
   {
      unsigned int v1i = v_coords[source(*ei,g)].first;
      unsigned int v1j = v_coords[source(*ei,g)].second;
      bool v1_free = grid.rows_changed[v1i][v1j];
      unsigned int v2i = v_coords[target(*ei,g)].first;
      unsigned int v2j = v_coords[target(*ei,g)].second;
      bool v2_free = grid.rows_changed[v2i][v2j];
      unsigned int new_dist = (v1_free && v2_free) ? 1 : UINT_MAX;
      if (new_dist == e_dists[*ei])
         continue;
      e_dists[*ei] = new_dist;
      edel.insert(*ei);
   }
   for (std::set<Edge>::iterator it=edel.begin(); it!=edel.end(); it++)
   {
      lpastar.update_vertex(source(*it,g));
      lpastar.update_vertex(target(*it,g));
   }
   printf("found %lu changed edges!\n", edel.size());
   ASSERT_EQ(29, edel.size());
   
    // run search
   num_expansions = 0;
   vs_expanded.clear();
   lpastar.compute_shortest_path();
   print_gridworld(grid.rows_changed, coord_to_v, v_gvalues, vs_expanded);
   printf("num_expansions: %d (%lu vertices)\n", num_expansions, vs_expanded.size());
   ASSERT_EQ(20, num_expansions);
   ASSERT_EQ(15, vs_expanded.size());
   ASSERT_EQ(14, v_gvalues[v_goal]);
}

int main(int argc, char **argv)
{
   testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
