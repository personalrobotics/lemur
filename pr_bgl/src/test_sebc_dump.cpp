#include <cstdio>
#include <list>
#include <map>
#include <vector>
#include <stdexcept>

#include <boost/function.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/reverse_graph.hpp>

#include <pr_bgl/compose_property_map.hpp>
#include <pr_bgl/rev_edge_map.h>
#include <pr_bgl/soft_edge_bc.h>

typedef boost::adjacency_list<
   boost::vecS, // Edgelist ds, for per-vertex out-edges
   boost::vecS, // VertexList ds, for vertex set
   boost::undirectedS // type of graph
   > Graph;
typedef boost::graph_traits<Graph> GraphTypes;
typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
typedef boost::graph_traits<Graph>::vertex_iterator VertexIter;
typedef boost::graph_traits<Graph>::edge_descriptor Edge;
typedef boost::graph_traits<Graph>::edge_iterator EdgeIter;
typedef boost::graph_traits<Graph>::out_edge_iterator EdgeOutIter;
typedef boost::graph_traits<Graph>::in_edge_iterator EdgeInIter;

double ratio_limit = 1.0;
double shortest_length = 0.0;

double f_linear(double len)
{
   if (ratio_limit == 1.0)
   {
      if (len == shortest_length)
         return 1.0;
      else
         return 0.0;
   }
   double ratio = len / shortest_length;
   return 1.0 - (ratio - 1.0) / (ratio_limit - 1.0);
}

int main(int argc, char *argv[])
{
   int err;
   FILE * fp;
   
   printf("starting test_sebc_dump!\n");

   if (argc != 4)
   {
      printf("Usage: test_sebc_dump <graph.txt> <ratio-limit> <sebc-score.txt>\n");
      return 1;
   }
   
   ratio_limit = atof(argv[2]);
   printf("got ratio_limit: %f\n", ratio_limit);
   
   // create graph
   Graph g;
   Vertex v_start = GraphTypes::null_vertex();
   Vertex v_goal = GraphTypes::null_vertex();
   std::map<Edge,unsigned int> eidxs;
   std::map<Edge,double> w_x;
   std::map<Edge,double> w_xhat;
   std::map<Edge,double> w_phat;
   
   fp = fopen(argv[1],"r");
   if (!fp)
   {
      printf("couldnt load graph from file!\n");
      return 1;
   }
   
   {
      char * line = 0;
      size_t len = 0;
      ssize_t read;
      // get vertices
      fseek(fp, 0, SEEK_SET);
      unsigned int vidx = 0;
      while ((read = getline(&line, &len, fp)) != -1)
      {
         if (0 != strncmp(line,"vertex ",7))
            continue;
         int ret;
         unsigned int line_vidx;
         ret = sscanf(line, "vertex %u at ", &line_vidx);
         if (ret != 1 || line_vidx != vidx)
         {
            printf("vertex scan failed, ret=%d, line_vidx=%u, vidx=%u!\n",ret,line_vidx,vidx);
            return 1;
         }
         Vertex v = boost::add_vertex(g);
         vidx++;
      }
      // get start/goal vertices
      fseek(fp, 0, SEEK_SET);
      while ((read = getline(&line, &len, fp)) != -1)
      {
         int ret;
         unsigned int line_vidx;
         ret = sscanf(line, "vertex_start %u", &line_vidx);
         if (ret == 1)
            v_start = boost::vertex(line_vidx, g);
         ret = sscanf(line, "vertex_goal %u", &line_vidx);
         if (ret == 1)
            v_goal = boost::vertex(line_vidx, g);
      }
      // get edges
      // edge 0 source 0 target 1 w_x 20.0 w_xhat 20.0 w_phat 20.0
      fseek(fp, 0, SEEK_SET);
      unsigned int eidx = 0;
      while ((read = getline(&line, &len, fp)) != -1)
      {
         if (0 != strncmp(line,"edge ",5))
            continue;
         int ret;
         unsigned int line_eidx;
         unsigned int line_source;
         unsigned int line_target;
         double line_w_x;
         double line_w_xhat;
         double line_w_phat;
         ret = sscanf(line, "edge %u source %u target %u w_x %lf w_xhat %lf w_phat %lf",
            &line_eidx, &line_source, &line_target, &line_w_x, &line_w_xhat, &line_w_phat);
         if (ret != 6 || line_eidx != eidx)
         {
            printf("edge scan failed, ret=%d, line_eidx=%u, eidx=%u!\n",ret,line_eidx,eidx);
            return 1;
         }
         Edge e = boost::add_edge(line_source, line_target, g).first;
         eidxs[e] = line_eidx;
         w_x[e] = line_w_x;
         w_xhat[e] = line_w_xhat;
         w_phat[e] = line_w_phat;
         eidx++;
      }
   }
   fclose(fp);
   if (v_start == GraphTypes::null_vertex()
      || v_goal == GraphTypes::null_vertex())
   {
      printf("start and/or goal vertices not found!\n");
      return 1;
   }

   // run reversed dijkstra's to get goaldist (and preds) from w_x
   printf("running reverse dijstra's ...\n");
   std::map<Vertex,double> goaldist;
   std::map<Vertex,Vertex> preds;
   boost::reverse_graph<Graph> rg(g);
   boost::dijkstra_shortest_paths(
      rg,
      v_goal, // source (actually dest of non-reversed graph)
      boost::make_assoc_property_map(preds),
      boost::make_assoc_property_map(goaldist),
      pr_bgl::make_compose_property_map(
         boost::make_assoc_property_map(w_x),
         pr_bgl::RevEdgeMap<Graph>(rg)),
      boost::get(boost::vertex_index, g),
      std::less<double>(), // compare
      boost::closed_plus<double>(std::numeric_limits<double>::max()), // combine
      std::numeric_limits<double>::max(),
      double(),
      boost::make_dijkstra_visitor(boost::null_visitor())
      );
   
   shortest_length = goaldist[v_start];
   printf("best length to goal: %f\n", shortest_length);
   
   // compute sebc scores
   printf("computing soft edge betweenness centrality ...\n");
   std::map<Edge,double> scores;
   pr_bgl::soft_edge_bc(g, v_start, v_goal,
      f_linear,
      //std::numeric_limits<double>::max(),
      ratio_limit*shortest_length,
      boost::make_assoc_property_map(w_x),
      boost::make_assoc_property_map(goaldist),
      boost::make_assoc_property_map(scores));
   
   // output result
   fp = fopen(argv[3],"w");
   if (!fp)
   {
      printf("couldnt load graph from file!\n");
      return 1;
   }
   for (std::pair<EdgeIter,EdgeIter> ep=boost::edges(g); ep.first!=ep.second; ep.first++)
   {
      Edge e = *ep.first;
      fprintf(fp,"edge_sebc_score %u %f\n", eidxs[e], scores[e]);
   }
   fclose(fp);

   return 0;
}
