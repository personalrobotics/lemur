/* File: test_incbi.cpp
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

#include <cstdio>
#include <list>
#include <map>
#include <vector>
#include <stdexcept>

#include <boost/property_map/property_map.hpp>
#include <boost/graph/adjacency_list.hpp>

#include <pr_bgl/heap_indexed.h>
#include <pr_bgl/incbi.h>

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


typedef pr_bgl::IncBi<
   Graph,
   boost::associative_property_map< std::map<Edge,double> >,
   boost::property_map<Graph, boost::vertex_index_t>::type
   > GraphIncBi;



class DumpVisitor
{
public:
   unsigned int * m_p_dump_num;
   char * m_dump_format;
   std::vector<unsigned int> * m_p_eidxs_evaled;
   DumpVisitor(unsigned int * p_dump_num, char * dump_format, std::vector<unsigned int> * p_eidxs_evaled):
      m_p_dump_num(p_dump_num), m_dump_format(dump_format), m_p_eidxs_evaled(p_eidxs_evaled)
   {}
   void dump(GraphIncBi & incbi)
   {
      std::list<Edge> path;
      dump_with_path(incbi, path, 0);
   }
   void dump_with_path(GraphIncBi & incbi, std::list<Edge> & path, Edge * ep)
   {
      FILE * fp;
      char buf[256];
      sprintf(buf, m_dump_format, *m_p_dump_num);
      fp = fopen(buf,"w");
      for (unsigned int i=0; i<boost::num_vertices(incbi.m_g); i++)
      {
         // start state
         if (incbi.m_queue_start.contains(i))
            fprintf(fp,"start_open %u\n", i);
         else if (incbi.m_ds_backing[i] != std::numeric_limits<double>::infinity())
            fprintf(fp,"start_closed %u\n", i);
         // goal state
         if (incbi.m_queue_goal.contains(i))
            fprintf(fp,"goal_open %u\n", i);
         else if (incbi.m_dg_backing[i] != std::numeric_limits<double>::infinity())
            fprintf(fp,"goal_closed %u\n", i);
         // conn state
         if (incbi.m_queue_conn.contains(i))
            fprintf(fp,"conn %u\n", i);
         // incbi values, ds rs dg rg
         fprintf(fp,"incbi_values %u %f %f %f %f\n", i,
            incbi.m_ds_backing[i], incbi.m_rs_backing[i],
            incbi.m_dg_backing[i], incbi.m_rg_backing[i]);
      }
      // evaled edges
      for (unsigned int ui=0; ui<(*m_p_eidxs_evaled).size(); ui++)
         fprintf(fp,"edge_evaled %u\n", (*m_p_eidxs_evaled)[ui]);
      // candidate path
      if (path.size())
      {
         fprintf(fp,"candidate_path %u", boost::get(incbi.m_vimap, boost::source(path.front(),incbi.m_g)));
         for (std::list<Edge>::iterator it=path.begin(); it!=path.end(); ++it)
            fprintf(fp," %u", boost::get(incbi.m_vimap, boost::target(*it,incbi.m_g)));
         fprintf(fp,"\n");
      }
      if (ep)
      {
         fprintf(fp,"edge_to_eval %u %u\n",
            boost::get(incbi.m_vimap,boost::source(*ep, incbi.m_g)),
            boost::get(incbi.m_vimap,boost::target(*ep, incbi.m_g)));
      }
      fclose(fp);
      (*m_p_dump_num)++;
   }
};

int main(int argc, char * argv[])
{
   int err;
   
   printf("incbi started!\n");
   
   if (argc != 3)
   {
      printf("Usage: test_incbi <graph.txt> <dump-format-w-%%u.txt>\n");
      return 1;
   }
   
   // create graph
   Graph g;
   Vertex v_start = GraphTypes::null_vertex();
   Vertex v_goal = GraphTypes::null_vertex();
   std::map<Edge,double> w;
   boost::associative_property_map< std::map<Edge,double> > map_w = boost::make_assoc_property_map(w);
   std::map<Edge,unsigned int> eidxs;
   std::map<Edge,bool> evaled;
   std::map<Edge,double> w_x;
   std::map<Edge,double> w_xhat;
   std::map<Edge,double> w_phat;
   std::vector<unsigned int> eidxs_evaled;
   
   FILE * fp;
   fp = fopen(argv[1],"r");
   if (!fp)
   {
      printf("couldnt load graph from file!\n");
      return 1;
   }
   
   {
      char * line = NULL;
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
         w[e] = line_w_xhat + line_w_phat;
         evaled[e] = false;
         eidx++;
      }
   }
   
   fclose(fp);
   
   if (v_start == GraphTypes::null_vertex() || v_goal == GraphTypes::null_vertex())
   {
      printf("start and/or goal vertices not found!\n");
      return 1;
   }
   
   
   // start incremental searches!
   unsigned int dump_num = 0;
   

   GraphIncBi incbi(g, v_start, v_goal, map_w, boost::get(boost::vertex_index,g));
   
   DumpVisitor visitor(&dump_num, argv[2], &eidxs_evaled);
   
   for (;;)
   {
      printf("running incremental bidirectional search!\n");
   
      std::list<Edge> epath;
      incbi.search(1.0, 1.0, visitor, epath);

      // dump path
      visitor.dump_with_path(incbi, epath, 0);
      
      printf("getting edges to eval ...\n");
      // evaluate FIRST and LAST unevaled edge!
      std::vector<Edge> edges_to_eval;
      for (std::list<Edge>::iterator it=epath.begin(); it!=epath.end(); ++it)
      {
         Edge e = *it;
         if (evaled[e])
            continue;
         edges_to_eval.push_back(e);
         break;
      }
      for (std::list<Edge>::reverse_iterator it=epath.rbegin(); it!=epath.rend(); ++it)
      {
         Edge e = *it;
         if (evaled[e])
            continue;
         if (e != edges_to_eval.back())
            edges_to_eval.push_back(e);
         break;
      }
      if (!edges_to_eval.size())
      {
         printf("totally done!\n");
         break;
      }
      
      printf("evaluating ...\n");
      for (std::vector<Edge>::iterator it=edges_to_eval.begin(); it!=edges_to_eval.end(); ++it)
      {
         Edge e = *it;
         
         // dump to_eval path
         visitor.dump_with_path(incbi, epath, &e);
         
         // evaluate
         evaled[e] = true;
         eidxs_evaled.push_back(eidxs[e]);
         printf("weight adjusted from %f", w[e]);
         w[e] = w_x[e];
         printf(" to %f!\n", w[e]);
      }
      
      incbi.notify_changed_edges(edges_to_eval.begin(), edges_to_eval.end());
      
   }
   
   return 0;
}
