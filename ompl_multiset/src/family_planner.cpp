/* File: lazy_prm.cpp
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

#include <fstream>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

#include <ompl/base/Planner.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/goals/GoalState.h>

// TEMP for stringify
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <pr_bgl/graph_io.h>
#include <pr_bgl/string_map.h>
#include <pr_bgl/edge_indexed_graph.h>
#include <pr_bgl/overlay_manager.h>
#include <pr_bgl/lazysp.h>

#include <ompl_multiset/BisectPerm.h>
#include <ompl_multiset/RoadmapGen.h>
#include <ompl_multiset/family_planner.h>


namespace ompl_multiset
{

inline void stringify_to_x(const std::string & in, ompl_multiset::StateConPtr & repr)
{
   repr.reset();
   //repr = atof(in.c_str());
}
inline void stringify_from_x(std::string & repr, const ompl_multiset::StateConPtr & in)
{
   unsigned int dim = in->space->getDimension();
   ompl::base::RealVectorStateSpace::StateType * state
      = in->state->as<ompl::base::RealVectorStateSpace::StateType>();
   repr.clear();
   for (unsigned int ui=0; ui<dim; ui++)
   {
      if (ui)
         repr += " ";
      std::string component_repr;
      pr_bgl::stringify_from_x(component_repr, (double)state->values[ui]);
      repr += component_repr;
   }
}

} // namespace multiset


namespace {

template <class VertexIndexMap, class EdgeIndexMap>
class LazySPLogVisitor
{
public:
   
   VertexIndexMap vertex_index_map;
   EdgeIndexMap edge_index_map;
   std::ostream & stream;
   LazySPLogVisitor(
         VertexIndexMap vertex_index_map,
         EdgeIndexMap edge_index_map,
         std::ostream & stream):
      vertex_index_map(vertex_index_map),
      edge_index_map(edge_index_map),
      stream(stream)
   {
   }
   
   template <class Vertex>
   void lazy_path(double length, std::vector<Vertex> & vpath)
   {
      stream << "lazy_path_length " << length << std::endl;
      stream << "lazy_path";
      for (unsigned int ui=0; ui<vpath.size(); ui++)
         stream << " " << get(vertex_index_map,vpath[ui]);
      stream << std::endl;
   }

   void no_path()
   {
      stream << "no_path" << std::endl;
   }

   void path_found()
   {
      stream << "path_found" << std::endl;
   }
   
   template <class Edge>
   void edge_evaluate(Edge & e)
   {
      stream << "eval_edge " << get(edge_index_map,e) << std::endl;
   }
};
template <class VertexIndexMap, class EdgeIndexMap>
LazySPLogVisitor<VertexIndexMap,EdgeIndexMap>
make_lazysp_log_visitor(
   VertexIndexMap vertex_index_map, EdgeIndexMap edge_index_map, std::ostream & stream)
{
   return LazySPLogVisitor<VertexIndexMap,EdgeIndexMap>(
      vertex_index_map, edge_index_map, stream);
}

ompl::base::SpaceInformationPtr
get_bogus_si(const ompl::base::StateSpacePtr space)
{
   ompl::base::SpaceInformationPtr
      si(new ompl::base::SpaceInformation(space));
   ompl::base::StateValidityCheckerPtr
      si_checker(new ompl::base::AllValidStateValidityChecker(si));
   si->setStateValidityChecker(si_checker);
   si->setup();
   return si;
}

} // anonymous namespace


ompl_multiset::FamilyPlanner::FamilyPlanner(
      const ompl::base::StateSpacePtr space,
      const RoadmapGenPtr roadmap_gen,
      std::ostream & os_graph, std::ostream & os_alglog):
   ompl::base::Planner(get_bogus_si(space), "FamilyPlanner"),
   space(space),
   check_radius(0.5*space->getLongestValidSegmentLength()),
   os_graph(os_graph), os_alglog(os_alglog),
   eig(g, get(&EProps::index,g)),
   overlay_manager(eig,og,
      get(&OverVProps::core_vertex, og),
      get(&OverEProps::core_edge, og))
{
   // before we start,
   // generate one level into our graph
   // generate a graph
   // note that new vertices/edges get properties from constructor
   roadmap_gen->generate(eig, 1,
      get(&VProps::state, g),
      get(&EProps::distance, g),
      get(&VProps::subgraph, g),
      get(&EProps::subgraph, g),
      get(&VProps::is_shadow, g));
   
   // initialize stuff
   EdgeIter ei, ei_end;
   for (boost::tie(ei,ei_end)=edges(g); ei!=ei_end; ++ei)
      edge_init_points(*ei);
}

ompl_multiset::FamilyPlanner::~FamilyPlanner()
{
}

void ompl_multiset::FamilyPlanner::setProblemDefinition(
   const ompl::base::ProblemDefinitionPtr & pdef)
{
   // check that state matches?
   
   // set my current space information
   si_ = pdef->getSpaceInformation();
   
   // call planner base class implementation
   // this will set my pdef_ and update my pis_
   ompl::base::Planner::setProblemDefinition(pdef);
   
   // add start to overlay graph
   assert(pdef->getStartStateCount() == 1);
   ov_start = add_vertex(og);
   og[ov_start].core_vertex = boost::graph_traits<Graph>::null_vertex();
   // set state
   og[ov_start].state.reset(new StateCon(space.get()));
   space->copyState(og[ov_start].state->state, pdef->getStartState(0));
   // regular vertex properties
   og[ov_start].subgraph = 0;
   og[ov_start].is_shadow = false;

   // add goal to overlay graph
   ompl::base::GoalPtr goal = pdef->getGoal();
   assert(goal->hasType(ompl::base::GOAL_STATE));
   ompl::base::GoalState * goal_state = goal->as<ompl::base::GoalState>();
   ov_goal = add_vertex(og);
   og[ov_goal].core_vertex = boost::graph_traits<Graph>::null_vertex();
   // set state
   og[ov_goal].state.reset(new StateCon(space.get()));
   space->copyState(og[ov_goal].state->state, goal_state->getState());
   // regular vertex properties
   og[ov_goal].subgraph = 0;
   og[ov_goal].is_shadow = false;
   
   // connect to vertices within fixed radius in roadmap
   std::vector<OverVertex> ovs;
   ovs.push_back(ov_start);
   ovs.push_back(ov_goal);
   for (std::vector<OverVertex>::iterator it=ovs.begin(); it!=ovs.end(); it++)
   {
      VertexIter vi, vi_end;
      for (boost::tie(vi,vi_end)=vertices(g); vi!=vi_end; ++vi)
      {
         double dist = space->distance(
            og[*it].state->state,
            g[*vi].state->state);
         if (0.12 < dist)
            continue;
         
         // add new anchor overlay vertex
         OverVertex v_anchor = add_vertex(og);
         og[v_anchor].core_vertex = *vi;
         // no need to set core properties (e.g. state) on anchors,
         // since this is just an anchor, wont be copied

         // add overlay edge from root to anchor
         OverEdge e = add_edge(*it, v_anchor, og).first;
         // add edge properties
         // og[e].core_properties.index -- needs to be set on apply
         og[e].distance = dist;
         og[e].subgraph = 0;
      }
   }
   
   overlay_manager.apply();
   
   // manually copy over properties
   for (unsigned int ui=0; ui<overlay_manager.applied_vertices.size(); ui++)
   {
      OverVertex vover = overlay_manager.applied_vertices[ui];
      Vertex vcore = og[vover].core_vertex;
      g[vcore].state = og[vover].state;
      g[vcore].subgraph = og[vover].subgraph;
      g[vcore].is_shadow = og[vover].is_shadow;
      g[vcore].status = og[vover].status;
   }
   for (unsigned int ui=0; ui<overlay_manager.applied_edges.size(); ui++)
   {
      OverEdge eover = overlay_manager.applied_edges[ui];
      Edge ecore = og[eover].core_edge;
      g[ecore].distance = og[eover].distance;
      g[ecore].subgraph = og[eover].subgraph;
      g[ecore].w_lazy = og[eover].w_lazy;
      g[ecore].is_evaled = og[eover].is_evaled;
      g[ecore].points = og[eover].points;
      edge_init_points(ecore);
   }
}

ompl::base::PlannerStatus
ompl_multiset::FamilyPlanner::solve(
   const ompl::base::PlannerTerminationCondition & ptc)
{
   // ok, do some sweet sweet lazy search!
   
   // for now, lets do 0 weight on planning cost
   
   // set everything as unknogn to start
   VertexIter vi, vi_end;
   for (boost::tie(vi,vi_end)=vertices(g); vi!=vi_end; ++vi)
   {
      g[*vi].status = 0;
   }
   EdgeIter ei, ei_end;
   for (boost::tie(ei,ei_end)=edges(g); ei!=ei_end; ++ei)
   {
      g[*ei].w_lazy = g[*ei].distance;
      g[*ei].is_evaled = false;
      for (unsigned int ui=0; ui<g[*ei].points.size(); ui++)
         g[*ei].points[ui].status = 0;
   }
   
   // dump graph
   // write it out to file
   pr_bgl::GraphIO<Graph, VertexIndexMap, EdgeIndexMap, EdgeVectorMap>
      io(g,
         get(boost::vertex_index, g),
         get(&EProps::index, g),
         eig.edge_vector_map);
   io.add_property_map("state", pr_bgl::make_string_map(get(&VProps::state,g)));
   io.add_property_map("subgraph", pr_bgl::make_string_map(get(&VProps::subgraph,g)));
   io.add_property_map("is_shadow", pr_bgl::make_string_map(get(&VProps::is_shadow,g)));
   io.add_property_map("subgraph", pr_bgl::make_string_map(get(&EProps::subgraph,g)));
   io.add_property_map("distance", pr_bgl::make_string_map(get(&EProps::distance,g)));
   io.dump_graph(os_graph);
   io.dump_properties(os_graph);
   
   pr_bgl::lazy_shortest_path(g,
      og[ov_start].core_vertex,
      og[ov_goal].core_vertex,
      ompl_multiset::WMap(*this),
      get(&EProps::w_lazy,g),
      get(&EProps::is_evaled,g),
      pr_bgl::LazySpEvalFwd(),
      make_lazysp_log_visitor(
         get(boost::vertex_index, g),
         get(&EProps::index, g),
         os_alglog));
   
   return ompl::base::PlannerStatus::EXACT_SOLUTION;
}

void ompl_multiset::FamilyPlanner::edge_init_points(const Edge & e)
{
   // now many interior points do we need?
   Vertex va = source(e,g);
   Vertex vb = target(e,g);
   unsigned int n = floor(g[e].distance/(2.0*check_radius));
   // allocate states
   g[e].points.resize(n);
   for (unsigned int ui=0; ui<n; ui++)
      g[e].points[ui].state.reset(new StateCon(space.get()));
   // fill with interpolated states in bisection order
   const std::vector< std::pair<int,int> > & order = bisect_perm.get(n);
   for (unsigned int ui=0; ui<n; ui++)
      space->interpolate(g[va].state->state, g[vb].state->state,
         1.0*(1+order[ui].first)/(n+1),
         g[e].points[ui].state->state);
}

double ompl_multiset::FamilyPlanner::wmap_get(const Edge & e)
{
   // check all vertices!   
   Vertex va = source(e, g);
   Vertex vb = target(e, g);
   // check endpoints first
   if (g[va].status == 0)
   {
      bool valid = si_->isValid(g[va].state->state);
      g[va].status = valid ? 1 : 2;
      if (!valid) return std::numeric_limits<double>::max();
   }
   if (g[vb].status == 0)
   {
      bool valid = si_->isValid(g[vb].state->state);
      g[vb].status = valid ? 1 : 2;
      if (!valid) return std::numeric_limits<double>::max();
   }
   for (unsigned int ui=0; ui<g[e].points.size(); ui++)
   {
      bool valid = si_->isValid(g[e].points[ui].state->state);
      g[e].points[ui].status = valid ? 1 : 2;
      if (!valid) return std::numeric_limits<double>::max();
   }
   return g[e].distance;
}
