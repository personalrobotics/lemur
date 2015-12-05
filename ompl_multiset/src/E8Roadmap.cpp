/* File: E8Roadmap.cpp
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

#include <fstream>

#include <boost/chrono.hpp>
#include <boost/property_map/dynamic_property_map.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/filtered_graph.hpp>

#include <ompl/base/Planner.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/datastructures/NearestNeighborsGNAT.h>

// TEMP for stringify
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <pr_bgl/compose_property_map.hpp>
#include <pr_bgl/graph_io.h>
#include <pr_bgl/string_map.h>
#include <pr_bgl/edge_indexed_graph.h>
#include <pr_bgl/overlay_manager.h>
#include <pr_bgl/lazysp.h>
#include <pr_bgl/heap_indexed.h>
#include <pr_bgl/pair_index_map.h>
#include <pr_bgl/partition_all.h>
#include <pr_bgl/lazysp_partition_all.h>
#include <pr_bgl/lazysp_sp_indicator_probability.h>

#include <ompl_multiset/aborting_space_information.h>
#include <ompl_multiset/rvstate_map_string_adaptor.h>
#include <ompl_multiset/BisectPerm.h>
#include <ompl_multiset/Roadmap.h>
#include <ompl_multiset/EffortModel.h>
#include <ompl_multiset/E8Roadmap.h>
#include <ompl_multiset/lazysp_log_visitor.h>

ompl_multiset::E8Roadmap::E8Roadmap(
      const ompl::base::StateSpacePtr & space,
      EffortModel & effort_model,
      TagCache<VIdxTagMap,EIdxTagsMap> & tag_cache,
      const RoadmapPtr roadmap_gen):
   ompl::base::Planner(
      ompl_multiset::get_aborting_space_information(space), "E8Roadmap"),
   effort_model(effort_model),
   roadmap_gen(roadmap_gen),
   space(space),
   check_radius(0.5*space->getLongestValidSegmentLength()),
   eig(g, get(&EProps::index,g)),
   overlay_manager(eig,og,
      get(&OverVProps::core_vertex, og),
      get(&OverEProps::core_edge, og)),
   tag_cache(tag_cache),
   ompl_nn(new ompl::NearestNeighborsGNAT<Vertex>),
   //nn(g, get(&VProps::state,g), space, ompl_nn.get()),
   nn(g, get(&VProps::state,g), space),
   _coeff_distance(1.),
   _coeff_checkcost(0.),
   _coeff_batch(0.),
   _do_timing(false),
   _num_batches_init(0),
   _max_batches(UINT_MAX),
   _search_type(SEARCH_TYPE_ASTAR),
   _eval_type(EVAL_TYPE_ALT),
   os_alglog(0),
   m_vidx_tag_map(pr_bgl::make_compose_property_map(get(&VProps::tag,g), get(boost::vertex_index,g))),
   m_eidx_tags_map(pr_bgl::make_compose_property_map(get(&EProps::edge_tags,g), eig.edge_vector_map))
{
   Planner::declareParam<double>("coeff_distance", this,
      &ompl_multiset::E8Roadmap::setCoeffDistance,
      &ompl_multiset::E8Roadmap::getCoeffDistance, "0.:1.:10000.");
   Planner::declareParam<double>("coeff_checkcost", this,
      &ompl_multiset::E8Roadmap::setCoeffCheckcost,
      &ompl_multiset::E8Roadmap::getCoeffCheckcost, "0.:1.:10000.");
   Planner::declareParam<double>("coeff_batch", this,
      &ompl_multiset::E8Roadmap::setCoeffBatch,
      &ompl_multiset::E8Roadmap::getCoeffBatch, "0.:1.:10000.");
   Planner::declareParam<bool>("do_timing", this,
      &ompl_multiset::E8Roadmap::setDoTiming,
      &ompl_multiset::E8Roadmap::getDoTiming);
   Planner::declareParam<unsigned int>("num_batches_init", this,
      &ompl_multiset::E8Roadmap::setNumBatchesInit,
      &ompl_multiset::E8Roadmap::getNumBatchesInit);
   Planner::declareParam<unsigned int>("max_batches", this,
      &ompl_multiset::E8Roadmap::setMaxBatches,
      &ompl_multiset::E8Roadmap::getMaxBatches);
   Planner::declareParam<std::string>("search_type", this,
      &ompl_multiset::E8Roadmap::setSearchType,
      &ompl_multiset::E8Roadmap::getSearchType);
   Planner::declareParam<std::string>("eval_type", this,
      &ompl_multiset::E8Roadmap::setEvalType,
      &ompl_multiset::E8Roadmap::getEvalType);
   
   // setup ompl_nn
   ompl_nn->setDistanceFunction(boost::bind(&ompl_multiset::E8Roadmap::ompl_nn_dist, this, _1, _2));
   
   printf("E8Roadmap: constructor finished.\n");
}

ompl_multiset::E8Roadmap::~E8Roadmap()
{
   overlay_unapply();
   
   // core roadmap
   VertexIter vi, vi_end;
   for (boost::tie(vi,vi_end)=vertices(g); vi!=vi_end; ++vi)
      space->freeState(g[*vi].state);
   EdgeIter ei, ei_end;
   for (boost::tie(ei,ei_end)=edges(g); ei!=ei_end; ++ei)
      for (unsigned int ui=0; ui<g[*ei].edge_states.size(); ui++)
         space->freeState(g[*ei].edge_states[ui]);
   
   // overlay roadmap
   OverVertexIter ovi, ovi_end;
   for (boost::tie(ovi,ovi_end)=vertices(og); ovi!=ovi_end; ++ovi)
      if (og[*ovi].state)
         space->freeState(og[*ovi].state);
   OverEdgeIter oei, oei_end;
   for (boost::tie(oei,oei_end)=edges(og); oei!=oei_end; ++oei)
      for (unsigned int ui=0; ui<og[*oei].edge_states.size(); ui++)
         space->freeState(og[*oei].edge_states[ui]);
}

void ompl_multiset::E8Roadmap::setCoeffDistance(double coeff_distance)
{
   _coeff_distance = coeff_distance;
}

double ompl_multiset::E8Roadmap::getCoeffDistance() const
{
   return _coeff_distance;
}

void ompl_multiset::E8Roadmap::setCoeffCheckcost(double coeff_checkcost)
{
   _coeff_checkcost = coeff_checkcost;
}

double ompl_multiset::E8Roadmap::getCoeffCheckcost() const
{
   return _coeff_checkcost;
}

void ompl_multiset::E8Roadmap::setCoeffBatch(double coeff_batch)
{
   _coeff_batch = coeff_batch;
}

double ompl_multiset::E8Roadmap::getCoeffBatch() const
{
   return _coeff_batch;
}

void ompl_multiset::E8Roadmap::setDoTiming(bool do_timing)
{
   _do_timing = do_timing;
}

double ompl_multiset::E8Roadmap::getDoTiming() const
{
   return _do_timing;
}

void ompl_multiset::E8Roadmap::setNumBatchesInit(unsigned int num_batches_init)
{
   _num_batches_init = num_batches_init;
}

unsigned int ompl_multiset::E8Roadmap::getNumBatchesInit() const
{
   return _num_batches_init;
}

void ompl_multiset::E8Roadmap::setMaxBatches(unsigned int max_batches)
{
   _max_batches = max_batches;
}

unsigned int ompl_multiset::E8Roadmap::getMaxBatches() const
{
   return _max_batches;
}

void ompl_multiset::E8Roadmap::setSearchType(std::string search_type)
{
   if (search_type == "dijkstras")
      _search_type = SEARCH_TYPE_DIJKSTRAS;
   else if (search_type == "astar")
      _search_type = SEARCH_TYPE_ASTAR;
   else
      OMPL_ERROR("%s: Search type parameter must be dijkstras or astar.",
         getName().c_str());
}

std::string ompl_multiset::E8Roadmap::getSearchType() const
{
   switch (_search_type)
   {
   case SEARCH_TYPE_DIJKSTRAS: return "dijkstras";
   case SEARCH_TYPE_ASTAR: return "astar";
   default:
      throw std::runtime_error("corrupted _search_type!");
   }
}

void ompl_multiset::E8Roadmap::setEvalType(std::string eval_type)
{
   if (eval_type == "fwd")
      _eval_type = EVAL_TYPE_FWD;
   else if (eval_type == "rev")
      _eval_type = EVAL_TYPE_REV;
   else if (eval_type == "alt")
      _eval_type = EVAL_TYPE_ALT;
   else if (eval_type == "bisect")
      _eval_type = EVAL_TYPE_BISECT;
   else if (eval_type == "fwd_expand")
      _eval_type = EVAL_TYPE_FWD_EXPAND;
   else if (eval_type == "partition_all")
      _eval_type = EVAL_TYPE_PARTITION_ALL;
   else if (eval_type == "sp_indicator_probability")
      _eval_type = EVAL_TYPE_SP_INDICATOR_PROBABILITY;
   else
      OMPL_ERROR("%s: Eval type parameter must be fwd rev alt bisect or fwd_expand.",
         getName().c_str());
}

std::string ompl_multiset::E8Roadmap::getEvalType() const
{
   switch (_eval_type)
   {
   case EVAL_TYPE_FWD: return "fwd";
   case EVAL_TYPE_REV: return "rev";
   case EVAL_TYPE_ALT: return "alt";
   case EVAL_TYPE_BISECT: return "bisect";
   case EVAL_TYPE_FWD_EXPAND: return "fwd_expand";
   case EVAL_TYPE_PARTITION_ALL: return "partition_all";
   case EVAL_TYPE_SP_INDICATOR_PROBABILITY: return "sp_indicator_probability";
   default:
      throw std::runtime_error("corrupted _search_type!");
   }
}

void ompl_multiset::E8Roadmap::setProblemDefinition(
   const ompl::base::ProblemDefinitionPtr & pdef)
{
   
   // call planner base class implementation
   // this will set my pdef_ and update my pis_
   ompl::base::Planner::setProblemDefinition(pdef);
   
   ompl::base::SpaceInformationPtr si_new = pdef->getSpaceInformation();
   //if (si_new != family_effort_model.si_target)
   if (effort_model.has_changed())
   {
      // route target si to family effort model
      // this will re-run reverse dijkstra's on the family graph
      //family_effort_model.set_target(si_new);
      
      // recalculate wlazy if necessary
      // we probably always need to recalc wlazy for overlay states!
      EdgeIter ei, ei_end;
      for (boost::tie(ei,ei_end)=edges(g); ei!=ei_end; ++ei)
         calculate_w_lazy(*ei);
   }
   
   overlay_unapply();
   
   // delete any states in overlay graph
   
   // clear overlay graph
   og.clear();
   
   // add starts to overlay graph
   ov_singlestart = add_vertex(og);
   ov_singlegoal = add_vertex(og);
   og[ov_singlestart].core_vertex = boost::graph_traits<Graph>::null_vertex();
   og[ov_singlegoal].core_vertex = boost::graph_traits<Graph>::null_vertex();
   og[ov_singlestart].state = 0;
   og[ov_singlestart].batch = 0;
   og[ov_singlestart].is_shadow = false;
   og[ov_singlestart].tag = 0;
   og[ov_singlegoal].state = 0;
   og[ov_singlegoal].batch = 0;
   og[ov_singlegoal].is_shadow = false;
   og[ov_singlegoal].tag = 0;
   
   // add all actual start/goal vertices here, for future connection
   std::vector<OverVertex> ovs;
   
   for (unsigned int istart=0; istart<pdef->getStartStateCount(); istart++)
   {
      OverVertex ov_start;
      ov_start = add_vertex(og);
      og[ov_start].core_vertex = boost::graph_traits<Graph>::null_vertex();
      // set state
      og[ov_start].state = space->allocState();
      space->copyState(og[ov_start].state, pdef->getStartState(istart));
      // regular vertex properties
      og[ov_start].batch = 0;
      og[ov_start].is_shadow = false;
      og[ov_start].tag = 0;
      // connecting edge
      OverEdge e = add_edge(ov_singlestart, ov_start, og).first;
      og[e].distance = 0.0;
      og[e].batch = 0;
      // no edge states!
      ovs.push_back(ov_start);
   }
   
   // add goal to overlay graph
   ompl::base::GoalPtr goal = pdef->getGoal();
   if (goal)
   {
      if (goal->getType() == ompl::base::GOAL_STATE)
      {
         ompl::base::GoalState * goal_state = goal->as<ompl::base::GoalState>();
         OverVertex ov_goal;
         ov_goal = add_vertex(og);
         og[ov_goal].core_vertex = boost::graph_traits<Graph>::null_vertex();
         // set state
         og[ov_goal].state = space->allocState();
         space->copyState(og[ov_goal].state, goal_state->getState());
         // regular vertex properties
         og[ov_goal].batch = 0;
         og[ov_goal].is_shadow = false;
         og[ov_goal].tag = 0;
         // connecting edge
         OverEdge e = add_edge(ov_singlegoal, ov_goal, og).first;
         og[e].distance = 0.0;
         og[e].batch = 0;
         // no edge states!
         ovs.push_back(ov_goal);
      }
      else if (goal->getType() == ompl::base::GOAL_STATES)
      {
         ompl::base::GoalStates * goal_states = goal->as<ompl::base::GoalStates>();
         for (unsigned int igoal=0; igoal<goal_states->getStateCount(); igoal++)
         {
            OverVertex ov_goal;
            ov_goal = add_vertex(og);
            og[ov_goal].core_vertex = boost::graph_traits<Graph>::null_vertex();
            // set state
            og[ov_goal].state = space->allocState();
            space->copyState(og[ov_goal].state, goal_states->getState(igoal));
            // regular vertex properties
            og[ov_goal].batch = 0;
            og[ov_goal].is_shadow = false;
            og[ov_goal].tag = 0;
            // connecting edge
            OverEdge e = add_edge(ov_singlegoal, ov_goal, og).first;
            og[e].distance = 0.0;
            og[e].batch = 0;
            // no edge states!
            ovs.push_back(ov_goal);
         }
      }
      else
         throw std::runtime_error("unsupported ompl goal type!");
   }
   
   // connect to vertices within fixed radius in roadmap
   // to all batch vertices that we've generated so far
   for (std::vector<OverVertex>::iterator it=ovs.begin(); it!=ovs.end(); it++)
   {
      VertexIter vi, vi_end;
      for (boost::tie(vi,vi_end)=vertices(g); vi!=vi_end; ++vi)
      {
         double root_radius = roadmap_gen->root_radius(g[*vi].batch);
         
         double dist = space->distance(
            og[*it].state,
            g[*vi].state);
         if (root_radius < dist)
            continue;
         
         // add new anchor overlay vertex
         OverVertex v_anchor = add_vertex(og);
         og[v_anchor].core_vertex = *vi;
         og[v_anchor].state = 0;
         // no need to set core properties (e.g. state) on anchors,
         // since this is just an anchor, wont be copied

         // add overlay edge from root to anchor
         OverEdge e = add_edge(*it, v_anchor, og).first;
         // add edge properties
         // og[e].core_properties.index -- needs to be set on apply
         og[e].distance = dist;
         og[e].batch = g[*vi].batch;
         // w_lazy??
         // interior points, in bisection order
         edge_init_points(og[*it].state, g[*vi].state,
            dist, og[e].edge_states);
         og[e].edge_tags.resize(og[e].edge_states.size(), 0);
         //og[e].tag = 0;
      }
   }
   
   overlay_apply();
}

template <class MyGraph, class IncSP, class EvalStrategy>
bool ompl_multiset::E8Roadmap::do_lazysp_b(
   MyGraph & g,
   IncSP incsp, EvalStrategy evalstrategy,
   std::vector<Edge> & epath)
{
   if (_do_timing)
   {
      if (os_alglog)
      {
         return pr_bgl::lazy_shortest_path(g,
            og[ov_singlestart].core_vertex,
            og[ov_singlegoal].core_vertex,
            ompl_multiset::WMap(*this),
            get(&EProps::w_lazy,g),
            ompl_multiset::IsEvaledMap(*this),
            epath,
            incsp,
            evalstrategy,
            pr_bgl::make_lazysp_null_visitor_pair(
               ompl_multiset::make_lazysp_log_visitor(
                  get(boost::vertex_index, g),
                  get(&EProps::index, g),
                  *os_alglog),
               LazySPTimingVisitor(_dur_search, _dur_eval))
            );
      }
      else
      {
         return pr_bgl::lazy_shortest_path(g,
            og[ov_singlestart].core_vertex,
            og[ov_singlegoal].core_vertex,
            ompl_multiset::WMap(*this),
            get(&EProps::w_lazy,g),
            ompl_multiset::IsEvaledMap(*this),
            epath,
            incsp,
            evalstrategy,
            LazySPTimingVisitor(_dur_search, _dur_eval));
      }
   }
   else // no timing
   {
      if (os_alglog)
      {
         return pr_bgl::lazy_shortest_path(g,
            og[ov_singlestart].core_vertex,
            og[ov_singlegoal].core_vertex,
            ompl_multiset::WMap(*this),
            get(&EProps::w_lazy,g),
            ompl_multiset::IsEvaledMap(*this),
            epath,
            incsp,
            evalstrategy,
            ompl_multiset::make_lazysp_log_visitor(
               get(boost::vertex_index, g),
               get(&EProps::index, g),
               *os_alglog));
      }
      else
      {
         return pr_bgl::lazy_shortest_path(g,
            og[ov_singlestart].core_vertex,
            og[ov_singlegoal].core_vertex,
            ompl_multiset::WMap(*this),
            get(&EProps::w_lazy,g),
            ompl_multiset::IsEvaledMap(*this),
            epath,
            incsp,
            evalstrategy,
            pr_bgl::lazysp_null_visitor());
      }
   }
}

template <class MyGraph>
bool ompl_multiset::E8Roadmap::do_lazysp_a(MyGraph & g, std::vector<Edge> & epath)
{
   if (_search_type == SEARCH_TYPE_ASTAR) // a* as inner search
   {
      boost::chrono::high_resolution_clock::time_point time_heur_begin;
      if (_do_timing)
         time_heur_begin = boost::chrono::high_resolution_clock::now();
      
      // if we're running a* as inner lazysp alg,
      // we need storage for:
      // 1. h-values for all vertices (will be used by each run of the inner search)
      // 2. color values for each vertex
      std::vector<double> v_hvalues(num_vertices(eig));
      std::vector<double> v_fvalues(num_vertices(eig));
      std::vector<boost::default_color_type> v_colors(num_vertices(eig));
      
      printf("computing heuristic values ...\n");
      // assign distances to singlestart/singlegoal vertices later
      typename boost::graph_traits<MyGraph>::vertex_iterator vi, vi_end;
      for (boost::tie(vi,vi_end)=vertices(g); vi!=vi_end; ++vi)
      {
         if (*vi == og[ov_singlestart].core_vertex) continue;
         if (*vi == og[ov_singlegoal].core_vertex) continue;
         ompl::base::State * v_state = g[*vi].state;
         double dist_to_goal = HUGE_VAL;
         typename boost::graph_traits<MyGraph>::out_edge_iterator ei, ei_end;
         for (boost::tie(ei,ei_end)=out_edges(og[ov_singlegoal].core_vertex,g); ei!=ei_end; ei++)
         {
            ompl::base::State * vgoal_state = g[target(*ei,g)].state;
            double dist = space->distance(v_state, vgoal_state);
            if (dist < dist_to_goal)
               dist_to_goal = dist;
         }
         v_hvalues[get(get(boost::vertex_index,g),*vi)] = _coeff_distance * dist_to_goal;
      }
      // singlestart vertex
      {
         double h_to_goal = HUGE_VAL;
         typename boost::graph_traits<MyGraph>::out_edge_iterator ei, ei_end;
         for (boost::tie(ei,ei_end)=out_edges(og[ov_singlestart].core_vertex,g); ei!=ei_end; ei++)
         {
            Vertex v_start = target(*ei,g);
            double h = v_hvalues[get(get(boost::vertex_index,g),v_start)];
            if (h < h_to_goal)
               h_to_goal = h;
         }
         v_hvalues[get(get(boost::vertex_index,g),og[ov_singlestart].core_vertex)] = h_to_goal;
      }
      // singlegoal vertex
      v_hvalues[get(get(boost::vertex_index,g),og[ov_singlegoal].core_vertex)] = 0.;
      
      if (_do_timing)
         _dur_search += boost::chrono::high_resolution_clock::now() - time_heur_begin;
      
      switch (_eval_type)
      {
      case EVAL_TYPE_FWD:
         return do_lazysp_b(g,
            pr_bgl::make_lazysp_incsp_astar<MyGraph,EPWlazyMap>(
               boost::make_iterator_property_map(v_hvalues.begin(), get(boost::vertex_index,g)), // heuristic_map
               boost::make_iterator_property_map(v_fvalues.begin(), get(boost::vertex_index,g)), // cost_map,
               boost::make_iterator_property_map(v_colors.begin(), get(boost::vertex_index,g))), // color_map
            pr_bgl::LazySpEvalFwd(),
            epath);
      case EVAL_TYPE_REV:
         return do_lazysp_b(g,
            pr_bgl::make_lazysp_incsp_astar<MyGraph,EPWlazyMap>(
               boost::make_iterator_property_map(v_hvalues.begin(), get(boost::vertex_index,g)), // heuristic_map
               boost::make_iterator_property_map(v_fvalues.begin(), get(boost::vertex_index,g)), // cost_map,
               boost::make_iterator_property_map(v_colors.begin(), get(boost::vertex_index,g))), // color_map
            pr_bgl::LazySpEvalRev(),
            epath);
      case EVAL_TYPE_ALT:
         return do_lazysp_b(g,
            pr_bgl::make_lazysp_incsp_astar<MyGraph,EPWlazyMap>(
               boost::make_iterator_property_map(v_hvalues.begin(), get(boost::vertex_index,g)), // heuristic_map
               boost::make_iterator_property_map(v_fvalues.begin(), get(boost::vertex_index,g)), // cost_map,
               boost::make_iterator_property_map(v_colors.begin(), get(boost::vertex_index,g))), // color_map
            pr_bgl::LazySpEvalAlt(),
            epath);
      case EVAL_TYPE_BISECT:
         return do_lazysp_b(g,
            pr_bgl::make_lazysp_incsp_astar<MyGraph,EPWlazyMap>(
               boost::make_iterator_property_map(v_hvalues.begin(), get(boost::vertex_index,g)), // heuristic_map
               boost::make_iterator_property_map(v_fvalues.begin(), get(boost::vertex_index,g)), // cost_map,
               boost::make_iterator_property_map(v_colors.begin(), get(boost::vertex_index,g))), // color_map
            pr_bgl::LazySpEvalBisect(),
            epath);
      case EVAL_TYPE_FWD_EXPAND:
         return do_lazysp_b(g,
            pr_bgl::make_lazysp_incsp_astar<MyGraph,EPWlazyMap>(
               boost::make_iterator_property_map(v_hvalues.begin(), get(boost::vertex_index,g)), // heuristic_map
               boost::make_iterator_property_map(v_fvalues.begin(), get(boost::vertex_index,g)), // cost_map,
               boost::make_iterator_property_map(v_colors.begin(), get(boost::vertex_index,g))), // color_map
            pr_bgl::LazySpEvalFwdExpand(),
            epath);
      case EVAL_TYPE_PARTITION_ALL:
         return do_lazysp_b(g,
            pr_bgl::make_lazysp_incsp_astar<MyGraph,EPWlazyMap>(
               boost::make_iterator_property_map(v_hvalues.begin(), get(boost::vertex_index,g)), // heuristic_map
               boost::make_iterator_property_map(v_fvalues.begin(), get(boost::vertex_index,g)), // cost_map,
               boost::make_iterator_property_map(v_colors.begin(), get(boost::vertex_index,g))), // color_map
            pr_bgl::lazysp_partition_all<MyGraph,EPWlazyMap>(
               g, get(&EProps::w_lazy,g),
               1.0/3.0, // len_ref
               og[ov_singlestart].core_vertex,
               og[ov_singlegoal].core_vertex,
               true),
            epath);
      case EVAL_TYPE_SP_INDICATOR_PROBABILITY:
         return do_lazysp_b(g,
            pr_bgl::make_lazysp_incsp_astar<MyGraph,EPWlazyMap>(
               boost::make_iterator_property_map(v_hvalues.begin(), get(boost::vertex_index,g)), // heuristic_map
               boost::make_iterator_property_map(v_fvalues.begin(), get(boost::vertex_index,g)), // cost_map,
               boost::make_iterator_property_map(v_colors.begin(), get(boost::vertex_index,g))), // color_map
            pr_bgl::lazysp_sp_indicator_probability<MyGraph,EPWlazyMap,ompl_multiset::IsEvaledMap>(
               get(&EProps::w_lazy,g),
               ompl_multiset::IsEvaledMap(*this),
               1000, // nsamps
               og[ov_singlestart].core_vertex,
               og[ov_singlegoal].core_vertex,
               0), // seed
            epath);
      }
   }
   else // _search_type == SEARCH_TYPE_DIJKSTRAS
   {
      switch (_eval_type)
      {
      case EVAL_TYPE_FWD:
         return do_lazysp_b(g,
            pr_bgl::lazysp_incsp_dijkstra<MyGraph,EPWlazyMap>(),
            pr_bgl::LazySpEvalFwd(),
            epath);
      case EVAL_TYPE_REV:
         return do_lazysp_b(g,
            pr_bgl::lazysp_incsp_dijkstra<MyGraph,EPWlazyMap>(),
            pr_bgl::LazySpEvalRev(),
            epath);
      case EVAL_TYPE_ALT:
         return do_lazysp_b(g,
            pr_bgl::lazysp_incsp_dijkstra<MyGraph,EPWlazyMap>(),
            pr_bgl::LazySpEvalAlt(),
            epath);
      case EVAL_TYPE_BISECT:
         return do_lazysp_b(g,
            pr_bgl::lazysp_incsp_dijkstra<MyGraph,EPWlazyMap>(),
            pr_bgl::LazySpEvalBisect(),
            epath);
      case EVAL_TYPE_FWD_EXPAND:
         return do_lazysp_b(g,
            pr_bgl::lazysp_incsp_dijkstra<MyGraph,EPWlazyMap>(),
            pr_bgl::LazySpEvalFwdExpand(),
            epath);
      case EVAL_TYPE_PARTITION_ALL:
         return do_lazysp_b(g,
            pr_bgl::lazysp_incsp_dijkstra<MyGraph,EPWlazyMap>(),
            pr_bgl::lazysp_partition_all<MyGraph,EPWlazyMap>(
               g, get(&EProps::w_lazy,g),
               1.0/3.0, // len_ref
               og[ov_singlestart].core_vertex,
               og[ov_singlegoal].core_vertex,
               true),
            epath);
      case EVAL_TYPE_SP_INDICATOR_PROBABILITY:
         return do_lazysp_b(g,
            pr_bgl::lazysp_incsp_dijkstra<MyGraph,EPWlazyMap>(),
            pr_bgl::lazysp_sp_indicator_probability<MyGraph,EPWlazyMap,ompl_multiset::IsEvaledMap>(
               get(&EProps::w_lazy,g),
               ompl_multiset::IsEvaledMap(*this),
               1000, // nsamps
               og[ov_singlestart].core_vertex,
               og[ov_singlegoal].core_vertex,
               0), // seed
            epath);
      }
   }
   OMPL_ERROR("switch error.");
   return false;
}


ompl::base::PlannerStatus
ompl_multiset::E8Roadmap::solve(
   const ompl::base::PlannerTerminationCondition & ptc)
{
   boost::chrono::high_resolution_clock::time_point time_total_begin;
   if (_do_timing)
   {
      _dur_search = boost::chrono::high_resolution_clock::duration();
      _dur_eval = boost::chrono::high_resolution_clock::duration();
      time_total_begin = boost::chrono::high_resolution_clock::now();
   }
   
   // ok, do some sweet sweet lazy search!
   
   if (out_degree(ov_singlestart,og) == 0)
      throw std::runtime_error("no start states passed!");
   if (out_degree(ov_singlegoal,og) == 0)
      throw std::runtime_error("no goal states passed!");
   
   bool success = false;
   std::vector<Edge> epath;
   
   if (os_alglog)
   {
      
      for (unsigned int ui=0; ui<overlay_manager.applied_vertices.size(); ui++)
      {
         OverVertex vover = overlay_manager.applied_vertices[ui];
         Vertex vcore = og[vover].core_vertex;
         if (!g[vcore].state)
            *os_alglog << "singleroot-vertex applied-" << ui << std::endl;
      }
      
      for (unsigned int ui=0; ui<overlay_manager.applied_edges.size(); ui++)
      {
         OverEdge eover = overlay_manager.applied_edges[ui];
         Edge ecore = og[eover].core_edge;
         if (!g[source(ecore,g)].state || !g[target(ecore,g)].state)
            *os_alglog << "singleroot-edge applied-" << ui << std::endl;
      }
   }
   
   unsigned int num_batches = 0;
   
   // run batches of lazy search
   while (!success && ptc() == false)
   {
      printf("considering %u batches ...\n", num_batches);
      
      // should we do a search?
      if (_num_batches_init <= num_batches)
      {
         if (os_alglog)
         {
            *os_alglog << "subgraph " << roadmap_gen->get_num_batches_generated() << std::endl;
            *os_alglog << "alias reset" << std::endl;
            for (unsigned int ui=0; ui<overlay_manager.applied_vertices.size(); ui++)
            {
               OverVertex vover = overlay_manager.applied_vertices[ui];
               Vertex vcore = og[vover].core_vertex;
               *os_alglog << "alias vertex applied-" << ui
                  << " index " << get(get(boost::vertex_index,g),vcore) << std::endl;
            }
            for (unsigned int ui=0; ui<overlay_manager.applied_edges.size(); ui++)
            {
               OverEdge eover = overlay_manager.applied_edges[ui];
               Edge ecore = og[eover].core_edge;
               *os_alglog << "alias edge applied-" << ui
                  << " index " << g[ecore].index << std::endl;
            }
         }
         
         printf("running lazy search over %lu vertices and %lu edges...\n",
            num_vertices(eig), num_edges(eig));
         
         // run lazy search
         if (num_batches < roadmap_gen->get_num_batches_generated())
         {
            printf("doing filtered lazy search ...\n");
            filter_num_batches filter(get(&EProps::batch,g), num_batches);
            boost::filtered_graph<Graph,filter_num_batches> fg(g, filter);
            success = do_lazysp_a(fg, epath);
         }
         else
         {
            success = do_lazysp_a(g, epath);
         }
         
         if (success)
            break;
      }
      
      // did we run out of batches?
      if (_max_batches <= num_batches)
         return ompl::base::PlannerStatus::EXACT_SOLUTION;
      
      // consider the next batch
      num_batches++;
      
      if (roadmap_gen->get_num_batches_generated() < num_batches)
      {
         if (roadmap_gen->max_batches && roadmap_gen->max_batches <= roadmap_gen->get_num_batches_generated())
            return ompl::base::PlannerStatus::EXACT_SOLUTION;
         
         std::size_t new_batch = roadmap_gen->get_num_batches_generated();
         
         printf("densifying to batch [%lu] ...\n", new_batch);
         
         overlay_unapply();
         
         size_t v_from = num_vertices(eig);
         size_t e_from = num_edges(eig);
         
         // add a batch!
         //NN nnbatched(eig, get(&VProps::state,g), space, ompl_nn.get());
         nn.sync();
         roadmap_gen->generate(eig, nn,
            get(&VProps::state, g),
            get(&EProps::distance, g),
            get(&VProps::batch, g),
            get(&EProps::batch, g),
            get(&VProps::is_shadow, g));
         
         size_t v_to = num_vertices(eig);
         size_t e_to = num_edges(eig);
         m_subgraph_sizes.push_back(std::make_pair(v_to,e_to));
         
         // initialize new vertices/edges
         for (size_t vidx=v_from; vidx<v_to; vidx++)
            put(m_vidx_tag_map, vidx, 0);
         for (size_t eidx=e_from; eidx<e_to; eidx++)
         {
            Edge e = get(eig.edge_vector_map,eidx);
            edge_init_points(
               g[source(e,g)].state,
               g[target(e,g)].state,
               g[e].distance,
               g[e].edge_states);
            g[e].edge_tags.resize(g[e].edge_states.size(), 0);
         }
         
         printf("E8Roadmap: loading from cache ...\n");
         
         // load new batch from cache
         tag_cache.load_begin();
         tag_cache.load_vertices(m_vidx_tag_map, v_from, v_to);
         tag_cache.load_edges(m_eidx_tags_map, e_from, e_to);
         tag_cache.load_end();
         
         for (size_t eidx=e_from; eidx<e_to; eidx++)
         {
            Edge e = get(eig.edge_vector_map,eidx);
            calculate_w_lazy(e);
         }
         
         // add new edges to roots
         double root_radius = roadmap_gen->root_radius(new_batch);
         
         // iterate over all roots, connect them to the new batch
         std::vector<OverVertex> ovs;
         OverVertexIter ovi, ovi_end;
         for (boost::tie(ovi,ovi_end)=vertices(og); ovi!=ovi_end; ovi++)
         {
            if (*ovi == ov_singlestart || *ovi == ov_singlegoal)
               continue;
            if (og[*ovi].core_vertex == boost::graph_traits<Graph>::null_vertex())
               ovs.push_back(*ovi);
         }
         for (std::vector<OverVertex>::iterator it=ovs.begin(); it!=ovs.end(); it++)
         {
            VertexIter vi, vi_end;
            for (boost::tie(vi,vi_end)=vertices(g); vi!=vi_end; ++vi)
            {
               // core vertices in new batch only
               if (g[*vi].batch != (int)(new_batch))
                  continue;
               
               double dist = space->distance(
                  og[*it].state,
                  g[*vi].state);
               if (root_radius < dist)
                  continue;
               
               //printf("adding new root edge ...\n");
               
               // add new anchor overlay vertex
               OverVertex v_anchor = add_vertex(og);
               og[v_anchor].core_vertex = *vi;
               og[v_anchor].state = 0;

               // add overlay edge from root to anchor
               OverEdge e = add_edge(*it, v_anchor, og).first;
               // add edge properties
               // og[e].core_properties.index -- needs to be set on apply
               og[e].distance = dist;
               og[e].batch = g[*vi].batch;
               // w_lazy??
               // interior points, in bisection order
               edge_init_points(og[*it].state, g[*vi].state,
                  dist, og[e].edge_states);
               og[e].edge_tags.resize(og[e].edge_states.size(), 0);
               //og[e].tag = 0;
            }
         }
         
         overlay_apply();
      }
   }
   
   if (_do_timing)
   {
      printf("search duration: %f\n",
         boost::chrono::duration<double>(_dur_search).count());
      printf("eval duration: %f\n",
         boost::chrono::duration<double>(_dur_eval).count());
      _dur_total = boost::chrono::high_resolution_clock::now() - time_total_begin;
      printf("total duration: %f\n",
         boost::chrono::duration<double>(_dur_total).count());
   }

   if (success)
   {
      /* create the path */
      ompl::geometric::PathGeometric * path
         = new ompl::geometric::PathGeometric(si_);
      //path->append(g[og[ov_start].core_vertex].state->state);
      epath.pop_back(); // last edge targets singlegoal
      for (std::vector<Edge>::iterator it=epath.begin(); it!=epath.end(); it++)
         path->append(g[target(*it,g)].state);
      pdef_->addSolutionPath(ompl::base::PathPtr(path));
      return ompl::base::PlannerStatus::EXACT_SOLUTION;
   }
   else
   {
      return ompl::base::PlannerStatus::TIMEOUT;
   }
}

void ompl_multiset::E8Roadmap::solve_all()
{
   overlay_unapply();
   
   // evaluate all vertices first
   printf("solve_all() evaluating vertices ...\n");
   VertexIter vi, vi_end;
   for (boost::tie(vi,vi_end)=vertices(g); vi!=vi_end; ++vi)
      while (!effort_model.is_evaled(g[*vi].tag))
         effort_model.eval_partial(g[*vi].tag, g[*vi].state);
   
   printf("solve_all() evaluating edges ...\n");
   unsigned int count = 0;
   EdgeIter ei, ei_end;
   for (boost::tie(ei,ei_end)=edges(g); ei!=ei_end; ++ei)
   {
      printf("calculating edge %u/%lu ...\n", count, num_edges(eig));
      for (unsigned ui=0; ui<g[*ei].edge_tags.size(); ui++)
      {
         while (!effort_model.is_evaled(g[*ei].edge_tags[ui]))
            effort_model.eval_partial(g[*ei].edge_tags[ui], g[*ei].edge_states[ui]);
         if (effort_model.x_hat(g[*ei].edge_tags[ui], g[*ei].edge_states[ui])
            == std::numeric_limits<double>::infinity())
            break;
      }
      count++;
   }
   
   overlay_apply();
}

void ompl_multiset::E8Roadmap::dump_graph(std::ostream & os_graph)
{
   // dump graph
   // write it out to file
   boost::dynamic_properties props;
   props.property("state", ompl_multiset::make_rvstate_map_string_adaptor(
      get(&VProps::state,g),
      space->as<ompl::base::RealVectorStateSpace>()));
   props.property("batch", pr_bgl::make_string_map(get(&VProps::batch,g)));
   props.property("batch", pr_bgl::make_string_map(get(&EProps::batch,g)));
   props.property("is_shadow", pr_bgl::make_string_map(get(&VProps::is_shadow,g)));
   props.property("distance", pr_bgl::make_string_map(get(&EProps::distance,g)));
   pr_bgl::write_graphio_graph(os_graph, g,
      get(boost::vertex_index, g), get(&EProps::index, g));
   pr_bgl::write_graphio_properties(os_graph, g,
      get(boost::vertex_index, g), get(&EProps::index, g),
      props);
}

void ompl_multiset::E8Roadmap::cache_save_all()
{
   overlay_unapply();
   
   tag_cache.save_begin();
   
   for (size_t ibatch=0; ibatch<m_subgraph_sizes.size(); ibatch++)
   {
      if (ibatch == 0)
      {
         tag_cache.save_vertices(m_vidx_tag_map, 0, m_subgraph_sizes[ibatch].first);
         tag_cache.save_edges(m_eidx_tags_map, 0, m_subgraph_sizes[ibatch].second);
      }
      else
      {
         tag_cache.save_vertices(m_vidx_tag_map, m_subgraph_sizes[ibatch-1].first, m_subgraph_sizes[ibatch].first);
         tag_cache.save_edges(m_eidx_tags_map, m_subgraph_sizes[ibatch-1].second, m_subgraph_sizes[ibatch].second);
      }
   }
   
   tag_cache.save_end();
   
   overlay_apply();
}

double ompl_multiset::E8Roadmap::getDurTotal()
{
   return boost::chrono::duration<double>(_dur_total).count();
}

double ompl_multiset::E8Roadmap::getDurSearch()
{
   return boost::chrono::duration<double>(_dur_search).count();
}

double ompl_multiset::E8Roadmap::getDurEval()
{
   return boost::chrono::duration<double>(_dur_eval).count();
}

void ompl_multiset::E8Roadmap::overlay_apply()
{
   if (overlay_manager.is_applied)
      return;
   
   overlay_manager.apply();
   
   // manually copy over properties
   for (unsigned int ui=0; ui<overlay_manager.applied_vertices.size(); ui++)
   {
      OverVertex vover = overlay_manager.applied_vertices[ui];
      Vertex vcore = og[vover].core_vertex;
      g[vcore].state = og[vover].state;
      g[vcore].batch = og[vover].batch;
      g[vcore].is_shadow = og[vover].is_shadow;
      g[vcore].tag = og[vover].tag;
   }
   
   for (unsigned int ui=0; ui<overlay_manager.applied_edges.size(); ui++)
   {
      OverEdge eover = overlay_manager.applied_edges[ui];
      Edge ecore = og[eover].core_edge;
      g[ecore].distance = og[eover].distance;
      g[ecore].batch = og[eover].batch;
      //g[ecore].w_lazy = og[eover].w_lazy;
      g[ecore].edge_states = og[eover].edge_states;
      g[ecore].edge_tags = og[eover].edge_tags;
      //g[ecore].tag = og[eover].tag;
      calculate_w_lazy(ecore);
   }
}

void ompl_multiset::E8Roadmap::overlay_unapply()
{
   // unapply the overlay graph if it is applied
   if (!overlay_manager.is_applied)
      return;
   
   for (unsigned int ui=0; ui<overlay_manager.applied_vertices.size(); ui++)
   {
      OverVertex vover = overlay_manager.applied_vertices[ui];
      Vertex vcore = og[vover].core_vertex;
      og[vover].state = g[vcore].state;
      og[vover].batch = g[vcore].batch;
      og[vover].is_shadow = g[vcore].is_shadow;
      og[vover].tag = g[vcore].tag;
   }
   
   for (unsigned int ui=0; ui<overlay_manager.applied_edges.size(); ui++)
   {
      OverEdge eover = overlay_manager.applied_edges[ui];
      Edge ecore = og[eover].core_edge;
      og[eover].distance = g[ecore].distance;
      og[eover].batch = g[ecore].batch;
      //og[eover].w_lazy = g[ecore].w_lazy;
      //og[eover].is_evaled = g[ecore].is_evaled;
      og[eover].edge_states = g[ecore].edge_states;
      og[eover].edge_tags = g[ecore].edge_tags;
      //og[eover].tag = g[ecore].tag;
   }
   
   overlay_manager.unapply();
}

void ompl_multiset::E8Roadmap::edge_init_points(
   ompl::base::State * va_state, ompl::base::State * vb_state,
   double e_distance, std::vector< ompl::base::State * > & edge_states)
{
   // now many interior points do we need?
   unsigned int n = floor(e_distance/(2.0*check_radius));
   // allocate states
   edge_states.resize(n);
   for (unsigned int ui=0; ui<n; ui++)
      edge_states[ui] = space->allocState();
   // fill with interpolated states in bisection order
   const std::vector< std::pair<int,int> > & order = bisect_perm.get(n);
   for (unsigned int ui=0; ui<n; ui++)
      space->interpolate(va_state, vb_state,
         1.0*(1+order[ui].first)/(n+1),
         edge_states[ui]);
}

void ompl_multiset::E8Roadmap::calculate_w_lazy(const Edge & e)
{
   Vertex va = source(e,g);
   Vertex vb = target(e,g);
   // special case for singleroot edges
   if (!g[va].state)
   {
      if (effort_model.x_hat(g[vb].tag, g[vb].state) == std::numeric_limits<double>::infinity())
         g[e].w_lazy = std::numeric_limits<double>::infinity();
      else
         g[e].w_lazy = 0.5 * _coeff_checkcost * effort_model.p_hat(g[vb].tag, g[vb].state);
      return;
   }
   if (!g[vb].state)
   {
      if (effort_model.x_hat(g[va].tag, g[va].state) == std::numeric_limits<double>::infinity())
         g[e].w_lazy = std::numeric_limits<double>::infinity();
      else
         g[e].w_lazy = 0.5 * _coeff_checkcost * effort_model.p_hat(g[va].tag, g[va].state);
      return;
   }
   // ok, its a non-singleroot edge
   unsigned int ui;
   for (ui=0; ui<g[e].edge_tags.size(); ui++)
      if (effort_model.x_hat(g[e].edge_tags[ui], g[e].edge_states[ui]) == std::numeric_limits<double>::infinity())
         break;
   if (ui<g[e].edge_states.size()
      || effort_model.x_hat(g[va].tag, g[va].state) == std::numeric_limits<double>::infinity()
      || effort_model.x_hat(g[vb].tag, g[vb].state) == std::numeric_limits<double>::infinity())
   {
      g[e].w_lazy = std::numeric_limits<double>::infinity();
   }
   else
   {
      g[e].w_lazy = 0.0;
      g[e].w_lazy += _coeff_distance * g[e].distance;
      g[e].w_lazy += _coeff_batch * g[e].distance * g[e].batch;
      // interior states
      for (ui=0; ui<g[e].edge_tags.size(); ui++)
         g[e].w_lazy += _coeff_checkcost * effort_model.p_hat(g[e].edge_tags[ui], g[e].edge_states[ui]);
      // half bounary vertices
      g[e].w_lazy += 0.5 * _coeff_checkcost * effort_model.p_hat(g[va].tag, g[va].state);
      g[e].w_lazy += 0.5 * _coeff_checkcost * effort_model.p_hat(g[vb].tag, g[vb].state);
   }
}

bool ompl_multiset::E8Roadmap::isevaledmap_get(const Edge & e)
{
   // this directly calls the family effort model (distance not needed!)
   Vertex va = source(e, g);
   if (g[va].state && !effort_model.is_evaled(g[va].tag))
      return false;
   Vertex vb = target(e, g);
   if (g[vb].state && !effort_model.is_evaled(g[vb].tag))
      return false;
   for (unsigned int ui=0; ui<g[e].edge_tags.size(); ui++)
      if (!effort_model.is_evaled(g[e].edge_tags[ui]))
         return false;
   return true;
}

double ompl_multiset::E8Roadmap::wmap_get(const Edge & e)
{
   // check all points!
   Vertex va = source(e, g);
   Vertex vb = target(e, g);

   // check endpoints first
   do
   {
      if (g[va].state && !effort_model.is_evaled(g[va].tag))
      {
         bool success = effort_model.eval_partial(g[va].tag, g[va].state);
         if (!success)
            break;
      }
      if (g[vb].state && !effort_model.is_evaled(g[vb].tag))
      {
         bool success = effort_model.eval_partial(g[vb].tag, g[vb].state);
         if (!success)
            break;
      }
      for (unsigned ui=0; ui<g[e].edge_tags.size(); ui++)
      {
         if (!effort_model.is_evaled(g[e].edge_tags[ui]))
         {
            bool success = effort_model.eval_partial(g[e].edge_tags[ui], g[e].edge_states[ui]);
            if (!success)
               break;
         }
      }
   }
   while (0);
   
   // recalculate wlazy for this edge and any incident edges
   calculate_w_lazy(e);
   OutEdgeIter ei, ei_end;
   for (boost::tie(ei,ei_end)=out_edges(va,g); ei!=ei_end; ei++)
      calculate_w_lazy(*ei);
   for (boost::tie(ei,ei_end)=out_edges(vb,g); ei!=ei_end; ei++)
      calculate_w_lazy(*ei);
   
   return g[e].w_lazy;
}

double ompl_multiset::E8Roadmap::ompl_nn_dist(const Vertex & va, const Vertex & vb)
{
   return space->distance(g[va].state, g[vb].state);
}
