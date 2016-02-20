/* File: LEMUR.cpp
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
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>

#include <ompl/base/Planner.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/datastructures/NearestNeighborsGNAT.h>
#include <ompl/datastructures/NearestNeighborsLinear.h>

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
#include <pr_bgl/lazysp_selector_partition_all.h>
#include <pr_bgl/lazysp_selector_sp_indicator_probability.h>
#include <pr_bgl/lpastar.h>
#include <pr_bgl/incbi.h>
#include <pr_bgl/lazysp_incsp_dijkstra.h>
#include <pr_bgl/lazysp_incsp_astar.h>
#include <pr_bgl/lazysp_incsp_lpastar.h>
#include <pr_bgl/lazysp_incsp_incbi.h>

#include <ompl_lemur/aborting_space_information.h>
#include <ompl_lemur/rvstate_map_string_adaptor.h>
#include <ompl_lemur/BisectPerm.h>
#include <ompl_lemur/NearestNeighborsLinearBGL.h>
#include <ompl_lemur/Roadmap.h>
#include <ompl_lemur/EffortModel.h>
#include <ompl_lemur/LEMUR.h>
#include <ompl_lemur/lazysp_log_visitor.h>

namespace boost {

template <class Filter>
inline
ompl_lemur::LEMUR::Vertex
vertex(size_t v_index, const boost::filtered_graph<ompl_lemur::LEMUR::Graph, Filter>& g)
{
   return vertex(v_index, g.m_g);
}

// since we've structured our graph (via the overlay manager)
// to always remove vertices in the reverse order,
// we replace boost's remove_vertex function
// (which iterates over all graph edges in order to shift vertex descriptors)
// with our simple and much faster version
inline
void remove_vertex(ompl_lemur::LEMUR::Vertex v, ompl_lemur::LEMUR::Graph & g)
{
   BOOST_ASSERT(v == num_vertices(g)-1);
   g.m_vertices.pop_back();
}

} // boost namespace

/*
 * ok, how does the overlay graph relate to the core graph?
 * 
 * there are three types of overlay vertices:
 *  - singleroots (no state, null core_vertex)
 *  - real vertices (with state) (roots, anchors, etc)
 *    - with core_vertex: anchors (states owned by core graph)
 *    - without core_vertex (states owned by overlay)
 * 
 * note that a vertex can be both an anchor and a root
 * (if a root is exactly in the graph); this will have no state
 * 
 *  - edges from singlestart/singlegoal to roots have no points and a distance of 0
 *  - other edges, from roots to core vertices (or other roots) look like regular edges
 */

ompl_lemur::LEMUR::LEMUR(
      const ompl::base::StateSpacePtr & space,
      EffortModel & effort_model,
      TagCache<VIdxTagMap,EIdxTagsMap> & tag_cache):
   ompl::base::Planner(
      ompl_lemur::get_aborting_space_information(space), "LEMUR"),
   effort_model(effort_model),
   space(space),
   check_radius(0.5*space->getLongestValidSegmentLength()),
   eig(g, get(&EProps::index,g)),
   overlay_manager(eig,og,
      get(&OverVProps::core_vertex, og),
      get(&OverEProps::core_edge, og)),
   tag_cache(tag_cache),
   //nn(new ompl::NearestNeighborsLinear<Vertex>), // option A1
   //nn(new ompl::NearestNeighborsGNAT<Vertex>), // option A2
   nn(new ompl_lemur::NearestNeighborsLinearBGL<Graph,VPStateMap>(g, get(&VProps::state,g), space)), // option B
   //nn(new ompl::NearestNeighborsGNAT<Vertex>),
   _coeff_distance(1.),
   _coeff_checkcost(0.),
   _coeff_batch(0.),
   _do_timing(false),
   _persist_roots(false),
   _num_batches_init(0),
   _max_batches(UINT_MAX),
   _search_type(SEARCH_TYPE_ASTAR),
   _eval_type(EVAL_TYPE_ALT),
   os_alglog(0),
   m_vidx_tag_map(pr_bgl::make_compose_property_map(get(&VProps::tag,g), get(boost::vertex_index,g))),
   m_eidx_tags_map(pr_bgl::make_compose_property_map(get(&EProps::edge_tags,g), eig.edge_vector_map))
{
   Planner::declareParam<std::string>("roadmap_type", this,
      &ompl_lemur::LEMUR::setRoadmapType,
      &ompl_lemur::LEMUR::getRoadmapType);
   Planner::declareParam<double>("coeff_distance", this,
      &ompl_lemur::LEMUR::setCoeffDistance,
      &ompl_lemur::LEMUR::getCoeffDistance, "0.:1.:10000.");
   Planner::declareParam<double>("coeff_checkcost", this,
      &ompl_lemur::LEMUR::setCoeffCheckcost,
      &ompl_lemur::LEMUR::getCoeffCheckcost, "0.:1.:10000.");
   Planner::declareParam<double>("coeff_batch", this,
      &ompl_lemur::LEMUR::setCoeffBatch,
      &ompl_lemur::LEMUR::getCoeffBatch, "0.:1.:10000.");
   Planner::declareParam<bool>("do_timing", this,
      &ompl_lemur::LEMUR::setDoTiming,
      &ompl_lemur::LEMUR::getDoTiming);
   Planner::declareParam<bool>("persist_roots", this,
      &ompl_lemur::LEMUR::setPersistRoots,
      &ompl_lemur::LEMUR::getPersistRoots);
   Planner::declareParam<unsigned int>("num_batches_init", this,
      &ompl_lemur::LEMUR::setNumBatchesInit,
      &ompl_lemur::LEMUR::getNumBatchesInit);
   Planner::declareParam<unsigned int>("max_batches", this,
      &ompl_lemur::LEMUR::setMaxBatches,
      &ompl_lemur::LEMUR::getMaxBatches);
   Planner::declareParam<std::string>("search_type", this,
      &ompl_lemur::LEMUR::setSearchType,
      &ompl_lemur::LEMUR::getSearchType);
   Planner::declareParam<std::string>("eval_type", this,
      &ompl_lemur::LEMUR::setEvalType,
      &ompl_lemur::LEMUR::getEvalType);
   
   // setup ompl_nn
   //nn->setDistanceFunction(boost::bind(&ompl_lemur::LEMUR::nn_dist, this, _1, _2)); // option A
   
   // construct persistent singleroot/singlegoal overlay vertices
   ov_singlestart = add_vertex(og);
   ov_singlegoal = add_vertex(og);
   og[ov_singlestart].core_vertex = boost::graph_traits<Graph>::null_vertex();
   og[ov_singlestart].state = 0;
   og[ov_singlestart].batch = 0;
   og[ov_singlestart].is_shadow = false;
   og[ov_singlestart].tag = 0;
   og[ov_singlegoal].core_vertex = boost::graph_traits<Graph>::null_vertex();
   og[ov_singlegoal].state = 0;
   og[ov_singlegoal].batch = 0;
   og[ov_singlegoal].is_shadow = false;
   og[ov_singlegoal].tag = 0;
   
   printf("space->getLongestValidSegmentLength(): %f\n", space->getLongestValidSegmentLength());
   
   printf("LEMUR: constructor finished.\n");
}

ompl_lemur::LEMUR::~LEMUR()
{
   overlay_unapply(); // just to be sure
   
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
      if (og[*ovi].state && og[*ovi].core_vertex==boost::graph_traits<Graph>::null_vertex())
         space->freeState(og[*ovi].state);
   OverEdgeIter oei, oei_end;
   for (boost::tie(oei,oei_end)=edges(og); oei!=oei_end; ++oei)
      for (unsigned int ui=0; ui<og[*oei].edge_states.size(); ui++)
         space->freeState(og[*oei].edge_states[ui]);
}

void ompl_lemur::LEMUR::registerRoadmapType(std::string roadmap_type,
   boost::function<Roadmap<RoadmapArgs> * (RoadmapArgs args)> factory)
{
   _roadmap_registry[roadmap_type] = factory;
}

void ompl_lemur::LEMUR::setRoadmapType(std::string roadmap_type)
{
   if (_roadmap && _roadmap->initialized)
   {
      if (roadmap_type == _roadmap_type)
         return;
      throw std::runtime_error("roadmap cannot be changed once initialized!");
   }

   std::map<std::string, boost::function<Roadmap<RoadmapArgs> * (RoadmapArgs args)> >::iterator it=_roadmap_registry.find(roadmap_type);
   if (it == _roadmap_registry.end())
      throw std::runtime_error("roadmap type not in registry!");
   
   RoadmapArgs args(
      space, eig,
      get(&VProps::state, g),
      get(&EProps::distance, g),
      get(&VProps::batch, g),
      get(&EProps::batch, g),
      get(&VProps::is_shadow, g),
      eig.edge_vector_map,
      nn.get());
   
   _roadmap.reset(it->second(args));
   
   // copy over parameters
   while (_roadmap_params.size())
   {
      std::string prefixed_param = "roadmap." + _roadmap_params.back();
      params().remove(prefixed_param);
      _roadmap_params.pop_back();
   }
   _roadmap->params.getParamNames(_roadmap_params);
   params().include(_roadmap->params, "roadmap");
   
   _roadmap_type = roadmap_type;
}

std::string ompl_lemur::LEMUR::getRoadmapType() const
{
   return _roadmap_type;
}

void ompl_lemur::LEMUR::setCoeffDistance(double coeff_distance)
{
   _coeff_distance = coeff_distance;
}

double ompl_lemur::LEMUR::getCoeffDistance() const
{
   return _coeff_distance;
}

void ompl_lemur::LEMUR::setCoeffCheckcost(double coeff_checkcost)
{
   _coeff_checkcost = coeff_checkcost;
}

double ompl_lemur::LEMUR::getCoeffCheckcost() const
{
   return _coeff_checkcost;
}

void ompl_lemur::LEMUR::setCoeffBatch(double coeff_batch)
{
   _coeff_batch = coeff_batch;
}

double ompl_lemur::LEMUR::getCoeffBatch() const
{
   return _coeff_batch;
}

void ompl_lemur::LEMUR::setDoTiming(bool do_timing)
{
   _do_timing = do_timing;
}

bool ompl_lemur::LEMUR::getDoTiming() const
{
   return _do_timing;
}

void ompl_lemur::LEMUR::setPersistRoots(bool persist_roots)
{
   _persist_roots = persist_roots;
}

bool ompl_lemur::LEMUR::getPersistRoots() const
{
   return _persist_roots;
}

void ompl_lemur::LEMUR::setNumBatchesInit(unsigned int num_batches_init)
{
   _num_batches_init = num_batches_init;
}

unsigned int ompl_lemur::LEMUR::getNumBatchesInit() const
{
   return _num_batches_init;
}

void ompl_lemur::LEMUR::setMaxBatches(unsigned int max_batches)
{
   _max_batches = max_batches;
}

unsigned int ompl_lemur::LEMUR::getMaxBatches() const
{
   return _max_batches;
}

void ompl_lemur::LEMUR::setSearchType(std::string search_type)
{
   if (search_type == "dijkstras")
      _search_type = SEARCH_TYPE_DIJKSTRAS;
   else if (search_type == "astar")
      _search_type = SEARCH_TYPE_ASTAR;
   else if (search_type == "lpastar")
      _search_type = SEARCH_TYPE_LPASTAR;
   else if (search_type == "incbi")
      _search_type = SEARCH_TYPE_INCBI;
   else
      throw std::runtime_error("Search type parameter must be dijkstras, astar, or lpastar.");
}

std::string ompl_lemur::LEMUR::getSearchType() const
{
   switch (_search_type)
   {
   case SEARCH_TYPE_DIJKSTRAS: return "dijkstras";
   case SEARCH_TYPE_ASTAR: return "astar";
   case SEARCH_TYPE_LPASTAR: return "lpastar";
   case SEARCH_TYPE_INCBI: return "incbi";
   default:
      throw std::runtime_error("corrupted _search_type!");
   }
}

void ompl_lemur::LEMUR::setEvalType(std::string eval_type)
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
      throw std::runtime_error("Eval type parameter must be fwd rev alt bisect or fwd_expand.");
}

std::string ompl_lemur::LEMUR::getEvalType() const
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
      throw std::runtime_error("corrupted _eval_type!");
   }
}

void ompl_lemur::LEMUR::setProblemDefinition(
   const ompl::base::ProblemDefinitionPtr & pdef)
{
   // call planner base class implementation
   // this will set my pdef_ and update my pis_
   ompl::base::Planner::setProblemDefinition(pdef);
   
   // clear overlay graph
   overlay_unapply(); // just to be sure
   
   // remove any virtual edges
   clear_vertex(ov_singlestart, og);
   clear_vertex(ov_singlegoal, og);
   
   // if asked to, remove all root (and anchor) vertices / edges
   if (!_persist_roots)
   {
      // free all edge internal data
      OverEdgeIter oei, oei_end;
      for (boost::tie(oei,oei_end)=edges(og); oei!=oei_end; oei++)
         for (unsigned int ui=0; ui<og[*oei].edge_states.size(); ui++)
            space->freeState(og[*oei].edge_states[ui]);
      // clear and free all non-singleroot vertices
      OverVertexIter ovi, ovi_end, ovi_next;
      boost::tie(ovi,ovi_end) = vertices(og);
      for (ovi_next=ovi; ovi!=ovi_end; ovi=ovi_next)
      {
         ++ovi_next;
         if (*ovi == ov_singlestart) continue;
         if (*ovi == ov_singlegoal) continue;
         space->freeState(og[*ovi].state);
         clear_vertex(*ovi, og);
         remove_vertex(*ovi, og);
      }
      map_to_overlay.clear();
   }
   
   // collect all root vertices to add (bool = is_goal)
   std::vector< std::pair<ompl::base::State *, bool> > root_states;
   
   // find start states
   for (unsigned int istart=0; istart<pdef->getStartStateCount(); istart++)
   {
      ompl::base::State * state = space->allocState();
      space->copyState(state, pdef->getStartState(istart));
      root_states.push_back(std::make_pair(state, false));
   }
   
   // find goal states
   ompl::base::GoalPtr goal = pdef->getGoal();
   if (goal)
   {
      if (goal->getType() == ompl::base::GOAL_STATE)
      {
         ompl::base::State * state = space->allocState();
         ompl::base::GoalState * goal_state = goal->as<ompl::base::GoalState>();
         space->copyState(state, goal_state->getState());
         root_states.push_back(std::make_pair(state, true));
      }
      else if (goal->getType() == ompl::base::GOAL_STATES)
      {
         ompl::base::GoalStates * goal_states = goal->as<ompl::base::GoalStates>();
         for (unsigned int igoal=0; igoal<goal_states->getStateCount(); igoal++)
         {
            ompl::base::State * state = space->allocState();
            space->copyState(state, goal_states->getState(igoal));
            root_states.push_back(std::make_pair(state, true));
         }
      }
      else
         throw std::runtime_error("unsupported ompl goal type!");
   }
   
   // add all root vertices, singleroot edges, and anchors necessary!
   for (unsigned int ui=0; ui<root_states.size(); ui++)
   {
      // find or create overlay vertex with this state
      ompl::base::State * state = root_states[ui].first;
      OverVertex ov_root;
      do
      {
         OverVertexIter ovi, ovi_end;
         VertexIter vi, vi_end;
         
         // does this state already exist in the overlay? (as a root or anchor)
         for (boost::tie(ovi,ovi_end)=vertices(og); ovi!=ovi_end; ovi++)
            if (og[*ovi].state && space->equalStates(og[*ovi].state, state))
               break;
         if (ovi!=ovi_end)
         {
            ov_root = *ovi;
            space->freeState(state);
            break;
         }
      
         // does this state already exist in the core graph?
         for (boost::tie(vi,vi_end)=vertices(g); vi!=vi_end; vi++)
            if (space->equalStates(g[*vi].state, state))
               break;
         if (vi != vi_end)
         {
            // add new anchor overlay vertex
            ov_root = add_vertex(og);
            og[ov_root].core_vertex = *vi;
            og[ov_root].state = g[*vi].state;
            // no need to set core properties on anchors,
            // since this is just an anchor, wont be copied
            // we set state pointer though so search above will find it
            map_to_overlay[*vi] = ov_root;
            space->freeState(state);
            break;
         }

         // ok, make a new non-anchor vertex for this root
         ov_root = add_vertex(og);
         og[ov_root].core_vertex = boost::graph_traits<Graph>::null_vertex();
         og[ov_root].state = state;
         og[ov_root].batch = 0;
         og[ov_root].is_shadow = false;
         og[ov_root].tag = 0;
            
         // find neighbors in core roadmap as anchors
         // (to all batch vertices that we've generated so far)
         // (for now, we skip neighbors in overlay graph -- what radius?)
         for (boost::tie(vi,vi_end)=vertices(g); vi!=vi_end; ++vi)
         {
            double root_radius = _roadmap->root_radius(g[*vi].batch);
            double dist = space->distance(state, g[*vi].state);
            if (root_radius < dist)
               continue;
            
            // get the anchor overlay vertex for this neighbor
            OverVertex ov_anchor;
            // does an anchor already exist for this core vertex?
            std::map<Vertex, OverVertex>::iterator found = map_to_overlay.find(*vi);
            if (found != map_to_overlay.end())
            {
               ov_anchor = found->second;
            }
            else
            {
               // add new anchor overlay vertex
               ov_anchor = add_vertex(og);
               og[ov_anchor].core_vertex = *vi;
               og[ov_anchor].state = g[*vi].state;
               // no need to set core properties on anchors,
               // since this is just an anchor, wont be copied
               // we set state pointer though so search above will find it
               map_to_overlay[*vi] = ov_anchor;
            }
            
            // add overlay edge from root to anchor
            OverEdge oe = add_edge(ov_root, ov_anchor, og).first;
            // add edge properties
            og[oe].distance = dist;
            og[oe].batch = g[*vi].batch;
            // w_lazy??
            // interior points, in bisection order
            og[oe].num_edge_states = floor(dist/(2.0*check_radius));
            //edge_init_points(state, g[*vi].state, dist, og[oe].edge_states);
            //og[oe].edge_tags.resize(og[oe].edge_states.size(), 0);
            //og[e].tag = 0;
         }
         
      }
      while (0);
      
      // connect up to singlestart or singlegoal (if not already)
      OverVertex ov_singleroot = (root_states[ui].second == false) ? ov_singlestart : ov_singlegoal;
      if (!edge(ov_singleroot, ov_root, og).second)
      {
         OverEdge e = add_edge(ov_singleroot, ov_root, og).first;
         og[e].distance = 0.0;
         og[e].batch = 0;
         og[e].num_edge_states = 0;
      }
   }
   
   // ompl::base::SpaceInformationPtr si_new = pdef->getSpaceInformation();
   //if (si_new != family_effort_model.si_target)
   if (effort_model.has_changed())
   {
      overlay_apply();
      
      // route target si to family effort model
      // this will re-run reverse dijkstra's on the family graph
      //family_effort_model.set_target(si_new);
      
      // recalculate wlazy if necessary
      // we probably always need to recalc wlazy for overlay states!
      EdgeIter ei, ei_end;
      for (boost::tie(ei,ei_end)=edges(g); ei!=ei_end; ++ei)
         calculate_w_lazy(*ei);
      
      overlay_unapply();
   }
}

template <class MyGraph, class IncSP, class EvalStrategy>
bool ompl_lemur::LEMUR::do_lazysp_b(
   MyGraph & g,
   IncSP incsp, EvalStrategy evalstrategy,
   std::vector<Edge> & epath)
{
   if (_do_timing)
   {
      if (os_alglog)
      {
         return pr_bgl::lazysp(g,
            og[ov_singlestart].core_vertex,
            og[ov_singlegoal].core_vertex,
            ompl_lemur::WMap(*this),
            get(&EProps::w_lazy,g),
            ompl_lemur::IsEvaledMap(*this),
            epath,
            incsp,
            evalstrategy,
            pr_bgl::make_lazysp_visitor_pair(
               ompl_lemur::make_lazysp_log_visitor(
                  get(boost::vertex_index, g),
                  get(&EProps::index, g),
                  *os_alglog),
               LazySPTimingVisitor(_dur_search, _dur_eval, _dur_selector, _dur_selector_notify))
            );
      }
      else
      {
         return pr_bgl::lazysp(g,
            og[ov_singlestart].core_vertex,
            og[ov_singlegoal].core_vertex,
            ompl_lemur::WMap(*this),
            get(&EProps::w_lazy,g),
            ompl_lemur::IsEvaledMap(*this),
            epath,
            incsp,
            evalstrategy,
            LazySPTimingVisitor(_dur_search, _dur_eval, _dur_selector, _dur_selector_notify));
      }
   }
   else // no timing
   {
      if (os_alglog)
      {
         return pr_bgl::lazysp(g,
            og[ov_singlestart].core_vertex,
            og[ov_singlegoal].core_vertex,
            ompl_lemur::WMap(*this),
            get(&EProps::w_lazy,g),
            ompl_lemur::IsEvaledMap(*this),
            epath,
            incsp,
            evalstrategy,
            ompl_lemur::make_lazysp_log_visitor(
               get(boost::vertex_index, g),
               get(&EProps::index, g),
               *os_alglog));
      }
      else
      {
         return pr_bgl::lazysp(g,
            og[ov_singlestart].core_vertex,
            og[ov_singlegoal].core_vertex,
            ompl_lemur::WMap(*this),
            get(&EProps::w_lazy,g),
            ompl_lemur::IsEvaledMap(*this),
            epath,
            incsp,
            evalstrategy,
            pr_bgl::lazysp_visitor_null());
      }
   }
}

template <class MyGraph>
bool ompl_lemur::LEMUR::do_lazysp_a(MyGraph & g, std::vector<Edge> & epath)
{
   if (_search_type == SEARCH_TYPE_ASTAR
      || _search_type == SEARCH_TYPE_LPASTAR) // a* as inner search
   {
      boost::chrono::high_resolution_clock::time_point time_heur_begin;
      if (_do_timing)
         time_heur_begin = boost::chrono::high_resolution_clock::now();
      
      // if we're running a* as inner lazysp alg,
      // we need storage for:
      // 1. h-values for all vertices (will be used by each run of the inner search)
      // 2. color values for each vertex
      std::vector<double> v_hvalues(num_vertices(eig), 0.0);
      
      // singlegoal vertex
      v_hvalues[get(get(boost::vertex_index,g),og[ov_singlegoal].core_vertex)] = 0.;
      
      // assign distances to all non-singlestart/singlegoal vertices
      typename boost::graph_traits<MyGraph>::vertex_iterator vi, vi_end;
      for (boost::tie(vi,vi_end)=vertices(g); vi!=vi_end; ++vi)
      {
         if (*vi == og[ov_singlestart].core_vertex) continue;
         if (*vi == og[ov_singlegoal].core_vertex) continue;
         ompl::base::State * v_state = g[*vi].state;
         double dist_to_goal = HUGE_VAL; // space distance to nearest goal vertex
         typename boost::graph_traits<MyGraph>::out_edge_iterator ei, ei_end;
         for (boost::tie(ei,ei_end)=out_edges(og[ov_singlegoal].core_vertex,g); ei!=ei_end; ei++)
         {
            ompl::base::State * vgoal_state = g[target(*ei,g)].state;
            double dist = space->distance(v_state, vgoal_state);
            if (dist < dist_to_goal)
               dist_to_goal = dist;
         }
         v_hvalues[get(get(boost::vertex_index,g),*vi)] = _singlegoal_cost + _coeff_distance * dist_to_goal;
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
         v_hvalues[get(get(boost::vertex_index,g),og[ov_singlestart].core_vertex)] = _singlestart_cost + h_to_goal;
      }
      
      if (_do_timing)
         _dur_search += boost::chrono::high_resolution_clock::now() - time_heur_begin;
      
      if (_search_type == SEARCH_TYPE_ASTAR)
      {
         // astar
         std::vector<Vertex> v_startpreds(num_vertices(eig));
         std::vector<double> v_startdist(num_vertices(eig));
         std::vector<double> v_fvalues(num_vertices(eig));
         std::vector<boost::default_color_type> v_colors(num_vertices(eig));
         
         switch (_eval_type)
         {
         case EVAL_TYPE_FWD:
            return do_lazysp_b(g,
               pr_bgl::make_lazysp_incsp_astar<MyGraph,EPWlazyMap>(
                  boost::make_iterator_property_map(v_hvalues.begin(), get(boost::vertex_index,g)), // heuristic_map
                  boost::make_iterator_property_map(v_startpreds.begin(), get(boost::vertex_index,g)), // startpreds_map
                  boost::make_iterator_property_map(v_startdist.begin(), get(boost::vertex_index,g)), // startdist_map
                  boost::make_iterator_property_map(v_fvalues.begin(), get(boost::vertex_index,g)), // cost_map,
                  boost::make_iterator_property_map(v_colors.begin(), get(boost::vertex_index,g))), // color_map
               pr_bgl::lazysp_selector_fwd(),
               epath);
         case EVAL_TYPE_REV:
            return do_lazysp_b(g,
               pr_bgl::make_lazysp_incsp_astar<MyGraph,EPWlazyMap>(
                  boost::make_iterator_property_map(v_hvalues.begin(), get(boost::vertex_index,g)), // heuristic_map
                  boost::make_iterator_property_map(v_startpreds.begin(), get(boost::vertex_index,g)), // startpreds_map
                  boost::make_iterator_property_map(v_startdist.begin(), get(boost::vertex_index,g)), // startdist_map
                  boost::make_iterator_property_map(v_fvalues.begin(), get(boost::vertex_index,g)), // cost_map,
                  boost::make_iterator_property_map(v_colors.begin(), get(boost::vertex_index,g))), // color_map
               pr_bgl::lazysp_selector_rev(),
               epath);
         case EVAL_TYPE_ALT:
            return do_lazysp_b(g,
               pr_bgl::make_lazysp_incsp_astar<MyGraph,EPWlazyMap>(
                  boost::make_iterator_property_map(v_hvalues.begin(), get(boost::vertex_index,g)), // heuristic_map
                  boost::make_iterator_property_map(v_startpreds.begin(), get(boost::vertex_index,g)), // startpreds_map
                  boost::make_iterator_property_map(v_startdist.begin(), get(boost::vertex_index,g)), // startdist_map
                  boost::make_iterator_property_map(v_fvalues.begin(), get(boost::vertex_index,g)), // cost_map,
                  boost::make_iterator_property_map(v_colors.begin(), get(boost::vertex_index,g))), // color_map
               pr_bgl::lazysp_selector_alt(),
               epath);
         case EVAL_TYPE_BISECT:
            return do_lazysp_b(g,
               pr_bgl::make_lazysp_incsp_astar<MyGraph,EPWlazyMap>(
                  boost::make_iterator_property_map(v_hvalues.begin(), get(boost::vertex_index,g)), // heuristic_map
                  boost::make_iterator_property_map(v_startpreds.begin(), get(boost::vertex_index,g)), // startpreds_map
                  boost::make_iterator_property_map(v_startdist.begin(), get(boost::vertex_index,g)), // startdist_map
                  boost::make_iterator_property_map(v_fvalues.begin(), get(boost::vertex_index,g)), // cost_map,
                  boost::make_iterator_property_map(v_colors.begin(), get(boost::vertex_index,g))), // color_map
               pr_bgl::lazysp_selector_bisect(),
               epath);
         case EVAL_TYPE_FWD_EXPAND:
            return do_lazysp_b(g,
               pr_bgl::make_lazysp_incsp_astar<MyGraph,EPWlazyMap>(
                  boost::make_iterator_property_map(v_hvalues.begin(), get(boost::vertex_index,g)), // heuristic_map
                  boost::make_iterator_property_map(v_startpreds.begin(), get(boost::vertex_index,g)), // startpreds_map
                  boost::make_iterator_property_map(v_startdist.begin(), get(boost::vertex_index,g)), // startdist_map
                  boost::make_iterator_property_map(v_fvalues.begin(), get(boost::vertex_index,g)), // cost_map,
                  boost::make_iterator_property_map(v_colors.begin(), get(boost::vertex_index,g))), // color_map
               pr_bgl::lazysp_selector_fwdexpand(),
               epath);
         case EVAL_TYPE_PARTITION_ALL: 
         {
            double len_ref = 1.0/3.0;
            
            boost::chrono::high_resolution_clock::time_point time_selector_init_begin
               = boost::chrono::high_resolution_clock::now();
            printf("computing partition_all from scratch ...\n");
            pr_bgl::partition_all_matrix solver(num_vertices(g));
            // add all edges (both directions)
            std::pair<EdgeIter,EdgeIter> ep=edges(g);
            for (EdgeIter ei=ep.first; ei!=ep.second; ei++)
            {
               double weight_frac = g[*ei].w_lazy / len_ref;
               Vertex va = source(*ei,g);
               Vertex vb = target(*ei,g);
               if (vb != og[ov_singlestart].core_vertex && va != og[ov_singlegoal].core_vertex)
                  solver.add_edge(va, vb, weight_frac); // add forward edge a->b
               if (va != og[ov_singlestart].core_vertex && vb != og[ov_singlegoal].core_vertex)
                  solver.add_edge(vb, va, weight_frac); // add backwards edge b->a
            }
            _dur_selector_init += boost::chrono::high_resolution_clock::now() - time_selector_init_begin;
         
            return do_lazysp_b(g,
               pr_bgl::make_lazysp_incsp_astar<MyGraph,EPWlazyMap>(
                  boost::make_iterator_property_map(v_hvalues.begin(), get(boost::vertex_index,g)), // heuristic_map
                  boost::make_iterator_property_map(v_startpreds.begin(), get(boost::vertex_index,g)), // startpreds_map
                  boost::make_iterator_property_map(v_startdist.begin(), get(boost::vertex_index,g)), // startdist_map
                  boost::make_iterator_property_map(v_fvalues.begin(), get(boost::vertex_index,g)), // cost_map,
                  boost::make_iterator_property_map(v_colors.begin(), get(boost::vertex_index,g))), // color_map
               pr_bgl::lazysp_partition_all_matrix<MyGraph,EPWlazyMap>(
                  g, get(&EProps::w_lazy,g),
                  len_ref,
                  og[ov_singlestart].core_vertex,
                  og[ov_singlegoal].core_vertex,
                  true,
                  solver),
               epath);
         }
         case EVAL_TYPE_SP_INDICATOR_PROBABILITY:
            return do_lazysp_b(g,
               pr_bgl::make_lazysp_incsp_astar<MyGraph,EPWlazyMap>(
                  boost::make_iterator_property_map(v_hvalues.begin(), get(boost::vertex_index,g)), // heuristic_map
                  boost::make_iterator_property_map(v_startpreds.begin(), get(boost::vertex_index,g)), // startpreds_map
                  boost::make_iterator_property_map(v_startdist.begin(), get(boost::vertex_index,g)), // startdist_map
                  boost::make_iterator_property_map(v_fvalues.begin(), get(boost::vertex_index,g)), // cost_map,
                  boost::make_iterator_property_map(v_colors.begin(), get(boost::vertex_index,g))), // color_map
               pr_bgl::lazysp_selector_sp_indicator_probability<MyGraph,EPWlazyMap,ompl_lemur::IsEvaledMap>(
                  get(&EProps::w_lazy,g),
                  ompl_lemur::IsEvaledMap(*this),
                  1000, // nsamps
                  og[ov_singlestart].core_vertex,
                  og[ov_singlegoal].core_vertex,
                  0), // seed
               epath);
         }
      }
      else
      {
         // lpastar
         std::vector<Vertex> v_startpreds(num_vertices(eig));
         std::vector<double> v_gvalues(num_vertices(eig));
         std::vector<double> v_rhsvalues(num_vertices(eig));
         
         switch (_eval_type)
         {
         case EVAL_TYPE_FWD:
            return do_lazysp_b(g,
               pr_bgl::make_lazysp_incsp_lpastar(g,
                  og[ov_singlestart].core_vertex,
                  og[ov_singlegoal].core_vertex,
                  get(&EProps::w_lazy,g),
                  boost::make_iterator_property_map(v_hvalues.begin(), get(boost::vertex_index,g)), // heuristic_map
                  boost::make_iterator_property_map(v_startpreds.begin(), get(boost::vertex_index,g)), // startpreds_map
                  boost::make_iterator_property_map(v_gvalues.begin(), get(boost::vertex_index,g)), // gvalues_map
                  boost::make_iterator_property_map(v_rhsvalues.begin(), get(boost::vertex_index,g)), // rhsvalues_map
                  1.0e-9),
               pr_bgl::lazysp_selector_fwd(),
               epath);
         case EVAL_TYPE_REV:
            return do_lazysp_b(g,
               pr_bgl::make_lazysp_incsp_lpastar(g,
                  og[ov_singlestart].core_vertex,
                  og[ov_singlegoal].core_vertex,
                  get(&EProps::w_lazy,g),
                  boost::make_iterator_property_map(v_hvalues.begin(), get(boost::vertex_index,g)), // heuristic_map
                  boost::make_iterator_property_map(v_startpreds.begin(), get(boost::vertex_index,g)), // startpreds_map
                  boost::make_iterator_property_map(v_gvalues.begin(), get(boost::vertex_index,g)), // gvalues_map
                  boost::make_iterator_property_map(v_rhsvalues.begin(), get(boost::vertex_index,g)), // rhsvalues_map
                  1.0e-9),
               pr_bgl::lazysp_selector_rev(),
               epath);
         case EVAL_TYPE_ALT:
            return do_lazysp_b(g,
               pr_bgl::make_lazysp_incsp_lpastar(g,
                  og[ov_singlestart].core_vertex,
                  og[ov_singlegoal].core_vertex,
                  get(&EProps::w_lazy,g),
                  boost::make_iterator_property_map(v_hvalues.begin(), get(boost::vertex_index,g)), // heuristic_map
                  boost::make_iterator_property_map(v_startpreds.begin(), get(boost::vertex_index,g)), // startpreds_map
                  boost::make_iterator_property_map(v_gvalues.begin(), get(boost::vertex_index,g)), // gvalues_map
                  boost::make_iterator_property_map(v_rhsvalues.begin(), get(boost::vertex_index,g)), // rhsvalues_map
                  1.0e-9),
               pr_bgl::lazysp_selector_alt(),
               epath);
         case EVAL_TYPE_BISECT:
            return do_lazysp_b(g,
               pr_bgl::make_lazysp_incsp_lpastar(g,
                  og[ov_singlestart].core_vertex,
                  og[ov_singlegoal].core_vertex,
                  get(&EProps::w_lazy,g),
                  boost::make_iterator_property_map(v_hvalues.begin(), get(boost::vertex_index,g)), // heuristic_map
                  boost::make_iterator_property_map(v_startpreds.begin(), get(boost::vertex_index,g)), // startpreds_map
                  boost::make_iterator_property_map(v_gvalues.begin(), get(boost::vertex_index,g)), // gvalues_map
                  boost::make_iterator_property_map(v_rhsvalues.begin(), get(boost::vertex_index,g)), // rhsvalues_map
                  1.0e-9),
               pr_bgl::lazysp_selector_bisect(),
               epath);
         case EVAL_TYPE_FWD_EXPAND:
            return do_lazysp_b(g,
               pr_bgl::make_lazysp_incsp_lpastar(g,
                  og[ov_singlestart].core_vertex,
                  og[ov_singlegoal].core_vertex,
                  get(&EProps::w_lazy,g),
                  boost::make_iterator_property_map(v_hvalues.begin(), get(boost::vertex_index,g)), // heuristic_map
                  boost::make_iterator_property_map(v_startpreds.begin(), get(boost::vertex_index,g)), // startpreds_map
                  boost::make_iterator_property_map(v_gvalues.begin(), get(boost::vertex_index,g)), // gvalues_map
                  boost::make_iterator_property_map(v_rhsvalues.begin(), get(boost::vertex_index,g)), // rhsvalues_map
                  1.0e-9),
               pr_bgl::lazysp_selector_fwdexpand(),
               epath);
         case EVAL_TYPE_PARTITION_ALL:
         {
            double len_ref = 1.0/3.0;
            
            boost::chrono::high_resolution_clock::time_point time_selector_init_begin
               = boost::chrono::high_resolution_clock::now();
            printf("computing partition_all from scratch ...\n");
            pr_bgl::partition_all_matrix solver(num_vertices(g));
            // add all edges (both directions)
            std::pair<EdgeIter,EdgeIter> ep=edges(g);
            for (EdgeIter ei=ep.first; ei!=ep.second; ei++)
            {
               double weight_frac = g[*ei].w_lazy / len_ref;
               Vertex va = source(*ei,g);
               Vertex vb = target(*ei,g);
               if (vb != og[ov_singlestart].core_vertex && va != og[ov_singlegoal].core_vertex)
                  solver.add_edge(va, vb, weight_frac); // add forward edge a->b
               if (va != og[ov_singlestart].core_vertex && vb != og[ov_singlegoal].core_vertex)
                  solver.add_edge(vb, va, weight_frac); // add backwards edge b->a
            }
            _dur_selector_init += boost::chrono::high_resolution_clock::now() - time_selector_init_begin;
            
            return do_lazysp_b(g,
               pr_bgl::make_lazysp_incsp_lpastar(g,
                  og[ov_singlestart].core_vertex,
                  og[ov_singlegoal].core_vertex,
                  get(&EProps::w_lazy,g),
                  boost::make_iterator_property_map(v_hvalues.begin(), get(boost::vertex_index,g)), // heuristic_map
                  boost::make_iterator_property_map(v_startpreds.begin(), get(boost::vertex_index,g)), // startpreds_map
                  boost::make_iterator_property_map(v_gvalues.begin(), get(boost::vertex_index,g)), // gvalues_map
                  boost::make_iterator_property_map(v_rhsvalues.begin(), get(boost::vertex_index,g)), // rhsvalues_map
                  1.0e-9),
               pr_bgl::lazysp_partition_all_matrix<MyGraph,EPWlazyMap>(
                  g, get(&EProps::w_lazy,g),
                  1.0/3.0, // len_ref
                  og[ov_singlestart].core_vertex,
                  og[ov_singlegoal].core_vertex,
                  true,
                  solver),
               epath);
         }
         case EVAL_TYPE_SP_INDICATOR_PROBABILITY:
            return do_lazysp_b(g,
               pr_bgl::make_lazysp_incsp_lpastar(g,
                  og[ov_singlestart].core_vertex,
                  og[ov_singlegoal].core_vertex,
                  get(&EProps::w_lazy,g),
                  boost::make_iterator_property_map(v_hvalues.begin(), get(boost::vertex_index,g)), // heuristic_map
                  boost::make_iterator_property_map(v_startpreds.begin(), get(boost::vertex_index,g)), // startpreds_map
                  boost::make_iterator_property_map(v_gvalues.begin(), get(boost::vertex_index,g)), // gvalues_map
                  boost::make_iterator_property_map(v_rhsvalues.begin(), get(boost::vertex_index,g)), // rhsvalues_map
                  1.0e-9),
               pr_bgl::lazysp_selector_sp_indicator_probability<MyGraph,EPWlazyMap,ompl_lemur::IsEvaledMap>(
                  get(&EProps::w_lazy,g),
                  ompl_lemur::IsEvaledMap(*this),
                  1000, // nsamps
                  og[ov_singlestart].core_vertex,
                  og[ov_singlegoal].core_vertex,
                  0), // seed
               epath);
         }
      }
   }
   else if (_search_type == SEARCH_TYPE_DIJKSTRAS)
   {
      std::vector<Vertex> v_startpreds(num_vertices(eig));
      std::vector<double> v_startdist(num_vertices(eig));
      
      switch (_eval_type)
      {
      case EVAL_TYPE_FWD:
         return do_lazysp_b(g,
            pr_bgl::make_lazysp_incsp_dijkstra<MyGraph,EPWlazyMap>(
               boost::make_iterator_property_map(v_startpreds.begin(), get(boost::vertex_index,g)), // startpreds_map
               boost::make_iterator_property_map(v_startdist.begin(), get(boost::vertex_index,g))), // startdist_map
            pr_bgl::lazysp_selector_fwd(),
            epath);
      case EVAL_TYPE_REV:
         return do_lazysp_b(g,
            pr_bgl::make_lazysp_incsp_dijkstra<MyGraph,EPWlazyMap>(
               boost::make_iterator_property_map(v_startpreds.begin(), get(boost::vertex_index,g)), // startpreds_map
               boost::make_iterator_property_map(v_startdist.begin(), get(boost::vertex_index,g))), // startdist_map
            pr_bgl::lazysp_selector_rev(),
            epath);
      case EVAL_TYPE_ALT:
         return do_lazysp_b(g,
            pr_bgl::make_lazysp_incsp_dijkstra<MyGraph,EPWlazyMap>(
               boost::make_iterator_property_map(v_startpreds.begin(), get(boost::vertex_index,g)), // startpreds_map
               boost::make_iterator_property_map(v_startdist.begin(), get(boost::vertex_index,g))), // startdist_map
            pr_bgl::lazysp_selector_alt(),
            epath);
      case EVAL_TYPE_BISECT:
         return do_lazysp_b(g,
            pr_bgl::make_lazysp_incsp_dijkstra<MyGraph,EPWlazyMap>(
               boost::make_iterator_property_map(v_startpreds.begin(), get(boost::vertex_index,g)), // startpreds_map
               boost::make_iterator_property_map(v_startdist.begin(), get(boost::vertex_index,g))), // startdist_map
            pr_bgl::lazysp_selector_bisect(),
            epath);
      case EVAL_TYPE_FWD_EXPAND:
         return do_lazysp_b(g,
            pr_bgl::make_lazysp_incsp_dijkstra<MyGraph,EPWlazyMap>(
               boost::make_iterator_property_map(v_startpreds.begin(), get(boost::vertex_index,g)), // startpreds_map
               boost::make_iterator_property_map(v_startdist.begin(), get(boost::vertex_index,g))), // startdist_map
            pr_bgl::lazysp_selector_fwdexpand(),
            epath);
      case EVAL_TYPE_PARTITION_ALL:
      {
         double len_ref = 1.0/3.0;
         
         boost::chrono::high_resolution_clock::time_point time_selector_init_begin
            = boost::chrono::high_resolution_clock::now();
         printf("computing partition_all from scratch ...\n");
         pr_bgl::partition_all_matrix solver(num_vertices(g));
         // add all edges (both directions)
         std::pair<EdgeIter,EdgeIter> ep=edges(g);
         for (EdgeIter ei=ep.first; ei!=ep.second; ei++)
         {
            double weight_frac = g[*ei].w_lazy / len_ref;
            Vertex va = source(*ei,g);
            Vertex vb = target(*ei,g);
            if (vb != og[ov_singlestart].core_vertex && va != og[ov_singlegoal].core_vertex)
               solver.add_edge(va, vb, weight_frac); // add forward edge a->b
            if (va != og[ov_singlestart].core_vertex && vb != og[ov_singlegoal].core_vertex)
               solver.add_edge(vb, va, weight_frac); // add backwards edge b->a
         }
         _dur_selector_init += boost::chrono::high_resolution_clock::now() - time_selector_init_begin;
      
         return do_lazysp_b(g,
            pr_bgl::make_lazysp_incsp_dijkstra<MyGraph,EPWlazyMap>(
               boost::make_iterator_property_map(v_startpreds.begin(), get(boost::vertex_index,g)), // startpreds_map
               boost::make_iterator_property_map(v_startdist.begin(), get(boost::vertex_index,g))), // startdist_map
            pr_bgl::lazysp_partition_all_matrix<MyGraph,EPWlazyMap>(
               g, get(&EProps::w_lazy,g),
               1.0/3.0, // len_ref
               og[ov_singlestart].core_vertex,
               og[ov_singlegoal].core_vertex,
               true,
               solver),
            epath);
      }
      case EVAL_TYPE_SP_INDICATOR_PROBABILITY:
         return do_lazysp_b(g,
            pr_bgl::make_lazysp_incsp_dijkstra<MyGraph,EPWlazyMap>(
               boost::make_iterator_property_map(v_startpreds.begin(), get(boost::vertex_index,g)), // startpreds_map
               boost::make_iterator_property_map(v_startdist.begin(), get(boost::vertex_index,g))), // startdist_map
            pr_bgl::lazysp_selector_sp_indicator_probability<MyGraph,EPWlazyMap,ompl_lemur::IsEvaledMap>(
               get(&EProps::w_lazy,g),
               ompl_lemur::IsEvaledMap(*this),
               1000, // nsamps
               og[ov_singlestart].core_vertex,
               og[ov_singlegoal].core_vertex,
               0), // seed
            epath);
      }
   }
   else // _search_type == SEARCH_TYPE_INCBI
   {
      std::vector<Vertex> v_startpreds(num_vertices(eig));
      std::vector<double> v_startdist(num_vertices(eig));
      std::vector<double> v_startdistlookahead(num_vertices(eig));
      std::vector<Vertex> v_goalpreds(num_vertices(eig));
      std::vector<double> v_goaldist(num_vertices(eig));
      std::vector<double> v_goaldistlookahead(num_vertices(eig));
      
      switch (_eval_type)
      {
      case EVAL_TYPE_FWD:
         return do_lazysp_b(g,
            pr_bgl::make_lazysp_incsp_incbi<MyGraph,EPWlazyMap>(g,
               og[ov_singlestart].core_vertex,
               og[ov_singlegoal].core_vertex,
               get(&EProps::w_lazy,g),
               boost::make_iterator_property_map(v_startpreds.begin(), get(boost::vertex_index,g)), // startpreds_map
               boost::make_iterator_property_map(v_startdist.begin(), get(boost::vertex_index,g)), // gvalues_map
               boost::make_iterator_property_map(v_startdistlookahead.begin(), get(boost::vertex_index,g)), // rhsvalues_map
               boost::make_iterator_property_map(v_goalpreds.begin(), get(boost::vertex_index,g)), // startpreds_map
               boost::make_iterator_property_map(v_goaldist.begin(), get(boost::vertex_index,g)), // gvalues_map
               boost::make_iterator_property_map(v_goaldistlookahead.begin(), get(boost::vertex_index,g)), // rhsvalues_map
               get(&EProps::index, g), eig.edge_vector_map,
               1.0e-9,
               pr_bgl::incbi_visitor_null<Graph>()),
            pr_bgl::lazysp_selector_fwd(),
            epath);
      case EVAL_TYPE_REV:
         return do_lazysp_b(g,
            pr_bgl::make_lazysp_incsp_incbi<MyGraph,EPWlazyMap>(g,
               og[ov_singlestart].core_vertex,
               og[ov_singlegoal].core_vertex,
               get(&EProps::w_lazy,g),
               boost::make_iterator_property_map(v_startpreds.begin(), get(boost::vertex_index,g)), // startpreds_map
               boost::make_iterator_property_map(v_startdist.begin(), get(boost::vertex_index,g)), // gvalues_map
               boost::make_iterator_property_map(v_startdistlookahead.begin(), get(boost::vertex_index,g)), // rhsvalues_map
               boost::make_iterator_property_map(v_goalpreds.begin(), get(boost::vertex_index,g)), // startpreds_map
               boost::make_iterator_property_map(v_goaldist.begin(), get(boost::vertex_index,g)), // gvalues_map
               boost::make_iterator_property_map(v_goaldistlookahead.begin(), get(boost::vertex_index,g)), // rhsvalues_map
               get(&EProps::index, g), eig.edge_vector_map,
               1.0e-9,
               pr_bgl::incbi_visitor_null<Graph>()),
            pr_bgl::lazysp_selector_rev(),
            epath);
      case EVAL_TYPE_ALT:
         return do_lazysp_b(g,
            pr_bgl::make_lazysp_incsp_incbi<MyGraph,EPWlazyMap>(g,
               og[ov_singlestart].core_vertex,
               og[ov_singlegoal].core_vertex,
               get(&EProps::w_lazy,g),
               boost::make_iterator_property_map(v_startpreds.begin(), get(boost::vertex_index,g)), // startpreds_map
               boost::make_iterator_property_map(v_startdist.begin(), get(boost::vertex_index,g)), // gvalues_map
               boost::make_iterator_property_map(v_startdistlookahead.begin(), get(boost::vertex_index,g)), // rhsvalues_map
               boost::make_iterator_property_map(v_goalpreds.begin(), get(boost::vertex_index,g)), // startpreds_map
               boost::make_iterator_property_map(v_goaldist.begin(), get(boost::vertex_index,g)), // gvalues_map
               boost::make_iterator_property_map(v_goaldistlookahead.begin(), get(boost::vertex_index,g)), // rhsvalues_map
               get(&EProps::index, g), eig.edge_vector_map,
               1.0e-9,
               pr_bgl::incbi_visitor_null<Graph>()),
            pr_bgl::lazysp_selector_alt(),
            epath);
      case EVAL_TYPE_BISECT:
         return do_lazysp_b(g,
            pr_bgl::make_lazysp_incsp_incbi<MyGraph,EPWlazyMap>(g,
               og[ov_singlestart].core_vertex,
               og[ov_singlegoal].core_vertex,
               get(&EProps::w_lazy,g),
               boost::make_iterator_property_map(v_startpreds.begin(), get(boost::vertex_index,g)), // startpreds_map
               boost::make_iterator_property_map(v_startdist.begin(), get(boost::vertex_index,g)), // gvalues_map
               boost::make_iterator_property_map(v_startdistlookahead.begin(), get(boost::vertex_index,g)), // rhsvalues_map
               boost::make_iterator_property_map(v_goalpreds.begin(), get(boost::vertex_index,g)), // startpreds_map
               boost::make_iterator_property_map(v_goaldist.begin(), get(boost::vertex_index,g)), // gvalues_map
               boost::make_iterator_property_map(v_goaldistlookahead.begin(), get(boost::vertex_index,g)), // rhsvalues_map
               get(&EProps::index, g), eig.edge_vector_map,
               1.0e-9,
               pr_bgl::incbi_visitor_null<Graph>()),
            pr_bgl::lazysp_selector_bisect(),
            epath);
      case EVAL_TYPE_FWD_EXPAND:
         return do_lazysp_b(g,
            pr_bgl::make_lazysp_incsp_incbi<MyGraph,EPWlazyMap>(g,
               og[ov_singlestart].core_vertex,
               og[ov_singlegoal].core_vertex,
               get(&EProps::w_lazy,g),
               boost::make_iterator_property_map(v_startpreds.begin(), get(boost::vertex_index,g)), // startpreds_map
               boost::make_iterator_property_map(v_startdist.begin(), get(boost::vertex_index,g)), // gvalues_map
               boost::make_iterator_property_map(v_startdistlookahead.begin(), get(boost::vertex_index,g)), // rhsvalues_map
               boost::make_iterator_property_map(v_goalpreds.begin(), get(boost::vertex_index,g)), // startpreds_map
               boost::make_iterator_property_map(v_goaldist.begin(), get(boost::vertex_index,g)), // gvalues_map
               boost::make_iterator_property_map(v_goaldistlookahead.begin(), get(boost::vertex_index,g)), // rhsvalues_map
               get(&EProps::index, g), eig.edge_vector_map,
               1.0e-9,
               pr_bgl::incbi_visitor_null<Graph>()),
            pr_bgl::lazysp_selector_fwdexpand(),
            epath);
      case EVAL_TYPE_PARTITION_ALL:
      {
         double len_ref = 1.0/3.0;
         
         boost::chrono::high_resolution_clock::time_point time_selector_init_begin
            = boost::chrono::high_resolution_clock::now();
         printf("computing partition_all from scratch ...\n");
         pr_bgl::partition_all_matrix solver(num_vertices(g));
         // add all edges (both directions)
         std::pair<EdgeIter,EdgeIter> ep=edges(g);
         for (EdgeIter ei=ep.first; ei!=ep.second; ei++)
         {
            double weight_frac = g[*ei].w_lazy / len_ref;
            Vertex va = source(*ei,g);
            Vertex vb = target(*ei,g);
            if (vb != og[ov_singlestart].core_vertex && va != og[ov_singlegoal].core_vertex)
               solver.add_edge(va, vb, weight_frac); // add forward edge a->b
            if (va != og[ov_singlestart].core_vertex && vb != og[ov_singlegoal].core_vertex)
               solver.add_edge(vb, va, weight_frac); // add backwards edge b->a
         }
         _dur_selector_init += boost::chrono::high_resolution_clock::now() - time_selector_init_begin;
         
         return do_lazysp_b(g,
            pr_bgl::make_lazysp_incsp_incbi<MyGraph,EPWlazyMap>(g,
               og[ov_singlestart].core_vertex,
               og[ov_singlegoal].core_vertex,
               get(&EProps::w_lazy,g),
               boost::make_iterator_property_map(v_startpreds.begin(), get(boost::vertex_index,g)), // startpreds_map
               boost::make_iterator_property_map(v_startdist.begin(), get(boost::vertex_index,g)), // gvalues_map
               boost::make_iterator_property_map(v_startdistlookahead.begin(), get(boost::vertex_index,g)), // rhsvalues_map
               boost::make_iterator_property_map(v_goalpreds.begin(), get(boost::vertex_index,g)), // startpreds_map
               boost::make_iterator_property_map(v_goaldist.begin(), get(boost::vertex_index,g)), // gvalues_map
               boost::make_iterator_property_map(v_goaldistlookahead.begin(), get(boost::vertex_index,g)), // rhsvalues_map
               get(&EProps::index, g), eig.edge_vector_map,
               1.0e-9,
               pr_bgl::incbi_visitor_null<Graph>()),
            pr_bgl::lazysp_partition_all_matrix<MyGraph,EPWlazyMap>(
               g, get(&EProps::w_lazy,g),
               1.0/3.0, // len_ref
               og[ov_singlestart].core_vertex,
               og[ov_singlegoal].core_vertex,
               true,
               solver),
            epath);
      }
      case EVAL_TYPE_SP_INDICATOR_PROBABILITY:
         return do_lazysp_b(g,
            pr_bgl::make_lazysp_incsp_incbi<MyGraph,EPWlazyMap>(g,
               og[ov_singlestart].core_vertex,
               og[ov_singlegoal].core_vertex,
               get(&EProps::w_lazy,g),
               boost::make_iterator_property_map(v_startpreds.begin(), get(boost::vertex_index,g)), // startpreds_map
               boost::make_iterator_property_map(v_startdist.begin(), get(boost::vertex_index,g)), // gvalues_map
               boost::make_iterator_property_map(v_startdistlookahead.begin(), get(boost::vertex_index,g)), // rhsvalues_map
               boost::make_iterator_property_map(v_goalpreds.begin(), get(boost::vertex_index,g)), // startpreds_map
               boost::make_iterator_property_map(v_goaldist.begin(), get(boost::vertex_index,g)), // gvalues_map
               boost::make_iterator_property_map(v_goaldistlookahead.begin(), get(boost::vertex_index,g)), // rhsvalues_map
               get(&EProps::index, g), eig.edge_vector_map,
               1.0e-9,
               pr_bgl::incbi_visitor_null<Graph>()),
            pr_bgl::lazysp_selector_sp_indicator_probability<MyGraph,EPWlazyMap,ompl_lemur::IsEvaledMap>(
               get(&EProps::w_lazy,g),
               ompl_lemur::IsEvaledMap(*this),
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
ompl_lemur::LEMUR::solve(
   const ompl::base::PlannerTerminationCondition & ptc)
{
   if (!_roadmap)
      throw std::runtime_error("no roadmap set!");
   
   boost::chrono::high_resolution_clock::time_point time_total_begin;
   if (_do_timing)
   {
      _dur_roadmapgen = boost::chrono::high_resolution_clock::duration();
      _dur_roadmapinit = boost::chrono::high_resolution_clock::duration();
      _dur_lazysp = boost::chrono::high_resolution_clock::duration();
      _dur_search = boost::chrono::high_resolution_clock::duration();
      _dur_eval = boost::chrono::high_resolution_clock::duration();
      _dur_selector_init = boost::chrono::high_resolution_clock::duration();
      _dur_selector = boost::chrono::high_resolution_clock::duration();
      _dur_selector_notify = boost::chrono::high_resolution_clock::duration();
      time_total_begin = boost::chrono::high_resolution_clock::now();
   }
   
   // ensure roadmap is initialized
   if (!_roadmap->initialized)
      _roadmap->initialize();
   
   // ok, do some sweet sweet lazy search!
   
   if (out_degree(ov_singlestart,og) == 0)
      throw std::runtime_error("no start states passed!");
   if (out_degree(ov_singlegoal,og) == 0)
      throw std::runtime_error("no goal states passed!");
   
   // compute singleroot costs
   switch (_search_type)
   {
   case SEARCH_TYPE_ASTAR:
   case SEARCH_TYPE_LPASTAR:
      _singlestart_cost = 0.5 * _coeff_distance * space->getMaximumExtent();
      _singlegoal_cost = 1.0e-9;
      break;
   default:
      _singlestart_cost = 1.0e-9;
      _singlegoal_cost = 1.0e-9;
   }

   
   if (_search_type == SEARCH_TYPE_LPASTAR && _coeff_distance == 0.0)
      throw std::runtime_error("cannot use lpastar with 0 distance coefficient!");   
   
   if (os_alglog)
   {
      overlay_apply();
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
      overlay_unapply();
   }
   
   unsigned int num_batches = 0;

   // run batches of lazy search
   ompl::base::PlannerStatus ret = ompl::base::PlannerStatus::TIMEOUT;
   while (ptc() == false)
   {
      printf("considering %u batches ...\n", num_batches);
      
      // should we do a search?
      if (_num_batches_init <= num_batches)
      {
         overlay_apply();
         
         if (os_alglog)
         {
            *os_alglog << "subgraph " << _roadmap->num_batches_generated << std::endl;
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
         bool success;
         std::vector<Edge> epath;
         
         boost::chrono::high_resolution_clock::time_point time_lazysp_begin;
         if (_do_timing)
            time_lazysp_begin = boost::chrono::high_resolution_clock::now();
         
         if (num_batches < _roadmap->num_batches_generated)
         {
            printf("doing filtered lazy search ...\n");
            abort();
#if 0
            filter_num_batches filter(get(&EProps::batch,g), num_batches);
            boost::filtered_graph<Graph,filter_num_batches> fg(g, filter);
            success = do_lazysp_a(fg, epath);
#endif
         }
         else
         {
            success = do_lazysp_a(g, epath);
         }
         
         if (_do_timing)
            _dur_lazysp += boost::chrono::high_resolution_clock::now() - time_lazysp_begin;
         
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
         }
         
         overlay_unapply();
         
         if (success)
         {
            ret = ompl::base::PlannerStatus::EXACT_SOLUTION;
            break;
         }
      }
      
      // did we run out of batches?
      if (_max_batches <= num_batches)
      {
         ret = ompl::base::PlannerStatus::EXACT_SOLUTION;
         break;
      }
      
      // consider the next batch
      num_batches++;
      
      if (_roadmap->num_batches_generated < num_batches)
      {
         if (_roadmap->max_batches && _roadmap->max_batches <= _roadmap->num_batches_generated)
         {
            ret = ompl::base::PlannerStatus::EXACT_SOLUTION;
            break;
         }
         
         std::size_t new_batch = _roadmap->num_batches_generated;
         
         printf("densifying to batch [%lu] ...\n", new_batch);
         
         size_t v_from = num_vertices(eig);
         size_t e_from = num_edges(eig);
         
         // timing
         boost::chrono::high_resolution_clock::time_point time_roadmapgen_begin;
         if (_do_timing)
            time_roadmapgen_begin = boost::chrono::high_resolution_clock::now();
         
         // add a batch!
         //nn.sync();
         _roadmap->generate();
         
         // timing
         boost::chrono::high_resolution_clock::time_point time_roadmapinit_begin;
         if (_do_timing)
         {
            time_roadmapinit_begin = boost::chrono::high_resolution_clock::now();
            _dur_roadmapgen += time_roadmapinit_begin - time_roadmapgen_begin;
         }
         
         size_t v_to = num_vertices(eig);
         size_t e_to = num_edges(eig);
         m_subgraph_sizes.push_back(std::make_pair(v_to,e_to));
         
         // initialize new vertices/edges
         for (size_t vidx=v_from; vidx<v_to; vidx++)
            put(m_vidx_tag_map, vidx, 0);
         for (size_t eidx=e_from; eidx<e_to; eidx++)
         {
            Edge e = get(eig.edge_vector_map,eidx);
            g[e].num_edge_states = floor(g[e].distance/(2.0*check_radius));
            //edge_init_points(
            //   g[source(e,g)].state,
            //   g[target(e,g)].state,
            //   g[e].distance,
            //   g[e].edge_states);
            //g[e].edge_tags.resize(g[e].edge_states.size(), 0);
         }
         
         printf("LEMUR: loading from cache ...\n");
         
         // load new batch from cache
         tag_cache.load_begin();
         tag_cache.load_vertices(m_vidx_tag_map, v_from, v_to);
         tag_cache.load_edges(m_eidx_tags_map, e_from, e_to);
         tag_cache.load_end();
         
         // calculate w_lazy for these new edges
         for (size_t eidx=e_from; eidx<e_to; eidx++)
         {
            Edge e = get(eig.edge_vector_map,eidx);
            calculate_w_lazy(e);
         }
         
         // add new edges to overlay vertices
         double root_radius = _roadmap->root_radius(new_batch);
         
         // iterate over all non-anchor non-singleroot overlay vertices
         // connect them to the new batch
         OverVertexIter ovi, ovi_end;
         for (boost::tie(ovi,ovi_end)=vertices(og); ovi!=ovi_end; ovi++)
         {
            // skip singleroot vertices
            if (*ovi == ov_singlestart || *ovi == ov_singlegoal)
               continue;
            
            // skip anchor vertices (themselves already in core graph!)
            if (og[*ovi].core_vertex != boost::graph_traits<Graph>::null_vertex())
               continue;

            // consider all new core vertices
            VertexIter vi, vi_end;
            for (boost::tie(vi,vi_end)=vertices(g); vi!=vi_end; ++vi)
            {
               // core vertices in new batch only
               if (g[*vi].batch != (int)(new_batch))
                  continue;
               
               double dist = space->distance(og[*ovi].state, g[*vi].state);
               if (root_radius < dist)
                  continue;
               
               // add new anchor overlay vertex
               OverVertex ov_anchor = add_vertex(og);
               og[ov_anchor].core_vertex = *vi;
               og[ov_anchor].state = g[*vi].state;
               // no need to set core properties on anchors,
               // since this is just an anchor, wont be copied
               // we set state pointer though so search above will find it
               map_to_overlay[*vi] = ov_anchor;
            
               // add overlay edge from root to anchor
               OverEdge oe = add_edge(*ovi, ov_anchor, og).first;
               // add edge properties
               og[oe].distance = dist;
               og[oe].batch = g[*vi].batch;
               // w_lazy??
               // interior points, in bisection order
               og[oe].num_edge_states = floor(dist/(2.0*check_radius));
               //edge_init_points(og[*ovi].state, g[*vi].state, dist, og[oe].edge_states);
               //og[oe].edge_tags.resize(og[oe].edge_states.size(), 0);
               //og[e].tag = 0;
            }
         }
         
         if (_do_timing)
            _dur_roadmapinit += boost::chrono::high_resolution_clock::now() - time_roadmapinit_begin;
      }
   }
   
   if (_do_timing)
      _dur_total = boost::chrono::high_resolution_clock::now() - time_total_begin;

   return ret;
}

// solves only core vertices
void ompl_lemur::LEMUR::solve_all()
{
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
      // generate edge tags if they're not there
      if (g[*ei].num_edge_states != g[*ei].edge_states.size())
         edge_init_states(*ei);
      // check edges
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
}

void ompl_lemur::LEMUR::dump_graph(std::ostream & os_graph)
{
   overlay_apply();
   
   // dump graph
   // write it out to file
   boost::dynamic_properties props;
   props.property("state", ompl_lemur::make_rvstate_map_string_adaptor(
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
   
   overlay_unapply();
}

// saves only core vertices
void ompl_lemur::LEMUR::cache_save_all()
{
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
}

double ompl_lemur::LEMUR::getDurTotal()
{
   return boost::chrono::duration<double>(_dur_total).count();
}

double ompl_lemur::LEMUR::getDurRoadmapGen()
{
   return boost::chrono::duration<double>(_dur_roadmapgen).count();
}

double ompl_lemur::LEMUR::getDurRoadmapInit()
{
   return boost::chrono::duration<double>(_dur_roadmapinit).count();
}

double ompl_lemur::LEMUR::getDurLazySP()
{
   return boost::chrono::duration<double>(_dur_lazysp).count();
}

double ompl_lemur::LEMUR::getDurSearch()
{
   return boost::chrono::duration<double>(_dur_search).count();
}

double ompl_lemur::LEMUR::getDurEval()
{
   return boost::chrono::duration<double>(_dur_eval).count();
}

double ompl_lemur::LEMUR::getDurSelectorInit()
{
   return boost::chrono::duration<double>(_dur_selector_init).count();
}

double ompl_lemur::LEMUR::getDurSelector()
{
   return boost::chrono::duration<double>(_dur_selector).count();
}

double ompl_lemur::LEMUR::getDurSelectorNotify()
{
   return boost::chrono::duration<double>(_dur_selector_notify).count();
}

void ompl_lemur::LEMUR::overlay_apply()
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
      g[ecore].num_edge_states = og[eover].num_edge_states;
      g[ecore].edge_states = og[eover].edge_states;
      g[ecore].edge_tags = og[eover].edge_tags;
      //g[ecore].tag = og[eover].tag;
      calculate_w_lazy(ecore);
   }
}

void ompl_lemur::LEMUR::overlay_unapply()
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
      og[eover].num_edge_states = g[ecore].num_edge_states;
      og[eover].edge_states = g[ecore].edge_states;
      og[eover].edge_tags = g[ecore].edge_tags;
      //og[eover].tag = g[ecore].tag;
   }
   
   overlay_manager.unapply();
}

void ompl_lemur::LEMUR::edge_init_states(const Edge & e)
{
   // get endpoint vertices in consistent order
   Vertex va = source(e,g);
   Vertex vb = target(e,g);
   ompl::base::State * v1_state;
   ompl::base::State * v2_state;
   if (va < vb)
   {
      v1_state = g[va].state;
      v2_state = g[vb].state;
   }
   else
   {
      v1_state = g[vb].state;
      v2_state = g[va].state;
   }
   // now many interior points do we need?
   unsigned int n = g[e].num_edge_states;
   // allocate states
   g[e].edge_states.resize(n);
   for (unsigned int ui=0; ui<n; ui++)
      g[e].edge_states[ui] = space->allocState();
   // fill with interpolated states in bisection order
   const std::vector< std::pair<int,int> > & order = bisect_perm.get(n);
   for (unsigned int ui=0; ui<n; ui++)
      space->interpolate(v1_state, v2_state,
         1.0*(1+order[ui].first)/(n+1),
         g[e].edge_states[ui]);
   // allocate tags
   g[e].edge_tags.resize(n, 0);
}

void ompl_lemur::LEMUR::calculate_w_lazy(const Edge & e)
{
   Vertex va = source(e,g);
   Vertex vb = target(e,g);
   // special case for singleroot edges
   if (!g[va].state)
   {
      double singleroot_cost = (va == og[ov_singlestart].core_vertex) ? _singlestart_cost : _singlegoal_cost;
      if (effort_model.x_hat(g[vb].tag, g[vb].state) == std::numeric_limits<double>::infinity())
         g[e].w_lazy = std::numeric_limits<double>::infinity();
      else
         g[e].w_lazy = singleroot_cost + 0.5 * _coeff_checkcost * effort_model.p_hat(g[vb].tag, g[vb].state);
      return;
   }
   if (!g[vb].state)
   {
      double singleroot_cost = (vb == og[ov_singlestart].core_vertex) ? _singlestart_cost : _singlegoal_cost;
      if (effort_model.x_hat(g[va].tag, g[va].state) == std::numeric_limits<double>::infinity())
         g[e].w_lazy = std::numeric_limits<double>::infinity();
      else
         g[e].w_lazy = singleroot_cost + 0.5 * _coeff_checkcost * effort_model.p_hat(g[va].tag, g[va].state);
      return;
   }
   // ok, its a non-singleroot edge
   // is it known infeasible?
   unsigned int ui;
   for (ui=0; ui<g[e].edge_tags.size(); ui++)
      if (effort_model.x_hat(g[e].edge_tags[ui], g[e].edge_states[ui]) == std::numeric_limits<double>::infinity())
         break;
   if (ui<g[e].edge_states.size()
      || effort_model.x_hat(g[va].tag, g[va].state) == std::numeric_limits<double>::infinity()
      || effort_model.x_hat(g[vb].tag, g[vb].state) == std::numeric_limits<double>::infinity())
   {
      g[e].w_lazy = std::numeric_limits<double>::infinity();
      return;
   }
   // ok, it's not infeasible
   g[e].w_lazy = 0.0;
   g[e].w_lazy += _coeff_distance * g[e].distance;
   g[e].w_lazy += _coeff_batch * g[e].distance * g[e].batch;
   // half bounary vertices
   g[e].w_lazy += 0.5 * _coeff_checkcost * effort_model.p_hat(g[va].tag, g[va].state);
   g[e].w_lazy += 0.5 * _coeff_checkcost * effort_model.p_hat(g[vb].tag, g[vb].state);
   // interior states
   if (g[e].edge_tags.size() == 0)
   {
      // perhaps it's not yet generated?
      g[e].w_lazy += _coeff_checkcost * g[e].num_edge_states * effort_model.p_hat(0,0);
   }
   else
   {
      for (ui=0; ui<g[e].edge_tags.size(); ui++)
         g[e].w_lazy += _coeff_checkcost * effort_model.p_hat(g[e].edge_tags[ui], g[e].edge_states[ui]);
   }
}

bool ompl_lemur::LEMUR::isevaledmap_get(const Edge & e)
{
   // this directly calls the family effort model (distance not needed!)
   Vertex va = source(e, g);
   if (g[va].state && !effort_model.is_evaled(g[va].tag))
      return false;
   Vertex vb = target(e, g);
   if (g[vb].state && !effort_model.is_evaled(g[vb].tag))
      return false;
   if (g[e].num_edge_states != g[e].edge_states.size())
      return false;
   for (unsigned int ui=0; ui<g[e].edge_tags.size(); ui++)
      if (!effort_model.is_evaled(g[e].edge_tags[ui]))
         return false;
   return true;
}

double ompl_lemur::LEMUR::wmap_get(const Edge & e)
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

      // make sure we've generated internal edges
      if (g[e].num_edge_states != g[e].edge_states.size())
         edge_init_states(e);

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

double ompl_lemur::LEMUR::nn_dist(const Vertex & va, const Vertex & vb)
{
   return space->distance(g[va].state, g[vb].state);
}
