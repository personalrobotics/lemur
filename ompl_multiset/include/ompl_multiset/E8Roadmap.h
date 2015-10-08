/* File: family_planner.h
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

namespace ompl_multiset
{

// a lightweight scopedstate
struct StateCon
{
   const ompl::base::StateSpace * space;
   ompl::base::State * state;
   StateCon(ompl::base::StateSpace * space):
      space(space), state(space->allocState()) {}
   ~StateCon() { space->freeState(this->state); }
};
typedef boost::shared_ptr<StateCon> StateConPtr;

// for now, this does HARD BATCHING with SUBGRAPH COSTS
class E8Roadmap : public ompl::base::Planner
{
public:

   // part 1: typedefs
   
   struct VProps
   {
      // from roadmap
      boost::shared_ptr<StateCon> state;
      int batch;
      bool is_shadow;
      // collision status (index or pointer?)
      size_t tag;
   };
   struct EProps
   {
      std::size_t index;
      // from roadmap
      double distance;
      int batch;
      // for lazysp; for current subset only!
      double w_lazy;
      // interior points, in bisection order
      std::vector< boost::shared_ptr<StateCon> > edge_states;
      std::vector< size_t > edge_tags;
      //size_t tag; // mega tag?
   };
   typedef boost::adjacency_list<
      boost::vecS, // Edgelist ds, for per-vertex out-edges
      boost::vecS, // VertexList ds, for vertex set
      boost::undirectedS, // type of graph
      VProps, // internal (bundled) vertex properties
      EProps // internal (bundled) edge properties
      > Graph;
   
   typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
   typedef boost::graph_traits<Graph>::vertex_iterator VertexIter;
   typedef boost::graph_traits<Graph>::edge_descriptor Edge;
   typedef boost::graph_traits<Graph>::edge_iterator EdgeIter;
   typedef boost::graph_traits<Graph>::out_edge_iterator OutEdgeIter;
   
   // types of property maps to bundled properties
   // these are needed by roadmap
   typedef boost::property_map<Graph, boost::shared_ptr<StateCon> VProps::*>::type VPStateMap;
   typedef boost::property_map<Graph, int VProps::*>::type VPBatchMap;
   typedef boost::property_map<Graph, bool VProps::*>::type VPIsShadowMap;
   typedef boost::property_map<Graph, std::size_t EProps::*>::type EPIndexMap;
   typedef boost::property_map<Graph, double EProps::*>::type EPDistanceMap;
   typedef boost::property_map<Graph, int EProps::*>::type EPBatchMap;
   typedef boost::property_map<Graph, double EProps::*>::type EPWlazyMap;
   typedef boost::property_map<Graph, bool EProps::*>::type EPIsEvaledMap;
   
   // for indexing
   typedef pr_bgl::EdgeIndexedGraph<Graph, EPIndexMap> EdgeIndexedGraph;
   typedef boost::property_map<Graph, boost::vertex_index_t>::type VertexIndexMap;
   typedef boost::property_map<Graph, std::size_t EProps::*>::type EdgeIndexMap;
   typedef EdgeIndexedGraph::EdgeVectorMap EdgeVectorMap;
   
   // roadmap generator type
   typedef ompl_multiset::Roadmap<EdgeIndexedGraph,VPStateMap,EPDistanceMap,VPBatchMap,EPBatchMap,VPIsShadowMap> Roadmap;
   typedef boost::shared_ptr<Roadmap> RoadmapPtr;

   // roots overlay graph (used internally)
   // create an overlay graph for roots
   struct OverVProps
   {
      Vertex core_vertex;
      // like VProps
      boost::shared_ptr<StateCon> state;
      int batch;
      bool is_shadow;
      size_t tag;
   };
   struct OverEProps
   {
      Edge core_edge;
      // like EProps
      double distance;
      int batch;
      double w_lazy;
      bool is_evaled;
      std::vector< boost::shared_ptr<StateCon> > edge_states;
      std::vector< size_t > edge_tags;
      size_t tag; // mega tag?
   };
   typedef boost::adjacency_list<
      boost::vecS, // Edgelist ds, for per-vertex out-edges
      boost::listS, // VertexList ds, for vertex set
      boost::undirectedS, // type of graph
      OverVProps, // internal (bundled) vertex properties
      OverEProps // internal (bundled) edge properties
      > OverGraph;
   typedef boost::graph_traits<OverGraph>::vertex_descriptor OverVertex;
   typedef boost::graph_traits<OverGraph>::vertex_iterator OverVertexIter;
   typedef boost::graph_traits<OverGraph>::edge_descriptor OverEdge;
   typedef boost::graph_traits<OverGraph>::edge_iterator OverEdgeIter;

   // part 2: members

   ompl_multiset::EffortModel & effort_model;

   const RoadmapPtr roadmap_gen;

   const ompl::base::StateSpacePtr space;
   double check_radius; // this is half the standard resolution
   std::ostream * os_alglog;

   Graph g;
   pr_bgl::EdgeIndexedGraph<Graph, EPIndexMap> eig;
   OverGraph og;
   
   pr_bgl::OverlayManager<EdgeIndexedGraph, OverGraph,
         boost::property_map<OverGraph, Vertex OverVProps::*>::type,
         boost::property_map<OverGraph, Edge OverEProps::*>::type>
      overlay_manager;
   
   OverVertex ov_singlestart;
   OverVertex ov_singlegoal;
   
   BisectPerm bisect_perm;
   
   //unsigned int num_batches;
   
   TagCache & tag_cache;
   
   // parameters
   double coeff_checkcost;
   double coeff_distance;
   double coeff_batch;

   // part 3: ompl methods

   E8Roadmap(
      const ompl::base::SpaceInformationPtr & si,
      ompl_multiset::EffortModel & effort_model,
      ompl_multiset::TagCache & tag_cache,
      const RoadmapPtr roadmap_gen,
      unsigned int num_batches);
   
   ~E8Roadmap(void);
   
   void setProblemDefinition(const ompl::base::ProblemDefinitionPtr & pdef);
   
   ompl::base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition & ptc);
   
   void solve_all();
   
   void dump_graph(std::ostream & os_graph);
   
   void cache_load_all();
   void cache_save_all();
   
   // part 4: private-ish methods
   
   void edge_init_points(ompl::base::State * va_state, ompl::base::State * vb_state,
      double e_distance, std::vector< boost::shared_ptr<StateCon> > & edge_states);
   
   void overlay_apply();
   void overlay_unapply();
   
   void calculate_w_lazy(const Edge & e);

   bool isevaledmap_get(const Edge & e);
   double wmap_get(const Edge & e);

};

// helper property map which delegates to FamilyPlanner::isevaledmap_get()
class IsEvaledMap
{
public:
   typedef boost::readable_property_map_tag category;
   typedef E8Roadmap::Edge key_type;
   typedef bool value_type;
   typedef bool reference;
   E8Roadmap & e8_roadmap;
   IsEvaledMap(E8Roadmap & e8_roadmap): e8_roadmap(e8_roadmap) {}
};
inline const double get(const IsEvaledMap & isevaledmap, const E8Roadmap::Edge & e)
{
   return isevaledmap.e8_roadmap.isevaledmap_get(e);
}

// helper property map which delegates to FamilyPlanner::wmap_get()
class WMap
{
public:
   typedef boost::readable_property_map_tag category;
   typedef E8Roadmap::Edge key_type;
   typedef double value_type;
   typedef double reference;
   E8Roadmap & e8_roadmap;
   WMap(E8Roadmap & e8_roadmap): e8_roadmap(e8_roadmap) {}
};
inline const double get(const WMap & wmap, const E8Roadmap::Edge & e)
{
   return wmap.e8_roadmap.wmap_get(e);
}

} // namespace ompl_multiset
