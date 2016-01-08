/* File: E8Roadmap.h
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

namespace ompl_multiset
{

// for now, this does HARD BATCHING with SUBGRAPH COSTS
// the core roadmap states are owned by the core roadmap
// the overlay roots and edges own their states (except anchors)
class E8Roadmap : public ompl::base::Planner
{
public:
   // part 1: typedefs
   
   struct VProps
   {
      // from roadmap
      ompl::base::State * state;
      int batch;
      bool is_shadow;
      // collision status (index or pointer?)
      size_t tag;
   };
   struct EProps
   {
      size_t index;
      // from roadmap
      double distance;
      int batch;
      // for lazysp; for current subset only!
      double w_lazy;
      // interior points, in bisection order
      // if edge_states.size() != num_edge_states,
      // then edge_states needs to be generated! (with tags = 0)
      size_t num_edge_states;
      std::vector< ompl::base::State * > edge_states;
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
   typedef boost::property_map<Graph, ompl::base::State * VProps::*>::type VPStateMap;
   typedef boost::property_map<Graph, int VProps::*>::type VPBatchMap;
   typedef boost::property_map<Graph, bool VProps::*>::type VPIsShadowMap;
   typedef boost::property_map<Graph, size_t VProps::*>::type VPTagMap;
   typedef boost::property_map<Graph, std::size_t EProps::*>::type EPIndexMap;
   typedef boost::property_map<Graph, double EProps::*>::type EPDistanceMap;
   typedef boost::property_map<Graph, int EProps::*>::type EPBatchMap;
   typedef boost::property_map<Graph, double EProps::*>::type EPWlazyMap;
   typedef boost::property_map<Graph, std::vector<size_t> EProps::*>::type EPTagsMap;
   
   // for indexing
   typedef pr_bgl::EdgeIndexedGraph<Graph, EPIndexMap> EdgeIndexedGraph;
   typedef boost::property_map<Graph, boost::vertex_index_t>::type VertexIndexMap;
   typedef boost::property_map<Graph, std::size_t EProps::*>::type EdgeIndexMap;
   typedef EdgeIndexedGraph::EdgeVectorMap EdgeVectorMap;
   
   // for tag cache object
   typedef pr_bgl::compose_property_map<VPTagMap,VertexIndexMap> VIdxTagMap;
   typedef pr_bgl::compose_property_map<EPTagsMap,EdgeVectorMap> EIdxTagsMap;
   
   // roadmap generator type
   //typedef ompl_multiset::NNOmplBatched<Graph,VPStateMap> NN;
   //typedef ompl_multiset::NNLinear<Graph,VPStateMap> NN;
   
   //typedef ompl::NearestNeighbors<Vertex> NN; // option A
   typedef ompl_multiset::NearestNeighborsLinearBGL<Graph,VPStateMap> NN; // option B
   typedef ompl_multiset::Roadmap<EdgeIndexedGraph,VPStateMap,EPDistanceMap,VPBatchMap,EPBatchMap,VPIsShadowMap,NN> Roadmap;
   typedef boost::shared_ptr<Roadmap> RoadmapPtr;

   // roots overlay graph (used internally)
   // create an overlay graph for roots
   struct OverVProps
   {
      Vertex core_vertex;
      // like VProps
      ompl::base::State * state;
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
      size_t num_edge_states;
      std::vector< ompl::base::State * > edge_states;
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

   struct filter_num_batches
   {
      EPBatchMap edge_batch_map;
      unsigned int num_batches;
      filter_num_batches() {}
      filter_num_batches(EPBatchMap edge_batch_map, unsigned int num_batches):
         edge_batch_map(edge_batch_map), num_batches(num_batches)
      {}
      bool operator()(const Edge & e) const
      {
         return get(edge_batch_map, e) < (int)num_batches;
      }
   };

private:
   // part 2: members

   ompl_multiset::EffortModel & effort_model;

   const RoadmapPtr roadmap_gen;
   std::vector< std::pair<size_t,size_t> > m_subgraph_sizes; // numverts,numedges (cumulative)

   const ompl::base::StateSpacePtr space;
   double check_radius; // this is half the standard resolution

   Graph g;
   pr_bgl::EdgeIndexedGraph<Graph, EPIndexMap> eig;
   OverGraph og;
   
   OverVertex ov_singlestart;
   OverVertex ov_singlegoal;
   
   // for overlay anchors
   std::map<Vertex, OverVertex> map_to_overlay;
   
   pr_bgl::OverlayManager<EdgeIndexedGraph, OverGraph,
         boost::property_map<OverGraph, Vertex OverVProps::*>::type,
         boost::property_map<OverGraph, Edge OverEProps::*>::type>
      overlay_manager;
   
   BisectPerm bisect_perm;
   
   TagCache<VIdxTagMap,EIdxTagsMap> & tag_cache;
   
   // note: the nearest neighbor object will only store core roadmap vertices
   // (not overlayed vertices), and will only be queried by the roadmap generator
   // (not when e.g. connecting overlay vertices to new things, etc)
   // eventually, it would be nice if we add/removed overlay vertices here as well!
   boost::shared_ptr<NN> nn;
   
   // parameters
   double _coeff_distance;
   double _coeff_checkcost;
   double _coeff_batch;
   
   bool _do_timing;
   bool _persist_roots;
   
   unsigned int _num_batches_init; // dont do a search on batches below this
   unsigned int _max_batches;
   
   enum
   {
      SEARCH_TYPE_DIJKSTRAS,
      SEARCH_TYPE_ASTAR
   } _search_type;
   
   enum
   {
      EVAL_TYPE_FWD,
      EVAL_TYPE_REV,
      EVAL_TYPE_ALT,
      EVAL_TYPE_BISECT,
      EVAL_TYPE_FWD_EXPAND,
      EVAL_TYPE_PARTITION_ALL,
      EVAL_TYPE_SP_INDICATOR_PROBABILITY
   } _eval_type;
   
public:
   std::ostream * os_alglog;

private:   
   // property maps
   VIdxTagMap m_vidx_tag_map;
   EIdxTagsMap m_eidx_tags_map;
   
   boost::chrono::high_resolution_clock::duration _dur_total;
   boost::chrono::high_resolution_clock::duration _dur_roadmapgen;
   boost::chrono::high_resolution_clock::duration _dur_search;
   boost::chrono::high_resolution_clock::duration _dur_eval;
   
   // part 3: ompl methods

public:
   E8Roadmap(
      const ompl::base::StateSpacePtr & space,
      ompl_multiset::EffortModel & effort_model,
      ompl_multiset::TagCache<VIdxTagMap,EIdxTagsMap> & tag_cache,
      const RoadmapPtr roadmap_gen);
   
   ~E8Roadmap(void);
   
   void setCoeffDistance(double coeff_distance);
   double getCoeffDistance() const;
   
   void setCoeffCheckcost(double coeff_checkcost);
   double getCoeffCheckcost() const;
   
   void setCoeffBatch(double coeff_batch);
   double getCoeffBatch() const;
   
   void setDoTiming(bool do_timing);
   bool getDoTiming() const;
   
   void setPersistRoots(bool persist_roots);
   bool getPersistRoots() const;
   
   void setNumBatchesInit(unsigned int max_batches);
   unsigned int getNumBatchesInit() const;
   
   void setMaxBatches(unsigned int max_batches);
   unsigned int getMaxBatches() const;
   
   void setSearchType(std::string search_type);
   std::string getSearchType() const;
   
   void setEvalType(std::string eval_type);
   std::string getEvalType() const;
   
   void setProblemDefinition(const ompl::base::ProblemDefinitionPtr & pdef);
   
   template <class MyGraph, class IncSP, class EvalStrategy>
   bool do_lazysp_b(MyGraph & graph, IncSP incsp, EvalStrategy evalstrategy, std::vector<Edge> & epath);
   
   template <class MyGraph>
   bool do_lazysp_a(MyGraph & graph, std::vector<Edge> & epath);
   
   ompl::base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition & ptc);
   
   void solve_all();
   
   void dump_graph(std::ostream & os_graph);
   
   void cache_save_all();
   
   double getDurTotal();
   double getDurRoadmapGen();
   double getDurSearch();
   double getDurEval();
   double getDurUnaccounted();
   
   // part 4: private-ish methods
   
   // assuming num_edge_states != edge_states.size(),
   // this generates the states and their tags (assumed tag=0)
   void edge_init_states(const Edge & e);
   
   void overlay_apply();
   void overlay_unapply();
   
   void calculate_w_lazy(const Edge & e);

   bool isevaledmap_get(const Edge & e);
   double wmap_get(const Edge & e);
   
   double nn_dist(const Vertex & va, const Vertex & vb);
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
