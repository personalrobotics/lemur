/*! \file LEMUR.h
 * \author Chris Dellin <cdellin@gmail.com>
 * \copyright 2015 Carnegie Mellon University
 * \copyright License: BSD
 */

namespace ompl_lemur
{

/*! \brief This uses pr_bgl::lazysp() and pr_bgl::incbi !
 * 
 * for now, this does HARD BATCHING with SUBGRAPH COSTS
 * 
 * the core roadmap states are owned by the core roadmap
 * 
 * the overlay roots and edges own their states (except anchors)
 */
class LEMUR : public ompl::base::Planner
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
      //std::vector< size_t > edge_tags;
      size_t edge_tag; // this is the tag for ALL INTERNAL STATES
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
   //typedef boost::property_map<Graph, std::vector<size_t> EProps::*>::type EPTagsMap;
   typedef boost::property_map<Graph, size_t EProps::*>::type EPTagMap;
   
   // for indexing
   typedef pr_bgl::edge_indexed_graph<Graph, EPIndexMap> EdgeIndexedGraph;
   typedef boost::property_map<Graph, boost::vertex_index_t>::type VertexIndexMap;
   typedef boost::property_map<Graph, std::size_t EProps::*>::type EdgeIndexMap;
   typedef EdgeIndexedGraph::EdgeVectorMap EdgeVectorMap;
   
   // for tag cache object
   typedef pr_bgl::compose_property_map<VPTagMap,VertexIndexMap> VIdxTagMap;
   typedef pr_bgl::compose_property_map<EPTagMap,EdgeVectorMap> EIdxTagsMap;
   
   // roadmap generator type
   //typedef ompl_lemur::NNOmplBatched<Graph,VPStateMap> NN;
   //typedef ompl_lemur::NNLinear<Graph,VPStateMap> NN;
   
   //typedef ompl::NearestNeighbors<Vertex> NN; // option A
   typedef ompl_lemur::NearestNeighborsLinearBGL<Graph,VPStateMap> NN; // option B

   typedef ompl_lemur::RoadmapArgs<EdgeIndexedGraph,VPStateMap,EPDistanceMap,VPBatchMap,EPBatchMap,VPIsShadowMap,EdgeVectorMap,NN> RoadmapArgs;

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
      //std::vector< size_t > edge_tags;
      size_t edge_tag; // mega tag?
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

   ompl_lemur::UtilityCheckerPtr _utility_checker;

   std::map<std::string, boost::function<Roadmap<RoadmapArgs> * (RoadmapArgs args)> > _roadmap_registry;

   std::string _roadmap_type;
   std::vector<std::string> _roadmap_params;

   boost::shared_ptr< Roadmap<RoadmapArgs> > _roadmap;
   
public:

   // danger, don't change this during planning!
   boost::shared_ptr< TagCache<VIdxTagMap,EIdxTagsMap> > _tag_cache;
   
private:

   std::vector< std::pair<size_t,size_t> > _subgraph_sizes; // numverts,numedges (cumulative)

   const ompl::base::StateSpacePtr space;
   double check_radius; // this is half the standard resolution

   Graph g;
   pr_bgl::edge_indexed_graph<Graph, EPIndexMap> eig;
   OverGraph og;
   
   OverVertex ov_singlestart;
   OverVertex ov_singlegoal;
   
   // hack to account for singleroot edges given 
   // undirected representation (breaks heuristic consistency)
   double _singlestart_cost;
   double _singlegoal_cost;
   
   // for overlay anchors
   std::map<Vertex, OverVertex> map_to_overlay;
   
   pr_bgl::overlay_manager<EdgeIndexedGraph, OverGraph,
         boost::property_map<OverGraph, Vertex OverVProps::*>::type,
         boost::property_map<OverGraph, Edge OverEProps::*>::type>
      overlay_manager;
   
   BisectPerm bisect_perm;
   
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
   
   unsigned int _num_batches_searched; // the number of batches currently being searched
   
   bool _solve_all;
   
   enum
   {
      SEARCH_TYPE_DIJKSTRAS,
      SEARCH_TYPE_ASTAR,
      SEARCH_TYPE_LPASTAR,
      SEARCH_TYPE_RLPASTAR,
      SEARCH_TYPE_INCBI,
      SEARCH_TYPE_WINCBI
   } _search_type;
   
   double _search_incbi_heur_interp;
   
   enum
   {
      SEARCH_INCBI_BALANCER_TYPE_DISTANCE,
      SEARCH_INCBI_BALANCER_TYPE_CARDINALITY
   } _search_incbi_balancer_type;
   
   double _search_incbi_balancer_goalfrac;
   
   enum
   {
      EVAL_TYPE_FWD,
      EVAL_TYPE_REV,
      EVAL_TYPE_ALT,
      EVAL_TYPE_EVEN,
      EVAL_TYPE_BISECT,
      EVAL_TYPE_FWD_EXPAND,
#if 0
      EVAL_TYPE_PARTITION_ALL,
      EVAL_TYPE_SP_INDICATOR_PROBABILITY
#endif
   } _eval_type;
   
public:
   std::ostream * os_alglog;

private:   
   // property maps
   VIdxTagMap _vidx_tag_map;
   EIdxTagsMap _eidx_tags_map;
   
   boost::chrono::high_resolution_clock::duration _dur_total;
   boost::chrono::high_resolution_clock::duration _dur_roadmapgen;
   boost::chrono::high_resolution_clock::duration _dur_roadmapinit;
   boost::chrono::high_resolution_clock::duration _dur_lazysp;
   boost::chrono::high_resolution_clock::duration _dur_search;
   boost::chrono::high_resolution_clock::duration _dur_eval;
   boost::chrono::high_resolution_clock::duration _dur_selector_init;
   boost::chrono::high_resolution_clock::duration _dur_selector;
   boost::chrono::high_resolution_clock::duration _dur_selector_notify;
   
   // part 3: ompl methods

public:
   LEMUR(const ompl::base::SpaceInformationPtr & si);
   ~LEMUR(void);
   
   template <template<class> class RoadmapTemplate>
   void registerRoadmapType(std::string roadmap_type)
   {
      registerRoadmapType(roadmap_type, ompl_lemur::RoadmapFactory<RoadmapArgs,RoadmapTemplate>());
   }
   
   void registerRoadmapType(std::string roadmap_type,
      boost::function<Roadmap<RoadmapArgs> * (RoadmapArgs args)> factory);
      
   boost::shared_ptr< const Roadmap<RoadmapArgs> > getRoadmap();
   
   void setRoadmapType(std::string roadmap_type);
   std::string getRoadmapType() const;
   
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
   
   void setSolveAll(bool solve_all);
   bool getSolveAll() const;
   
   void setSearchType(std::string search_type);
   std::string getSearchType() const;
   
   void setSearchIncbiHeurInterp(double search_incbi_heur_interp);
   double getSearchIncbiHeurInterp() const;
   
   void setSearchIncbiBalancerType(std::string search_incbi_balancer_type);
   std::string getSearchIncbiBalancerType() const;
   
   void setSearchIncbiBalancerGoalfrac(double search_incbi_balancer_goalfrac);
   double getSearchIncbiBalancerGoalfrac() const;
   
   void setEvalType(std::string eval_type);
   std::string getEvalType() const;
   
   // this is guaranteed to initialize the roadmap
   void setProblemDefinition(const ompl::base::ProblemDefinitionPtr & pdef);
   
   template <class MyGraph, class IncSP, class EvalStrategy>
   bool do_lazysp_c(MyGraph & graph, std::vector<Edge> & epath, IncSP incsp, EvalStrategy evalstrategy);
   
   template <class MyGraph, class IncSP>
   bool do_lazysp_b(MyGraph & graph, std::vector<Edge> & epath, IncSP incsp);
   
   template <class MyGraph>
   bool do_lazysp_a(MyGraph & graph, std::vector<Edge> & epath);
   
   // this fails if problem definition not set
   ompl::base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition & ptc);
   
   void dump_graph(std::ostream & os_graph);
   
   void saveTagCache();
   
   double getDurTotal();
   double getDurRoadmapGen();
   double getDurRoadmapInit();
   double getDurLazySP();
   double getDurSearch();
   double getDurEval();
   double getDurSelectorInit();
   double getDurSelector();
   double getDurSelectorNotify();
   
   // part 4: private methods
private:
   
   // assuming num_edge_states != edge_states.size(),
   // this generates the states and their tags (assumed tag=0)
   void edge_init_states(const Edge & e);
   
   void overlay_apply();
   void overlay_unapply();
   
   void calculate_w_lazy(const Edge & e);

   // these are public so the property map wrappers can access them;
   // instead, i should probable move those classes inside LEMUR
public:
   bool isevaledmap_get(const Edge & e);
   std::pair<double, std::vector<Edge> > wmap_get(const Edge & e);
   
private:
   double nn_dist(const Vertex & va, const Vertex & vb);
};

// helper property map which delegates to FamilyPlanner::isevaledmap_get()
class IsEvaledMap
{
public:
   typedef boost::readable_property_map_tag category;
   typedef LEMUR::Edge key_type;
   typedef bool value_type;
   typedef bool reference;
   LEMUR & lemur;
   IsEvaledMap(LEMUR & lemur): lemur(lemur) {}
};
inline const double get(const IsEvaledMap & isevaledmap, const LEMUR::Edge & e)
{
   return isevaledmap.lemur.isevaledmap_get(e);
}

// helper property map which delegates to FamilyPlanner::wmap_get()
class WMap
{
public:
   typedef boost::readable_property_map_tag category;
   typedef LEMUR::Edge key_type;
   typedef std::pair<double, std::vector<LEMUR::Edge> > value_type;
   typedef std::pair<double, std::vector<LEMUR::Edge> > reference;
   LEMUR & lemur;
   WMap(LEMUR & lemur): lemur(lemur) {}
};
inline const std::pair<double, std::vector<LEMUR::Edge> > get(const WMap & wmap, const LEMUR::Edge & e)
{
   return wmap.lemur.wmap_get(e);
}

} // namespace ompl_lemur
