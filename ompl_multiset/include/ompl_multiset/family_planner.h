/* File: LazyPRM.h
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

// FOR NOW,
// status:
//  0 = unknown
//  1 = valid
//  2 = invalid

class FamilyPlanner : public ompl::base::Planner
{
public:

   // part 1: typedefs
   
   struct VProps
   {
      // from roadmap
      boost::shared_ptr<StateCon> state;
      int subgraph;
      bool is_shadow;
      // collision status (index or pointer?)
      int status;
   };
   struct EPropsPoint
   {
      boost::shared_ptr<StateCon> state;
      // collision status (index or pointer?)
      int status;
   };
   struct EProps
   {
      std::size_t index;
      // from roadmap
      double distance;
      int subgraph;
      // for lazysp
      double w_lazy;
      bool is_evaled;
      // interior points, in bisection order
      std::vector<EPropsPoint> points;
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
   
   // types of property maps to bundled properties
   // these are needed by roadmap
   typedef boost::property_map<Graph, boost::shared_ptr<StateCon> VProps::*>::type VPStateMap;
   typedef boost::property_map<Graph, int VProps::*>::type VPSubgraphMap;
   typedef boost::property_map<Graph, bool VProps::*>::type VPIsShadowMap;
   typedef boost::property_map<Graph, std::size_t EProps::*>::type EPIndexMap;
   typedef boost::property_map<Graph, double EProps::*>::type EPDistanceMap;
   typedef boost::property_map<Graph, int EProps::*>::type EPSubgraphMap;
   typedef boost::property_map<Graph, double EProps::*>::type EPWlazyMap;
   typedef boost::property_map<Graph, bool EProps::*>::type EPIsEvaledMap;
   
   // for indexing
   typedef pr_bgl::EdgeIndexedGraph<Graph, EPIndexMap> EdgeIndexedGraph;
   typedef boost::property_map<Graph, boost::vertex_index_t>::type VertexIndexMap;
   typedef boost::property_map<Graph, std::size_t EProps::*>::type EdgeIndexMap;
   typedef EdgeIndexedGraph::EdgeVectorMap EdgeVectorMap;
   
   // roadmap generator type
   typedef ompl_multiset::RoadmapGen<EdgeIndexedGraph,VPStateMap,EPDistanceMap,VPSubgraphMap,EPSubgraphMap,VPIsShadowMap> RoadmapGen;
   typedef boost::shared_ptr<RoadmapGen> RoadmapGenPtr;

   // roots overlay graph (used internally)
   // create an overlay graph for roots
   struct OverVProps
   {
      Vertex core_vertex;
      // like VProps
      boost::shared_ptr<StateCon> state;
      int subgraph;
      bool is_shadow;
      int status;
   };
   struct OverEProps
   {
      Edge core_edge;
      // like EProps
      double distance;
      int subgraph;
      double w_lazy;
      bool is_evaled;
      std::vector<EPropsPoint> points;
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

   const ompl::base::StateSpacePtr space;
   double check_radius; // this is half the standard resolution
   std::ostream & os_graph;
   std::ostream & os_alglog;

   // construct a graph
   Graph g;
   pr_bgl::EdgeIndexedGraph<Graph, EPIndexMap> eig;
   OverGraph og;
   
   pr_bgl::OverlayManager<EdgeIndexedGraph, OverGraph,
         boost::property_map<OverGraph, Vertex OverVProps::*>::type,
         boost::property_map<OverGraph, Edge OverEProps::*>::type>
      overlay_manager;
   
   OverVertex ov_start;
   OverVertex ov_goal;
   
   BisectPerm bisect_perm;

   // part 3: ompl methods

   FamilyPlanner(
      const ompl::base::StateSpacePtr space,
      const RoadmapGenPtr roadmap_gen,
      std::ostream & os_graph, std::ostream & os_alglog);
   
   ~FamilyPlanner(void);
   
   void setProblemDefinition(const ompl::base::ProblemDefinitionPtr & pdef);
   
   ompl::base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition & ptc);
   
   // part 4: private-ish methods
   
   void edge_init_points(const Edge & e);
   
   double wmap_get(const Edge & e);
};

// helper property map which delegates to FamilyPlanner::wmap_get()
class WMap
{
public:
   typedef boost::readable_property_map_tag category;
   typedef FamilyPlanner::Edge key_type;
   typedef double value_type;
   typedef double reference;
   FamilyPlanner & family_planner;
   WMap(FamilyPlanner & family_planner): family_planner(family_planner) {}
};
inline const double get(const WMap & wmap, const FamilyPlanner::Edge & e)
{
   return wmap.family_planner.wmap_get(e);
}

} // namespace ompl_multiset
