#include <cstdio>
#include <functional>
#include <list>
#include <queue>
#include <string>
#include <vector>
#include <boost/function.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/Planner.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>

#include <ompl_multiset/Roadmap.h>
#include <ompl_multiset/Cache.h>
#include <ompl_multiset/MultiSetPRM.h>

//#define DEBUG_DUMPFILE 1

namespace
{

// from calc_order_endgood
std::vector<int> perm_edgestates(int n)
{
   int i;
   int last_true;
   int max_i;
   int max_val;
   std::vector<int> perm;
   std::vector<bool> done(n, false);
   std::vector<int> dist(n);
   for (;;)
   {
      last_true = -1;
      for (i=0; i<n; i++)
      {
         if (done[i]) last_true = i;
         dist[i] = (i-last_true);
      }
      last_true = n;
      for (i=n-1; i>=0; i--)
      {
         if (done[i]) last_true = i;
         dist[i] = (last_true-i) < dist[i] ? (last_true-i) : dist[i];
      }
      max_val = 0;
      for (i=0; i<n; i++) if (max_val < dist[i])
      {
         max_val = dist[i];
         max_i = i;
      }
      if (!max_val)
         break;
      perm.push_back(max_i);
      done[max_i] = true;
   }
   return perm;
}

class P : public ompl_multiset::MultiSetPRM
{
private:
   // types
   enum CheckStatus
   {
      STATUS_UNKNOWN,
      STATUS_VALID,
      STATUS_INVALID
   };
   // a checkable state (1 per vertex, >=0 per edge)
   struct EdgeState
   {
      ompl::base::State * s;
      std::vector<enum CheckStatus> statuses; // one per si
   };
   struct CheckEstimate // one per si, optimistic (-:
   {
      CheckEstimate(): cost_dirty(true) {}
      bool cost_dirty;
      double cost_remaining;
   };
   struct VertexProperties
   {
      ompl::base::State * s;
      std::vector<CheckEstimate> estimates;
      std::vector<enum CheckStatus> statuses;
   };
   struct EdgeProperties
   {
      double euc_dist;
      std::vector<EdgeState> edgestates;
      std::vector<CheckEstimate> estimates;
      std::vector<enum CheckStatus> statuses;
   };
   typedef boost::adjacency_list<
      boost::vecS, // Edgelist ds, for per-vertex out-edges
      boost::vecS, // VertexList ds, for vertex set
      boost::undirectedS, // type of graph
      VertexProperties, EdgeProperties // bundled internal properties
      > Graph;
   typedef boost::graph_traits<Graph> GraphTypes;
   typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
   typedef boost::graph_traits<Graph>::vertex_iterator VertexIter;
   typedef boost::graph_traits<Graph>::edge_descriptor Edge;
   typedef boost::graph_traits<Graph>::edge_iterator EdgeIter;
   typedef boost::graph_traits<Graph>::out_edge_iterator EdgeOutIter;
   
   // just kept in a map
   struct ProbDefData
   {
      std::vector<Vertex> v_starts;
      std::vector<Vertex> v_goals;
   };
   // kept in a vector,
   // also with map for cfree index [ci]
   struct Cfree
   {
      Cfree(ompl::base::SpaceInformationPtr si, std::string name, double check_cost):
         si(si), name(name), check_cost(check_cost) {}
      // given stuff
      ompl::base::SpaceInformationPtr si;
      std::string name;
      double check_cost;
   };
   
   struct DijkType
   {
      DijkType(double v_cost, Vertex v, Vertex v_parent, Edge e_to_parent):
         v_cost(v_cost), v(v), v_parent(v_parent), e_to_parent(e_to_parent) {}
      double v_cost;
      Vertex v;
      Vertex v_parent;
      Edge e_to_parent;
      bool operator < (const DijkType rhs) const // for sorting on the open priority queue
      {
        return v_cost > rhs.v_cost;
      }
   };
   
   struct Inclusion
   {
      Inclusion(const ompl::base::SpaceInformationPtr si_superset,
         const ompl::base::SpaceInformationPtr si_subset):
         si_superset(si_superset), si_subset(si_subset) {}
      ompl::base::SpaceInformationPtr si_superset;
      ompl::base::SpaceInformationPtr si_subset;
   };
   struct Intersection
   {
      Intersection(
         const ompl::base::SpaceInformationPtr si_intersection,
         const std::vector<ompl::base::SpaceInformationPtr> si_supersets):
         si_intersection(si_intersection), si_supersets(si_supersets) {}
      ompl::base::SpaceInformationPtr si_intersection;
      std::vector<ompl::base::SpaceInformationPtr> si_supersets;
   };

public:

   P(const ompl::base::StateSpacePtr space,
      const ompl_multiset::RoadmapPtr roadmap,
      const ompl::base::SpaceInformationPtr si_bogus);
   ~P(void);

   // ompl planner interface
   void setProblemDefinition(const ompl::base::ProblemDefinitionPtr & pdef);
   ompl::base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition & ptc);
   void clear(void);
   
   // parameters
   void set_interroot_radius(double interroot_radius);
   void set_lambda(double lambda);
   
   void set_dumpfile(const char * dumpfile);
   
   void add_cfree(const ompl::base::SpaceInformationPtr si, std::string name, double check_cost);

   void add_inclusion(
      const ompl::base::SpaceInformationPtr si_superset,
      const ompl::base::SpaceInformationPtr si_subset);
   void add_intersection(
      const ompl::base::SpaceInformationPtr si_a,
      const ompl::base::SpaceInformationPtr si_b,
      const ompl::base::SpaceInformationPtr si_intersection);
   void add_intersection(
      const ompl::base::SpaceInformationPtr si_intersection,
      const std::vector<ompl::base::SpaceInformationPtr> si_supersets);
   
   // extras
   void use_num_subgraphs(unsigned int num);
   void eval_everything(const ompl::base::SpaceInformationPtr si);
   
   void cache_load(const ompl_multiset::CachePtr cache);
   void cache_save(const ompl_multiset::CachePtr cache);

private:
   // private methods
   
   // given the relations (inclusions and intersections),
   // for each cfree, calcualte which other cfrees which
   // might potentially imply something about its membership
   // actually this does something different
   void recalc_truthtable();
   
   // given the current statuses and target cfree,
   // compute (or use from cache)
   // the cheapest set of checks which would, if they return as desired,
   // prove valid membership in the target cfree
   const std::pair<double, std::vector<std::pair<int,bool> > > & get_optimistic_checks(
      std::vector<enum CheckStatus> & statuses, int ci_target);
   
   // given the current problem,
   // try to get a path!
   // return pair:
   //   first: optimistic feasible path found
   //   second: at lease one start and goal are optimistic feasible
   enum DijkResult
   {
      DIJK_SUCCESS,
      DIJK_NO_PATH,
      DIJK_NO_STARTS,
      DIJK_NO_GOALS
   };
   enum DijkResult dijkstras_bidirectional(ProbDefData & pdefdata,
      std::list<Vertex> & path_vs, std::list<Edge> & path_es, double & cost_estimate);
   
   bool isvalid_path(std::list<Vertex> & path_vs, std::list<Edge> & path_es);
   
   void add_subgraph();
   
   // this takes ownership of the state
   // does not add any edges!
   Vertex add_vertex(ompl::base::State * s);
   
   Edge add_edge(Vertex va, Vertex vb);
   
   // assume vroot isnt in this->v_roots[] yet
   void add_initial_root_edges(Vertex vroot);
   
   // ok, this is where the magic happens!
   // hopefully this will be inlined
   // update the checkablestate's checkestimates[sii]
   // given the current validity mask across all si's
   void update_vertex_estimate(struct VertexProperties & vp, unsigned int ci);
   void update_edge_estimate(struct EdgeProperties & ep, unsigned int ci);
   
   // these two correctly set dirty bits
   bool isvalid_vertex(struct VertexProperties & vp, unsigned int ci_target);
   bool isvalid_edge(struct EdgeProperties & ep, unsigned int ci_target, Edge e);
   
   // this doesnt set estimate dirty bits or anything
   void implies_statuses(std::vector<enum CheckStatus> & statuses);
   
   const std::vector<int> & get_edgeperm(int n);

   
   // private members
   const ompl_multiset::RoadmapPtr roadmap;
   
   // map from roadmap v/e ids to my boost vertex/edge descriptors
   unsigned int roadmap_subgraphs_used;
   std::vector<Vertex> roadmap_vertices;
   std::vector<Edge> roadmap_edges;
   
   std::vector<Vertex> v_roots;
   
   // PRM parameters
   double interroot_radius;
   double lambda;
   
   double resolution; // from space
   
   // we have a graph g
   Graph g;
   
   // this planner can only work with a single space,
   // provided on construction
   // (of course we can handle many different spaceinformations,
   //  as long as they all share the same space and parameters
   ompl::base::StateSpacePtr space;
   
   // the current problem definition
   ompl::base::ProblemDefinitionPtr pdef;
   int pdef_ci;
   
   // we should maintain info from past probdefs
   // e.g. which vertices correspond to the goals
   // so that we can switch back to them quickly
   std::map<ompl::base::ProblemDefinitionPtr, ProbDefData> pdefs;
   
   // known cfrees and their costs
   std::vector<Cfree> cfrees;
   std::map<ompl::base::SpaceInformation*, unsigned int> si_to_ci;
   
   // maintain list of si relations
   // (inclusions, intersections, etc)
   std::vector<Inclusion> inclusions;
   std::vector<Intersection> intersections;
   
   // this is recalculated whenever cfrees or relations are added
   std::list< std::vector<bool> > truthtable;
   
   // cache of optimistic checks
   // this is cleared whenever the truthtable is recalculated
   std::map<
      std::pair< std::vector<enum CheckStatus>, int >,
      std::pair<double, std::vector<std::pair<int,bool> > >
      > optimistic_checks;
   
   // cache of edge permutations
   std::map<int, const std::vector<int> > edgeperms;
   
#ifdef DEBUG_DUMPFILE
   FILE * dump_fp;
#endif
};

} // anonymous namespace

// static creation method
ompl_multiset::MultiSetPRM * ompl_multiset::MultiSetPRM::create(
   const ompl::base::StateSpacePtr space,
   const ompl_multiset::RoadmapPtr roadmap)
{
   ompl::base::SpaceInformationPtr si_bogus(new ompl::base::SpaceInformation(space));
   si_bogus->setStateValidityChecker(
      ompl::base::StateValidityCheckerPtr(
         new ompl::base::AllValidStateValidityChecker(si_bogus)
      )
   );
   si_bogus->setup();
   return new P(space, roadmap, si_bogus);
}

P::P(const ompl::base::StateSpacePtr space,
      const ompl_multiset::RoadmapPtr roadmap,
      const ompl::base::SpaceInformationPtr si_bogus):
   ompl_multiset::MultiSetPRM(si_bogus, "MultiSetPRM"),
   roadmap(roadmap), roadmap_subgraphs_used(0),
   interroot_radius(1.0), lambda(1.0),
   space(space)
{
   if (this->roadmap->space != space)
      throw ompl::Exception("roadmap's space doesnt match planner's space!");
   printf("constructor called!\n");
   this->resolution = space->getLongestValidSegmentFraction() * space->getMaximumExtent();
   printf("resolution: %f\n", this->resolution);
#ifdef DEBUG_DUMPFILE
   this->dump_fp = 0;
#endif
}

P::~P()
{
   printf("destructor called!\n");
   // eventually we need to actually free all the ompl states in our graph
#ifdef DEBUG_DUMPFILE
   if (this->dump_fp) fclose(this->dump_fp);
#endif
}

void P::setProblemDefinition(const ompl::base::ProblemDefinitionPtr & pdef)
{
   const ompl::base::SpaceInformationPtr si = pdef->getSpaceInformation();
   // do we need to add this si with a default cost?
   if (this->si_to_ci.find(si.get()) == this->si_to_ci.end())
   {
      OMPL_INFORM("cfree not known, adding with default cost of 1.0");
      this->add_cfree(si, "", 1.0);
   }
   // save in base class (we should just use this!)
   this->pdef_ = pdef;
   // save pdef, ci for this problem
   this->pdef = pdef;
   this->pdef_ci = this->si_to_ci[si.get()];
   // if no pdef data already saved, make a new one
   if (this->pdefs.find(pdef) == this->pdefs.end())
      this->pdefs.insert(std::make_pair(pdef,ProbDefData()));
}

ompl::base::PlannerStatus P::solve(const ompl::base::PlannerTerminationCondition & ptc)
{
   bool success;
   
   printf("planning ...\n");
   
   // get pdef stuff
   if (!this->pdef)
      throw ompl::Exception("no problem definition set!");
   std::map<ompl::base::ProblemDefinitionPtr, ProbDefData>::iterator pdefs_it;
   pdefs_it = this->pdefs.find(this->pdef);
   if (pdefs_it == this->pdefs.end())
      throw ompl::Exception("no problem definition data found!");
   ProbDefData & pdefdata = pdefs_it->second;
   
   // ensure that the goal type has a finite number of samples (for now)
   if (!this->pdef->getGoal())
      throw ompl::Exception("no goal set!");
   if (!this->pdef->getGoal()->hasType(ompl::base::GOAL_SAMPLEABLE_REGION))
      throw ompl::Exception("goal is not sampleable!");
   ompl::base::GoalSampleableRegion * goalset = this->pdef->getGoal()->as<ompl::base::GoalSampleableRegion>();
   
   // make sure we've added our start and goal vertices to the graph
   // EVENTUALLY THIS SHOULD SEARCH THE GRAPH FOR EXISTING VERTICES!
   while (pdefdata.v_starts.size() < this->pdef->getStartStateCount())
   {
      const ompl::base::State * s = this->pdef->getStartState(pdefdata.v_starts.size());
      ompl::base::State * s_new = this->space->allocState();
      this->space->copyState(s_new, s);
      Vertex v = this->add_vertex(s_new);
      this->add_initial_root_edges(v);
      pdefdata.v_starts.push_back(v);
      v_roots.push_back(v);
   }
   while (pdefdata.v_goals.size() < goalset->maxSampleCount() && goalset->canSample())
   {
      ompl::base::State * s_new = this->space->allocState();
      goalset->sampleGoal(s_new);
      Vertex v = this->add_vertex(s_new);
      this->add_initial_root_edges(v);
      pdefdata.v_goals.push_back(v);
      v_roots.push_back(v);
   }
   
   if (!pdefdata.v_starts.size())
      throw ompl::Exception("no start states found!");
   if (!pdefdata.v_goals.size())
      throw ompl::Exception("no goal states found!");
   
   // add states
   while (ptc() == false)
   {
      //printf("graph has %lu vertics and %lu edges!\n",
      //   boost::num_vertices(this->g),
      //   boost::num_edges(this->g));
      
      // first, run bidirectional dijkstra's to find a candidate path
      std::list<Vertex> path_vs;
      std::list<Edge> path_es;
      double cost_estimate;
      enum DijkResult dijk_result = this->dijkstras_bidirectional(pdefdata, path_vs, path_es, cost_estimate);
      switch (dijk_result)
      {
      case DIJK_SUCCESS:
         break;
      case DIJK_NO_STARTS:
         return ompl::base::PlannerStatus::INVALID_START;
      case DIJK_NO_GOALS:
         return ompl::base::PlannerStatus::INVALID_GOAL;
      case DIJK_NO_PATH:
         printf("no connection found! considering next subgraph [%u] ...\n", this->roadmap_subgraphs_used);
         this->add_subgraph();
         continue;
      }
      
      printf("found potential path w/ cost estimate %f!\n", cost_estimate);
      
      printf("   path_vs:");
      for (std::list<Vertex>::iterator it_v=path_vs.begin(); it_v!=path_vs.end(); it_v++)
         printf(" %u", *it_v);
      printf("\n");
      
#ifdef DEBUG_DUMPFILE
      if (this->dump_fp)
      {
         fprintf(this->dump_fp, "candidate_path %f", cost_estimate);
         for (std::list<Vertex>::iterator it_v=path_vs.begin(); it_v!=path_vs.end(); it_v++)
         {
            double * q = g[*it_v].s->as<ompl::base::RealVectorStateSpace::StateType>()->values;
            fprintf(this->dump_fp, " %f,%f", q[0], q[1]);
         }
         fprintf(this->dump_fp, "\n");
         
         std::list<Vertex>::iterator it_v;
         std::list<Edge>::iterator it_e;
         it_v = path_vs.begin();
         fprintf(this->dump_fp, "candidate_path_cost_vertex %f\n",
            this->g[*it_v].estimates[this->pdef_ci].cost_remaining);
         it_v++;
         it_e=path_es.begin();
         for (; it_v!=path_vs.end(); it_v++,it_e++)
         {
            fprintf(this->dump_fp, "candidate_path_cost_edge %f\n",
               this->g[*it_e].estimates[this->pdef_ci].cost_remaining);
            fprintf(this->dump_fp, "candidate_path_cost_vertex %f\n",
               this->g[*it_v].estimates[this->pdef_ci].cost_remaining);
         }
      }
#endif
      
      success = isvalid_path(path_vs, path_es);
      if (!success) continue;
      
      printf("success!\n");
      
      /* create the path */
      ompl::geometric::PathGeometric * path = new ompl::geometric::PathGeometric(
         this->cfrees[this->pdef_ci].si);
      for (std::list<Vertex>::iterator it_v=path_vs.begin(); it_v!=path_vs.end(); it_v++)
         path->append(g[*it_v].s);
      this->pdef->addSolutionPath(ompl::base::PathPtr(path));
      
      return ompl::base::PlannerStatus::EXACT_SOLUTION;
   }
   
   return ompl::base::PlannerStatus::TIMEOUT;
}

void P::clear(void)
{
   throw std::runtime_error("clear not implemented!");
}

void P::set_interroot_radius(double interroot_radius)
{
   this->interroot_radius = interroot_radius;
}

void P::set_lambda(double lambda)
{
   if (lambda < 0.0)
   {
      OMPL_WARN("lambda passed less than zero! setting to zero ...");
      lambda = 0.0;
   }
   if (1.0 < lambda)
   {
      OMPL_WARN("lambda passed greater than one! setting to one ...");
      lambda = 1.0;
   }
   if (!(0.0 <= lambda && lambda <= 1.0))
      throw std::runtime_error("lambda not in range!");
   this->lambda = lambda;
}

void P::set_dumpfile(const char * dumpfile)
{
#ifdef DEBUG_DUMPFILE
   if (this->dump_fp) fclose(this->dump_fp);
   this->dump_fp = fopen(dumpfile, "w");
#else
   OMPL_WARN("set_dumpfile called, but dumping not supported!");
#endif
}

void P::add_cfree(const ompl::base::SpaceInformationPtr si, std::string name, double check_cost)
{
   // make sure it doesnt already exist
   if (this->si_to_ci.find(si.get()) != this->si_to_ci.end())
      throw ompl::Exception("add_cfree passed si we already know about!");
   // make sure the space matches our space
   if (si->getStateSpace() != this->space)
      throw ompl::Exception("add_si passed si with non-matching state space!");
   // make sure its set up
   if (!si->isSetup())
      si->setup();
   // add it!
   this->si_to_ci[si.get()] = this->cfrees.size();
   this->cfrees.push_back(Cfree(si, name, check_cost));
   this->recalc_truthtable();
   
   // extend all statuses with unknowns, and invalidate all estimates
   if (boost::num_vertices(this->g))
   {
      OMPL_WARN("attempting experimental space addition with existing graph!");
      for (std::pair<VertexIter,VertexIter> vp = boost::vertices(this->g);
         vp.first!=vp.second; vp.first++)
      {
         Vertex v = *vp.first;
         this->g[v].statuses.resize(this->cfrees.size(), STATUS_UNKNOWN);
         this->g[v].estimates.clear();
         this->g[v].estimates.resize(this->cfrees.size()); // defaults to dirty
      }
      for (std::pair<EdgeIter,EdgeIter> ep = boost::edges(this->g);
         ep.first!=ep.second; ep.first++)
      {
         Edge e = *ep.first;
         for (unsigned int ei=0; ei<this->g[e].edgestates.size(); ei++)
            this->g[e].edgestates[ei].statuses.resize(this->cfrees.size(), STATUS_UNKNOWN);
         this->g[e].statuses.resize(this->cfrees.size(), STATUS_UNKNOWN);
         this->g[e].estimates.clear();
         this->g[e].estimates.resize(this->cfrees.size()); // defaults to dirty
      }
      // clear optimistic checks cache
      this->optimistic_checks.clear();
   }
}

void P::add_inclusion(
   const ompl::base::SpaceInformationPtr si_superset,
   const ompl::base::SpaceInformationPtr si_subset)
{
   this->inclusions.push_back(Inclusion(si_superset, si_subset));
   // we should check that the si's are known!
   this->recalc_truthtable();
}

void P::add_intersection(
   const ompl::base::SpaceInformationPtr si_a,
   const ompl::base::SpaceInformationPtr si_b,
   const ompl::base::SpaceInformationPtr si_intersection)
{
   std::vector<ompl::base::SpaceInformationPtr> si_supersets;
   si_supersets.push_back(si_a);
   si_supersets.push_back(si_b);
   this->intersections.push_back(Intersection(si_intersection, si_supersets));
   // we should check that the si's are known!
   this->recalc_truthtable();
}

void P::add_intersection(
   const ompl::base::SpaceInformationPtr si_intersection,
   const std::vector<ompl::base::SpaceInformationPtr> si_supersets)
{
   this->intersections.push_back(Intersection(si_intersection, si_supersets));
   // we should check that the si's are known!
   this->recalc_truthtable();
}

void P::use_num_subgraphs(unsigned int num)
{
   while (this->roadmap_subgraphs_used < num)
   {
      printf("force considering next subgraph [%u] ...\n", this->roadmap_subgraphs_used);
      this->add_subgraph();
   }
}

void P::add_subgraph()
{
   // new subgraph to include
   // ensure new subgraph is generated
   unsigned int gi = this->roadmap_subgraphs_used;
   this->roadmap->subgraphs_generate(gi+1);
   ompl_multiset::Roadmap::SubGraph & sg = this->roadmap->subgraphs[gi];
   
   // add new vertices
   while (this->roadmap_vertices.size() < sg.nv)
   {
      ompl::base::State * s_new = this->space->allocState();
      this->space->copyState(s_new, this->roadmap->vertices[this->roadmap_vertices.size()]);
      Vertex v = this->add_vertex(s_new);
      this->roadmap_vertices.push_back(v);
      
      // add new edges to roots?
      for (unsigned int vi=0; vi<this->v_roots.size(); vi++)
      {
         Vertex root = this->v_roots[vi];
         double dist = this->space->distance(s_new, this->g[root].s);
         if (sg.root_radius < dist)
            continue;
         this->add_edge(v, root);
      }
   }
   
   // add new edges between roadmap vertices
   while (this->roadmap_edges.size() < sg.ne)
   {
      std::pair<unsigned int, unsigned int> & re = this->roadmap->edges[this->roadmap_edges.size()];
      Vertex va = this->roadmap_vertices[re.first];
      Vertex vb = this->roadmap_vertices[re.second];
      Edge e = this->add_edge(va, vb);
      this->roadmap_edges.push_back(e);
   }
   
   this->roadmap_subgraphs_used++;
}

void P::eval_everything(const ompl::base::SpaceInformationPtr si)
{
   // do we need to add this si with a default cost?
   // also get check index for this subset
   if (this->si_to_ci.find(si.get()) == this->si_to_ci.end())
   {
      OMPL_WARN("cfree not known, adding with default cost of 1.0");
      this->add_cfree(si, "", 1.0);
   }
   int ci = this->si_to_ci[si.get()];
   
   // check validity of ALL edges
   for (std::pair<EdgeIter,EdgeIter> ep = boost::edges(this->g);
      ep.first!=ep.second; ep.first++)
   {
      Edge e = *ep.first;
      isvalid_edge(g[e], ci, e);
   }
   
   // check validity of ALL vertices
   for (std::pair<VertexIter,VertexIter> vp = boost::vertices(this->g);
      vp.first!=vp.second; vp.first++)
   {
      Vertex v = *vp.first;
      isvalid_vertex(g[v], ci);
   }
}

void P::cache_load(const ompl_multiset::CachePtr cache)
{
   // load into roadmap
   cache->roadmap_load(this->roadmap.get());
   // for each si
   std::set<Vertex> vertices_touched;
   std::set<Edge> edges_touched;
   for (unsigned int ci=0; ci<this->cfrees.size(); ci++)
   {
      // save all known roadmap results
      std::vector< std::pair<unsigned int, bool> > vertex_results;
      std::vector< std::pair<unsigned int, bool> > edge_results;
      // load!
      cache->si_load(this->roadmap.get(), this->cfrees[ci].name,
         vertex_results, edge_results);
      // vertices
      for (unsigned int ri=0; ri<vertex_results.size(); ri++)
      {
         unsigned int vi = vertex_results[ri].first;
         if (this->roadmap_vertices.size() <= vi)
            continue;
         Vertex v = this->roadmap_vertices[vi];
         g[v].statuses[ci] = vertex_results[ri].second ? STATUS_VALID : STATUS_INVALID;
         vertices_touched.insert(v);
      }
      // edges
      for (unsigned int ri=0; ri<edge_results.size(); ri++)
      {
         unsigned int ei = edge_results[ri].first;
         if (this->roadmap_edges.size() <= ei)
            continue;
         Edge e = this->roadmap_edges[ei];
         // meta-status
         g[e].statuses[ci] = edge_results[ri].second ? STATUS_VALID : STATUS_INVALID;
         // sub statues
         // this is an approximation;
         // if the meta-status is INVALID,
         // some of the sub states may actually be VALID or UNKNOWN,
         // but i dont know how i should set them
         for (unsigned int esi=0; esi<g[e].edgestates.size(); esi++)
            g[e].edgestates[esi].statuses[ci] = edge_results[ri].second ? STATUS_VALID : STATUS_INVALID;
         edges_touched.insert(e);
      }
   }
   // calculate cross-ci status implications for touched elements
   // and set dirty bits
   for (std::set<Vertex>::iterator it=vertices_touched.begin(); it!=vertices_touched.end(); it++)
   {
      this->implies_statuses(g[*it].statuses);
      for (unsigned int ci=0; ci<g[*it].estimates.size(); ci++)
         g[*it].estimates[ci].cost_dirty = true;
   }
   for (std::set<Edge>::iterator it=edges_touched.begin(); it!=edges_touched.end(); it++)
   {
      this->implies_statuses(g[*it].statuses);
      for (unsigned int esi=0; esi<g[*it].edgestates.size(); esi++)
         this->implies_statuses(g[*it].edgestates[esi].statuses);
      for (unsigned int ci=0; ci<g[*it].estimates.size(); ci++)
         g[*it].estimates[ci].cost_dirty = true;
   }
}

void P::cache_save(const ompl_multiset::CachePtr cache)
{
   // save from roadmap
   cache->roadmap_save(this->roadmap.get());
   // for each si
   for (unsigned int ci=0; ci<this->cfrees.size(); ci++)
   {
      // save all known roadmap results
      std::vector< std::pair<unsigned int, bool> > vertex_results;
      std::vector< std::pair<unsigned int, bool> > edge_results;
      // vertices
      for (unsigned int vi=0; vi<this->roadmap_vertices.size(); vi++)
      {
         Vertex v = this->roadmap_vertices[vi];
         switch (g[v].statuses[ci])
         {
         case STATUS_VALID:
            vertex_results.push_back(std::make_pair(vi,true));
            break;
         case STATUS_INVALID:
            vertex_results.push_back(std::make_pair(vi,false));
            break;
         case STATUS_UNKNOWN:
            break;
         }
      }
      // edges
      for (unsigned int ei=0; ei<this->roadmap_edges.size(); ei++)
      {
         Edge e = this->roadmap_edges[ei];
         switch (g[e].statuses[ci])
         {
         case STATUS_VALID:
            edge_results.push_back(std::make_pair(ei,true));
            break;
         case STATUS_INVALID:
            edge_results.push_back(std::make_pair(ei,false));
            break;
         case STATUS_UNKNOWN:
            break;
         }
      }
      // save!
      cache->si_save(this->roadmap.get(), this->cfrees[ci].name,
         vertex_results, edge_results);
   }
}

void P::recalc_truthtable()
{
   int ci;
   unsigned int ui;
   unsigned int ui2;
   
   printf("recalculating truthtable ...\n");
   
   // generate all possible states into truthtable
   std::vector<bool> state(this->cfrees.size(), false);
   this->truthtable.clear();
   for (;;)
   {
      // add
      this->truthtable.push_back(state);
      // increment
      bool carry = true;
      for (ci=0; ci<state.size(); ci++)
      {
         if (!carry)
            break;
         if (!state[ci])
         {
            state[ci] = true;
            carry = false;
            break;
         }
         state[ci] = false;
      }
      if (carry)
         break;
   }
   
   // next, remove states which conflict with provided set relation
   for (std::list< std::vector<bool> >::iterator it=this->truthtable.begin(); it!=this->truthtable.end();)
   {
      // inclusions
      for (ui=0; ui<this->inclusions.size(); ui++)
      {
         unsigned int ci_superset = this->si_to_ci[this->inclusions[ui].si_superset.get()];
         unsigned int ci_subset = this->si_to_ci[this->inclusions[ui].si_subset.get()];
         if (!(*it)[ci_superset] && (*it)[ci_subset])
         {
            it = this->truthtable.erase(it);
            break;
         }
      }
      if (ui<this->inclusions.size())
         continue;
      
      // intersections
      for (ui=0; ui<this->intersections.size(); ui++)
      {
         bool rhs = true;
         for (ui2=0; ui2<this->intersections[ui].si_supersets.size(); ui2++)
         {
            unsigned int ci_superset = this->si_to_ci[this->intersections[ui].si_supersets[ui2].get()];
            rhs = rhs && (*it)[ci_superset];
         }
         unsigned int ci_intersection = this->si_to_ci[this->intersections[ui].si_intersection.get()];
         //if ((*it)[ci_intersection] == (!(*it)[ci_a] || !(*it)[ci_b]))
         if ((*it)[ci_intersection] != rhs)
         {
            it = this->truthtable.erase(it);
            break;
         }
      }
      if (ui<this->intersections.size())
         continue;
      
      it++;
   }
   
   this->optimistic_checks.clear();
}

inline
const std::pair<double, std::vector<std::pair<int,bool> > > & P::get_optimistic_checks(
      std::vector<enum CheckStatus> & statuses, int ci_target)
{
   int ci;
   int ti;
   std::map<
      std::pair< std::vector<enum CheckStatus>, int >,
      std::pair<double, std::vector<std::pair<int,bool> > >
      >::iterator it;
   
   // look it up in the cache
   std::pair< std::vector<enum CheckStatus>, int > key = std::make_pair(statuses,ci_target);
   it = this->optimistic_checks.find(key);
   if (it != this->optimistic_checks.end())
      return it->second;
   
   // remove any truths which don't match our current status
   std::list< std::vector<bool> > truths = this->truthtable;
   for (std::list< std::vector<bool> >::iterator it=truths.begin(); it!=truths.end();)
   {
      for (ci=0; ci<statuses.size(); ci++)
      {
         if (statuses[ci] == STATUS_UNKNOWN)
            continue;
         if ((statuses[ci]==STATUS_VALID) != (*it)[ci])
            break;
      }
      if (ci<statuses.size())
         it = truths.erase(it);
      else
         it++;
   }
   
   // consider possible tests, each is (ci,desired_result)
   std::pair<double, std::vector<std::pair<int,bool> > > optimal;
   bool optimal_found = false;
   std::priority_queue<
      std::pair<double, std::vector<std::pair<int,bool> > >,
      std::vector< std::pair<double, std::vector<std::pair<int,bool> > > >,
      std::greater< std::pair<double, std::vector<std::pair<int,bool> > > >
      > open;
   open.push(std::make_pair(0.0, std::vector<std::pair<int,bool> >()));
   while (!open.empty())
   {
      std::pair<double, std::vector<std::pair<int,bool> > > top = open.top();
      open.pop();
      // count up the consistent truths; if our ci_target is all T, we're done!
      bool found_true = false;
      bool found_false = false;
      for (std::list< std::vector<bool> >::iterator it=truths.begin(); it!=truths.end(); it++)
      {
         // ensure this truth is consistent with our tests
         for (ti=0; ti<top.second.size(); ti++)
            if ((*it)[top.second[ti].first] != top.second[ti].second)
               break;
         if (ti<top.second.size())
            continue; // inconsistent!
         // is it false?
         if ((*it)[ci_target])
            found_true = true;
         else
            found_false = true;
      }
      if (!found_true) // no trues means its a hopeless branch, so kill it
         continue;
      if (!found_false)
      {
         optimal = top;
         optimal_found = true;
         break;
      }
      // add any test that hasn't been tried yet
      for (ci=0; ci<statuses.size(); ci++)
      {
         for (ti=0; ti<top.second.size(); ti++)
            if (ci == top.second[ti].first)
               break;
         if (ti<top.second.size())
            continue;
         std::vector<std::pair<int,bool> > tests_new;
         tests_new = top.second;
         tests_new.push_back(std::make_pair(ci,false));
         open.push(std::make_pair(top.first + this->cfrees[ci].check_cost, tests_new));
         tests_new = top.second;
         tests_new.push_back(std::make_pair(ci,true));
         open.push(std::make_pair(top.first + this->cfrees[ci].check_cost, tests_new));
      }
   }
   
   // else calculate it
   printf("calculating optimistic_checks ...\n");
   printf("  for current statuses:");
   for (ci=0; ci<statuses.size(); ci++)
      printf(" %c", "UVI"[statuses[ci]]);
   printf("\n");
   printf("  for ci_target: %d\n", ci_target);

   printf("  possible truths:\n");
   for (std::list< std::vector<bool> >::iterator it=truths.begin(); it!=truths.end(); it++)
   {
      printf("   ");
      for (ci=0; ci<statuses.size(); ci++)
         printf(" %c", "FT"[(*it)[ci]]);
      printf("\n");
   }
   
   if (!optimal_found)
      throw ompl::Exception("no optimal found!");
   
   // we have a winner!
   printf("  we have a winner! cost:%f, %u tests:\n", optimal.first, optimal.second.size());
   for (ti=0; ti<optimal.second.size(); ti++)
      printf("    test ci=%d desired=%c\n", optimal.second[ti].first, "FT"[optimal.second[ti].second]);

   this->optimistic_checks.insert(std::make_pair(key,optimal));
   return this->optimistic_checks[key];
}

inline
enum P::DijkResult P::dijkstras_bidirectional(ProbDefData & pdefdata,
   std::list<Vertex> & path_vs, std::list<Edge> & path_es, double & cost_estimate)
{
   int i;
   std::priority_queue<DijkType> open_fwd;
   std::priority_queue<DijkType> open_bck;
   std::map<Vertex, DijkType> closed_fwd;
   std::map<Vertex, DijkType> closed_bck;
   double cost_connection = HUGE_VAL;
   Vertex v_connection = GraphTypes::null_vertex();
   for (i=0; i<pdefdata.v_starts.size(); i++)
   {
      Vertex v = pdefdata.v_starts[i];
      update_vertex_estimate(g[v], this->pdef_ci);
      if (g[v].statuses[this->pdef_ci] == STATUS_INVALID)
         continue;
      double cost = 0.0;
      if (g[v].statuses[this->pdef_ci] == STATUS_UNKNOWN)
         cost += 0.5*g[v].estimates[this->pdef_ci].cost_remaining;
      open_fwd.push(DijkType(cost,
         v, GraphTypes::null_vertex(), Edge()));
   }
   if (open_fwd.empty())
      return DIJK_NO_STARTS;
   for (i=0; i<pdefdata.v_goals.size(); i++)
   {
      Vertex v = pdefdata.v_goals[i];
      update_vertex_estimate(g[v], this->pdef_ci);
      if (g[v].statuses[this->pdef_ci] == STATUS_INVALID)
         continue;
      double cost = 0.0;
      if (g[v].statuses[this->pdef_ci] == STATUS_UNKNOWN)
         cost += 0.5*g[v].estimates[this->pdef_ci].cost_remaining;
      open_bck.push(DijkType(cost,
         v, GraphTypes::null_vertex(), Edge()));
   }
   if (open_bck.empty())
      return DIJK_NO_GOALS;
   while (!open_fwd.empty() && !open_bck.empty())
   {
      std::priority_queue<DijkType> * open_mine;
      std::map<Vertex, DijkType> * closed_mine;
      std::map<Vertex, DijkType> * closed_other;
      // are we done?
      if (cost_connection <= open_fwd.top().v_cost + open_bck.top().v_cost)
         break;
      // get appropriate open/closed lists
      if (open_fwd.top().v_cost < open_bck.top().v_cost)
      {
         open_mine = &open_fwd;
         closed_mine = &closed_fwd;
         closed_other = &closed_bck;
      }
      else
      {
         open_mine = &open_bck;
         closed_mine = &closed_bck;
         closed_other = &closed_fwd;
      }
      // pop next
      DijkType top = open_mine->top();
      open_mine->pop();
      // already in closed list?
      if (closed_mine->find(top.v) != closed_mine->end())
         continue;
      // add to closed list!
      closed_mine->insert(std::make_pair(top.v,top));
      // did we find a better potential path?
      std::map<Vertex, DijkType>::iterator it_other = closed_other->find(top.v);
      if (it_other != closed_other->end())
      {
         if (top.v_cost + it_other->second.v_cost < cost_connection)
         {
            cost_connection = top.v_cost + it_other->second.v_cost;
            v_connection = top.v;
         }
      }
      // get all successors
      for (std::pair<EdgeOutIter,EdgeOutIter> ep = boost::out_edges(top.v, g);
         ep.first!=ep.second; ep.first++)
      {
         Edge e = *ep.first;
         Vertex v_succ = boost::target(e, g);
         if (v_succ == top.v)
            v_succ = boost::source(e, g);
         // already in closed list?
         if (closed_mine->find(v_succ) != closed_mine->end())
            continue;
         // get cumulative cost to the vertex
         double new_cost = top.v_cost;
         // add in half cost of the source vertex
         new_cost += this->lambda * 0.5 * g[top.v].estimates[this->pdef_ci].cost_remaining;
         // get cost of the target (successor) vertex itself
         update_vertex_estimate(g[v_succ], this->pdef_ci);
         if (g[v_succ].statuses[this->pdef_ci] == STATUS_INVALID)
            continue;
         if (g[v_succ].statuses[this->pdef_ci] == STATUS_UNKNOWN)
            new_cost += this->lambda * 0.5 * g[v_succ].estimates[this->pdef_ci].cost_remaining;
         // get cost of the edge
         update_edge_estimate(g[e], this->pdef_ci);
         if (g[e].statuses[this->pdef_ci] == STATUS_INVALID)
            continue;
         new_cost += (1.0 - this->lambda) * g[e].euc_dist;
         if (g[e].statuses[this->pdef_ci] == STATUS_UNKNOWN)
            new_cost += this->lambda * g[e].estimates[this->pdef_ci].cost_remaining;
         // add to open list
         open_mine->push(DijkType(new_cost, v_succ, top.v, e));
      }
   }
   if (v_connection == GraphTypes::null_vertex())
      return DIJK_NO_PATH;
   // return cost estimate
   cost_estimate = cost_connection;
   // create the path
   path_vs.push_back(v_connection);
   for (;;)
   {
      std::map<Vertex, DijkType>::iterator it = closed_fwd.find(path_vs.front());
      if (it->second.v_parent == GraphTypes::null_vertex())
         break;
      path_vs.push_front(it->second.v_parent);
      path_es.push_front(it->second.e_to_parent);
   }
   for (;;)
   {
      std::map<Vertex, DijkType>::iterator it = closed_bck.find(path_vs.back());
      if (it->second.v_parent == GraphTypes::null_vertex())
         break;
      path_vs.push_back(it->second.v_parent);
      path_es.push_back(it->second.e_to_parent);
   }
   return DIJK_SUCCESS;
}

bool P::isvalid_path(std::list<Vertex> & path_vs, std::list<Edge> & path_es)
{
   bool isvalid = true;
   
   // walk path forward, checking as we go
   do
   {
      std::list<Vertex>::iterator it_v;
      std::list<Edge>::iterator it_e;
      
      // check first vertex
      it_v = path_vs.begin();
      if (!isvalid_vertex(g[*it_v], this->pdef_ci))
      {
         isvalid = false;
         break;
      }
      // check each vertex,edge combo as we go
      // (far vertex first)
      it_v++;
      it_e=path_es.begin();
      for (; it_v!=path_vs.end(); it_v++,it_e++)
      {
         if (!isvalid_vertex(g[*it_v], this->pdef_ci))
         {
            isvalid = false;
            break;
         }
         
         if (!isvalid_edge(g[*it_e], this->pdef_ci, *it_e))
         {
            isvalid = false;
            break;
         }
      }
   }
   while (0);
   
   if (isvalid)
      return isvalid;
   
   // walk path backwards too
   do
   {
      std::list<Vertex>::reverse_iterator it_v;
      std::list<Edge>::reverse_iterator it_e;
      
      // check first vertex
      it_v = path_vs.rbegin();
      if (!isvalid_vertex(g[*it_v], this->pdef_ci))
      {
         isvalid = false;
         break;
      }
      // check each vertex,edge combo as we go
      // (far vertex first)
      it_v++;
      it_e=path_es.rbegin();
      for (; it_v!=path_vs.rend(); it_v++,it_e++)
      {
         if (!isvalid_vertex(g[*it_v], this->pdef_ci))
         {
            isvalid = false;
            break;
         }
         
         if (!isvalid_edge(g[*it_e], this->pdef_ci, *it_e))
         {
            isvalid = false;
            break;
         }
      }
   }
   while (0);
   
   return isvalid;
}

P::Vertex P::add_vertex(ompl::base::State * s)
{
   Vertex v = boost::add_vertex(this->g);
   // as soon as we add a vertex,
   // we must initialize the associated vertex properties
   this->g[v].s = s;
   this->g[v].estimates.clear();
   this->g[v].estimates.resize(this->cfrees.size()); // defaults to dirty
   this->g[v].statuses.clear();
   this->g[v].statuses.resize(this->cfrees.size(), STATUS_UNKNOWN);
#ifdef DEBUG_DUMPFILE
   if (this->dump_fp)
   {
      double * q = s->as<ompl::base::RealVectorStateSpace::StateType>()->values;
      fprintf(this->dump_fp, "add_vertex %f,%f", q[0], q[1]);
      for (int i=0; i<this->cfrees.size(); i++) fprintf(this->dump_fp," U"); fprintf(this->dump_fp,"\n");
   }
#endif
#if 0
   // for now, we find nearest neighbors by just iterating over
   // all existing vertices
   for (std::pair<VertexIter,VertexIter> vp = boost::vertices(this->g);
      vp.first!=vp.second; vp.first++)
   {
      Vertex n = *vp.first;
      if (n == v)
         continue;
      double euc_dist = this->space->distance(this->g[v].s, this->g[n].s);
      if (this->radius < euc_dist)
         continue;
      this->add_edge(v, n);
   }
#endif
   return v;
}

P::Edge P::add_edge(P::Vertex va, P::Vertex vb)
{
   Edge e;
   bool success;
   double euc_dist;
   int i;
   int n;
#ifdef DEBUG_DUMPFILE
   if (this->dump_fp)
   {
      double * qa = this->g[va].s->as<ompl::base::RealVectorStateSpace::StateType>()->values;
      double * qb = this->g[vb].s->as<ompl::base::RealVectorStateSpace::StateType>()->values;
      fprintf(this->dump_fp, "add_edge %f,%f %f,%f", qa[0], qa[1], qb[0], qb[1]);
      for (int i=0; i<this->cfrees.size(); i++) fprintf(this->dump_fp," U"); fprintf(this->dump_fp,"\n");
   }
#endif
   boost::tie(e,success) = boost::add_edge(va, vb, this->g);
   if (!success)
      throw ompl::Exception("edge already exists somehow!");
   euc_dist = this->space->distance(this->g[va].s, this->g[vb].s);
   this->g[e].euc_dist = euc_dist;
   // allocate internal states for this edge
   n = floor(euc_dist / this->resolution);
   this->g[e].edgestates.clear();
   this->g[e].edgestates.resize(n);
   this->g[e].statuses.clear();
   this->g[e].statuses.resize(this->cfrees.size(), STATUS_UNKNOWN);
   const std::vector<int> & perm = this->get_edgeperm(n);
   for (i=0; i<n; i++)
   {
      this->g[e].edgestates[i].s = this->space->allocState();
      space->interpolate(g[va].s, g[vb].s, 1.0*(perm[i]+1)/(n+1), this->g[e].edgestates[i].s);
      this->g[e].edgestates[i].statuses.clear();
      this->g[e].edgestates[i].statuses.resize(this->cfrees.size(), STATUS_UNKNOWN);
   }
   // set estimates
   this->g[e].estimates.clear();
   this->g[e].estimates.resize(this->cfrees.size()); // defaults to unknown, dirty
   if (n==0)
   {
      for (i=0; i<this->g[e].estimates.size(); i++)
         this->g[e].statuses[i] = STATUS_VALID;
   }
   return e;
}

void P::add_initial_root_edges(Vertex vroot)
{
   // iterate over all fixed-roadmap (non-root) vertices
   // and connect to them based on the subgraph's root_radius
   unsigned int vi=0;
   for (unsigned int gi=0; gi<this->roadmap_subgraphs_used; gi++)
   {
      ompl_multiset::Roadmap::SubGraph & sg = this->roadmap->subgraphs[gi];
      for (; vi<sg.nv; vi++)
      {
         Vertex v = this->roadmap_vertices[vi];
         double dist = this->space->distance(this->g[vroot].s, this->g[v].s);
         if (sg.root_radius < dist)
            continue;
         this->add_edge(vroot, v);
      }
   }
   // iterate over all existing roots
   // and connect to them based on some random radius parameter
   for (vi=0; vi<this->v_roots.size(); vi++)
   {
      Vertex v = this->v_roots[vi];
      double dist = this->space->distance(this->g[vroot].s, this->g[v].s);
      if (this->interroot_radius < dist)
         continue;
      this->add_edge(vroot, v);
   }
}

// from calc_order_endgood
const std::vector<int> & P::get_edgeperm(int n)
{
   // look it up in the cache
   std::map<int, const std::vector<int> >::iterator it;
   it = this->edgeperms.find(n);
   if (it != this->edgeperms.end())
      return it->second;
   // else calculate it
   printf("calculating perm for n=%d ...\n", n);
   int i;
   int last_true;
   int max_i;
   int max_val;
   std::vector<int> perm;
   std::vector<bool> done(n, false);
   std::vector<int> dist(n);
   for (;;)
   {
      last_true = -1;
      for (i=0; i<n; i++)
      {
         if (done[i]) last_true = i;
         dist[i] = (i-last_true);
      }
      last_true = n;
      for (i=n-1; i>=0; i--)
      {
         if (done[i]) last_true = i;
         dist[i] = (last_true-i) < dist[i] ? (last_true-i) : dist[i];
      }
      max_val = 0;
      for (i=0; i<n; i++) if (max_val < dist[i])
      {
         max_val = dist[i];
         max_i = i;
      }
      if (!max_val)
         break;
      perm.push_back(max_i);
      done[max_i] = true;
   }
   printf("  result: [");
   for (i=0; i<n; i++)
      printf(" %d", perm[i]);
   printf(" ]\n");
   this->edgeperms.insert(std::make_pair(n,perm));
   return this->edgeperms[n];
}

inline
void P::update_vertex_estimate(struct VertexProperties & vp, unsigned int ci_target)
{
   if (vp.statuses[ci_target] != STATUS_UNKNOWN) return;
   if (vp.estimates[ci_target].cost_dirty == false) return;
   
   const std::pair<double, std::vector<std::pair<int,bool> > > & checks
      = this->get_optimistic_checks(vp.statuses, ci_target);
   
   vp.estimates[ci_target].cost_remaining = checks.first;
   vp.estimates[ci_target].cost_dirty = false;
}

inline
void P::update_edge_estimate(struct EdgeProperties & ep, unsigned int ci_target)
{
   if (ep.statuses[ci_target] != STATUS_UNKNOWN) return;
   if (ep.estimates[ci_target].cost_dirty == false) return;
   
   int ei;
   int ti;
   int ci;
   
   // assume the edge's meta statuses are up-to-date
   
   // given edge's meta statuses, calculate checks required w.r.t. ci_target
   const std::pair<double, std::vector<std::pair<int,bool> > > & checks
      = this->get_optimistic_checks(ep.statuses, ci_target);
   
   // visit each state on the edge
   // this is not perfect w.r.t. our cfree check model
   ep.estimates[ci_target].cost_remaining = 0.0;
   for (ei=0; ei<ep.edgestates.size(); ei++)
   {
      if (ep.edgestates[ei].statuses[ci_target] == STATUS_VALID)
         continue; // no cost, already valid!
      // sum up the cost of each required check!
      for (ti=0; ti<checks.second.size(); ti++)
      {
         ci = checks.second[ti].first;
         if (ep.edgestates[ei].statuses[ci] == STATUS_UNKNOWN)
            ep.estimates[ci_target].cost_remaining += this->cfrees[ci].check_cost;
      }
   }
   ep.estimates[ci_target].cost_dirty = false;
}

inline
bool P::isvalid_vertex(struct VertexProperties & vp, unsigned int ci_target)
{
   int ti;
   int ci;
   if (vp.statuses[ci_target] == STATUS_UNKNOWN)
   {
      // what checks should we perform?
      const std::pair<double, std::vector<std::pair<int,bool> > > & checks
         = this->get_optimistic_checks(vp.statuses, ci_target);
      
      // perform all requested checks
      for (ti=0; ti<checks.second.size(); ti++)
      {
         ci = checks.second[ti].first;
         if (this->cfrees[ci].si->isValid(vp.s))
            vp.statuses[ci] = STATUS_VALID;
         else
            vp.statuses[ci] = STATUS_INVALID;
      }
      
      // what does this imply for all our other statuses?
      this->implies_statuses(vp.statuses);
      
#ifdef DEBUG_DUMPFILE
      if (this->dump_fp)
      {
         double * q = vp.s->as<ompl::base::RealVectorStateSpace::StateType>()->values;
         fprintf(this->dump_fp, "update_vertex %f,%f", q[0], q[1]);
         for (ci=0; ci<vp.statuses.size(); ci++)
            fprintf(this->dump_fp," %c","UVI"[vp.statuses[ci]]);
         fprintf(this->dump_fp,"\n");
      }
#endif

      // invalidate estimates
      for (ci=0; ci<vp.estimates.size(); ci++)
         vp.estimates[ci].cost_dirty = true;
   }
   if (vp.statuses[ci_target] == STATUS_VALID)
      return true;
   else
      return false;
}

inline
bool P::isvalid_edge(struct EdgeProperties & ep, unsigned int ci_target, Edge e)
{
   int ei;
   int ti;
   int ci;
   
   if (ep.statuses[ci_target] == STATUS_UNKNOWN)
   {
      // what checks should we perform?
      // this assumes that the meta-statusus are correct
      const std::pair<double, std::vector<std::pair<int,bool> > > & checks
         = this->get_optimistic_checks(ep.statuses, ci_target);
      
      // do sub-checks for each edge point
      // we assume that the status for ci_target for all edge points
      // are correctly also unknown or valid
      for (ei=0; ei<ep.edgestates.size(); ei++)
      {
         if (ep.edgestates[ei].statuses[ci_target] == STATUS_VALID)
            continue;
         
         // else unknown
         
         // perform all requested checks
         for (ti=0; ti<checks.second.size(); ti++)
         {
            ci = checks.second[ti].first;
            if (this->cfrees[ci].si->isValid(ep.edgestates[ei].s))
               ep.edgestates[ei].statuses[ci] = STATUS_VALID;
            else
               ep.edgestates[ei].statuses[ci] = STATUS_INVALID;
         }
         
         // what does this imply for all our other statuses (at the edge point only)?
         this->implies_statuses(ep.edgestates[ei].statuses);
         
         /* fail early if we know we're invalid w.r.t. ci_target */
         if (ep.edgestates[ei].statuses[ci_target] == STATUS_INVALID)
            break;
      }
      
      // update meta-statuses
      for (ci=0; ci<ep.statuses.size(); ci++)
      {
         ep.statuses[ci] = STATUS_VALID;
         for (ei=0; ei<ep.edgestates.size(); ei++)
         {
            if (ep.edgestates[ei].statuses[ci] == STATUS_INVALID)
            {
               ep.statuses[ci] = STATUS_INVALID;
               break;
            }
            if (ep.edgestates[ei].statuses[ci] == STATUS_UNKNOWN)
               ep.statuses[ci] = STATUS_UNKNOWN;
         }
      }

#ifdef DEBUG_DUMPFILE
      if (this->dump_fp)
      {
         double * qa = this->g[boost::source(e,g)].s->as<ompl::base::RealVectorStateSpace::StateType>()->values;
         double * qb = this->g[boost::target(e,g)].s->as<ompl::base::RealVectorStateSpace::StateType>()->values;
         fprintf(this->dump_fp, "update_edge %f,%f %f,%f", qa[0], qa[1], qb[0], qb[1]);
         for (ci=0; ci<ep.statuses.size(); ci++)
            fprintf(this->dump_fp," %c","UVI"[ep.statuses[ci]]);
         fprintf(this->dump_fp,"\n");
      }
#endif
      
      // invalidate estimates
      for (ci=0; ci<ep.estimates.size(); ci++)
         ep.estimates[ci].cost_dirty = true;
   }
   if (ep.statuses[ci_target] == STATUS_VALID)
      return true;
   else
      return false;
}

inline
void P::implies_statuses(std::vector<enum CheckStatus> & statuses)
{
   int ci;
   std::list< std::vector<bool> > truths = this->truthtable;
   for (std::list< std::vector<bool> >::iterator it=truths.begin(); it!=truths.end();)
   {
      for (ci=0; ci<statuses.size(); ci++)
      {
         if (statuses[ci] == STATUS_UNKNOWN)
            continue;
         if ((statuses[ci]==STATUS_VALID) != (*it)[ci])
            break;
      }
      if (ci<statuses.size())
         it = truths.erase(it);
      else
         it++;
   }
   for (ci=0; ci<statuses.size(); ci++)
   {
      bool found_true = false;
      bool found_false = false;
      for (std::list< std::vector<bool> >::iterator it=truths.begin(); it!=truths.end(); it++)
      {
         if ((*it)[ci])
            found_true = true;
         else
            found_false = true;
      }
      if (found_true && found_false)
         continue;
      if (!found_true && !found_false)
         throw ompl::Exception("inconsistent statuses!");
      if (found_true && !found_false)
         statuses[ci] = STATUS_VALID;
      if (!found_true && found_false)
         statuses[ci] = STATUS_INVALID;
   }
}
