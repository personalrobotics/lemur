#include <cstdio>
#include <list>
#include <queue>
#include <string>
#include <vector>
#include <boost/function.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <ompl/base/Planner.h>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <checkmask/graph.h>

#define DEBUG_DUMPFILE "dump.txt"

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

class P : public checkmask::GraphPlanner
{
public:
   P(const ompl::base::StateSpacePtr & space,
      const ompl::base::SpaceInformationPtr & si_bogus);
   ~P(void);

   // ompl planner interface
   void setProblemDefinition(const ompl::base::ProblemDefinitionPtr & pdef);
   ompl::base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition & ptc);
   void clear(void);
   
   void add_si(const ompl::base::SpaceInformationPtr si, double check_cost);

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
      CheckEstimate(): status(STATUS_UNKNOWN), cost_dirty(true) {}
      enum CheckStatus status;
      bool cost_dirty;
      double cost_remaining;
   };
   struct VertexProperties
   {
      ompl::base::State * s;
      std::vector<CheckEstimate> estimates;
   };
   struct EdgeProperties
   {
      double euc_dist;
      std::vector<EdgeState> edgestates;
      std::vector<CheckEstimate> estimates;
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
   typedef boost::graph_traits<Graph>::out_edge_iterator EdgeOutIter;
   
   struct ProbDefData
   {
      ProbDefData(ompl::base::ProblemDefinitionPtr pdef):
         v_start(GraphTypes::null_vertex()),
         v_goal(GraphTypes::null_vertex())
      {
         pis.use(pdef);
      }
      Vertex v_start;
      Vertex v_goal;
      ompl::base::PlannerInputStates pis;
   };
   
   struct BiDijkType
   {
      BiDijkType(double v_cost, Vertex v, Vertex v_parent, Edge e_to_parent, bool isbck=true):
         v_cost(v_cost), v(v), v_parent(v_parent), e_to_parent(e_to_parent), isbck(isbck) {}
      double v_cost;
      Vertex v;
      Vertex v_parent;
      Edge e_to_parent;
      bool isbck;
      bool operator < (const BiDijkType rhs) const // for sorting on the open priority queue
      {
        return v_cost > rhs.v_cost;
      }
   };

   // private methods
   
   // given the current problem,
   // try to get a path!
   // return true on success
   bool dijkstras_bidirectional(ProbDefData & pdefdata,
      std::list<Vertex> & path_vs, std::list<Edge> & path_es);
   
   bool isvalid_path(std::list<Vertex> & path_vs, std::list<Edge> & path_es);
   
   // this takes ownership of the state
   // this also adds edges within the specified radius (currently 0.2)
   Vertex add_vertex(ompl::base::State * s);
   Edge add_edge(Vertex va, Vertex vb);
   
   // ok, this is where the magic happens!
   // hopefully this will be inlined
   // update the checkablestate's checkestimates[sii]
   // given the current validity mask across all si's
   void update_vertex_estimate(struct VertexProperties & vp, unsigned int sii);
   void update_edge_estimate(struct EdgeProperties & ep, unsigned int sii);
   
   bool isvalid_vertex(struct VertexProperties & vp, unsigned int sii);
   bool isvalid_edge(struct EdgeProperties & ep, unsigned int sii);
   
   const std::vector<int> & get_edgeperm(int n);

#if 0
   void add_inclusion(std::pair<std::string,std::string> supersub);
   void remove_inclusion(std::pair<std::string,std::string> supersub);
   std::pair<std::string,std::string> get_inclusions();
#endif
   
   // private members
   
   // we have a graph g
   Graph g;
   
   // this planner can only work with a single space,
   // provided on construction
   // (of course we can handle many different spaceinformations,
   //  as long as they all share the same space and parameters
   ompl::base::StateSpacePtr space;
   ompl::base::StateSamplerPtr sampler;
   
   // the current problem definition
   ompl::base::ProblemDefinitionPtr pdef;
   int pdef_sii;
   
   // we should maintain info from past probdefs
   // e.g. which vertices correspond to the goals
   // so that we can switch back to them quickly
   std::map<ompl::base::ProblemDefinitionPtr, ProbDefData> pdefs;
   
   // known sis and their costs
   std::vector< std::pair<ompl::base::SpaceInformationPtr,double> > sis;
   
   // maintain list of si relations
   // (inclusions, intersections, etc)
   
   // cache
   std::map<int, const std::vector<int> > edgeperms;
   
#ifdef DEBUG_DUMPFILE
   FILE * dump_fp;
#endif
};

} // anonymous namespace

checkmask::GraphPlanner * checkmask::GraphPlanner::create(const ompl::base::StateSpacePtr & space)
{
   ompl::base::SpaceInformationPtr si_bogus(new ompl::base::SpaceInformation(space));
   si_bogus->setStateValidityChecker(
      ompl::base::StateValidityCheckerPtr(
         new ompl::base::AllValidStateValidityChecker(si_bogus)
      )
   );
   si_bogus->setup();
   return new P(space, si_bogus);
}

P::P(
      const ompl::base::StateSpacePtr & space,
      const ompl::base::SpaceInformationPtr & si_bogus):
   checkmask::GraphPlanner(si_bogus),
   space(space), sampler(space->allocStateSampler())
{
   printf("constructor called!\n");
   
#ifdef DEBUG_DUMPFILE
   this->dump_fp = fopen(DEBUG_DUMPFILE, "w");
#endif
}

checkmask::GraphPlanner::GraphPlanner(const ompl::base::SpaceInformationPtr & si_bogus):
   ompl::base::Planner(si_bogus, "CheckMaskGraph")
{
}

P::~P()
{
   printf("destructor called!\n");
   // eventually we need to actually free all the ompl states in our graph

#ifdef DEBUG_DUMPFILE
   fclose(this->dump_fp);
#endif
}

void P::setProblemDefinition(const ompl::base::ProblemDefinitionPtr & pdef)
{
   const ompl::base::SpaceInformationPtr si = pdef->getSpaceInformation();
   // do we need to add this si with a default cost?
   unsigned int ui;
   for (ui=0; ui<this->sis.size(); ui++)
      if (this->sis[ui].first == si)
         break;
   if (!(ui<this->sis.size()))
   {
      OMPL_INFORM("si not known, adding with default cost of 1.0");
      this->add_si(si, 1.0);
   }
   // save pdef, sii forthis problem
   this->pdef = pdef;
   for (ui=0; ui<this->sis.size(); ui++)
      if (this->sis[ui].first == si)
         this->pdef_sii = ui;
   // if no pdef data already saved, make a new one
   if (this->pdefs.find(pdef) == this->pdefs.end())
      this->pdefs.insert(std::make_pair(pdef,ProbDefData(pdef)));
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
   
   // make sure we've added our start and goal vertices to the graph
   // EVENTUALLY THIS SHOULD SEARCH THE GRARPH FOR EXISTING VERTICES!
   while (const ompl::base::State * s = pdefdata.pis.nextStart())
   {
      if (pdefdata.v_start != GraphTypes::null_vertex())
         throw ompl::Exception("we already have a start state!");
      ompl::base::State * s_new = this->space->allocState();
      this->space->copyState(s_new, s);
      pdefdata.v_start = this->add_vertex(s_new);
   }
   while (pdefdata.pis.haveMoreGoalStates())
   {
      const ompl::base::State * s = pdefdata.pis.nextGoal();
      if (pdefdata.v_goal != GraphTypes::null_vertex())
         throw ompl::Exception("we already have a goal state!");
      ompl::base::State * s_new = this->space->allocState();
      this->space->copyState(s_new, s);
      pdefdata.v_goal = this->add_vertex(s_new);
   }
   if (pdefdata.v_start == GraphTypes::null_vertex())
      throw ompl::Exception("no start state found!");
   if (pdefdata.v_goal == GraphTypes::null_vertex())
      throw ompl::Exception("no goal state found!");
   
   // add states
   while (ptc() == false)
   {
      // first, run bidirectional dijkstra's to find a candidate path
      std::list<Vertex> path_vs;
      std::list<Edge> path_es;
      success = this->dijkstras_bidirectional(pdefdata, path_vs, path_es);
      if (!success)
      {
         printf("no connection found! adding random vertex ...\n");
         ompl::base::State * s_new = this->space->allocState();
         this->sampler->sampleUniform(s_new);
         this->add_vertex(s_new);
         continue;
      }
      
#ifdef DEBUG_DUMPFILE
      {
         fprintf(this->dump_fp, "candidate_path");
         for (std::list<Vertex>::iterator it_v=path_vs.begin(); it_v!=path_vs.end(); it_v++)
         {
            double * q = g[*it_v].s->as<ompl::base::RealVectorStateSpace::StateType>()->values;
            fprintf(this->dump_fp, " %f,%f", q[0], q[1]);
         }
         fprintf(this->dump_fp, "\n");
      }
#endif
      

#if 0
      // print path
      printf("found path:\n");
      std::list<Vertex>::iterator it_v;
      std::list<Edge>::iterator it_e;
      for (it_v=path_vs.begin(),it_e=path_es.begin(); it_v!=path_vs.end(); it_v++,it_e++)
      {
         double * q = g[*it_v].s->as<ompl::base::RealVectorStateSpace::StateType>()->values;
         printf("  vertex: (%f,%f)\n", q[0], q[1]);
         
         if (it_e!=path_es.end())
         {
            Vertex va = boost::source(*it_e, g);
            Vertex vb = boost::target(*it_e, g);
            double * qa = g[va].s->as<ompl::base::RealVectorStateSpace::StateType>()->values;
            double * qb = g[vb].s->as<ompl::base::RealVectorStateSpace::StateType>()->values;
            printf("    edge from (%f,%f) to (%f,%f)\n", qa[0], qa[1], qb[0], qb[1]);
         }
      }
#endif
      
      success = isvalid_path(path_vs, path_es);
      if (success)
      {
         printf("success!\n");
         return ompl::base::PlannerStatus::TIMEOUT;
      }
  
      //if (count >= 2)
      //   break;
   }
   
   return ompl::base::PlannerStatus::TIMEOUT;
}

void P::clear(void)
{
   throw std::runtime_error("clear not implemented!");
}

void P::add_si(const ompl::base::SpaceInformationPtr si, double check_cost)
{
   // make sure it doesnt already exist
   unsigned int ui;
   for (ui=0; ui<this->sis.size(); ui++)
      if (this->sis[ui].first == si)
         break;
   if (ui<this->sis.size())
      throw ompl::Exception("add_si passed si we already know about!");
   // make sure the space matches our space
   if (si->getStateSpace() != this->space)
      throw ompl::Exception("add_si passed si with non-matching state space!");
   // add it!
   this->sis.push_back(std::make_pair(si, check_cost));
}

inline
bool P::dijkstras_bidirectional(ProbDefData & pdefdata,
   std::list<Vertex> & path_vs, std::list<Edge> & path_es)
{
   std::map<Vertex, BiDijkType> closed_fwd;
   std::map<Vertex, BiDijkType> closed_bck;
   std::priority_queue<BiDijkType> open;
   Vertex v_connection = GraphTypes::null_vertex();
   // actually we should set these costs to the expense of checking the roots themselves,
   // which will become important once we're doing multi-root stuff;
   // TODO: does it matter that we're sort of double-counting the cost of the connection vertex?
   open.push(BiDijkType(0.0, pdefdata.v_start, GraphTypes::null_vertex(), Edge(), false));
   open.push(BiDijkType(0.0, pdefdata.v_goal, GraphTypes::null_vertex(), Edge(), true));
   while (!open.empty())
   {
      // pop next
      BiDijkType top = open.top();
      open.pop();
      // get appropriate closed list
      std::map<Vertex, BiDijkType> * closed_mine = &closed_fwd;
      std::map<Vertex, BiDijkType> * closed_other = &closed_bck;
      if (top.isbck)
      {
         closed_mine = &closed_bck;
         closed_other = &closed_fwd;
      }
      // already in closed list?
      if (closed_mine->find(top.v) != closed_mine->end())
         continue;
      // are we done?
      // (i think we should be done when we discover the first overlapping EDGE!!)
      if (closed_other->find(top.v_parent) != closed_other->end())
      {
         v_connection = top.v_parent;
         break;
      }
      // add to closed list!
      closed_mine->insert(std::make_pair(top.v,top));
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
         // get cost of the vertex itself
         update_vertex_estimate(g[v_succ], this->pdef_sii);
         if (g[v_succ].estimates[this->pdef_sii].status == STATUS_INVALID)
            continue;
         if (g[v_succ].estimates[this->pdef_sii].status == STATUS_UNKNOWN)
            new_cost += g[v_succ].estimates[this->pdef_sii].cost_remaining;
         // get cost of the edge
         update_edge_estimate(g[e], this->pdef_sii);
         if (g[e].estimates[this->pdef_sii].status == STATUS_INVALID)
            continue;
         if (g[e].estimates[this->pdef_sii].status == STATUS_UNKNOWN)
            new_cost += g[e].estimates[this->pdef_sii].cost_remaining;
         // add to open list
         open.push(BiDijkType(new_cost, v_succ, top.v, e, top.isbck));
      }
   }
   if (v_connection == GraphTypes::null_vertex())
      return false;
   // create the path
   path_vs.push_back(v_connection);
   for (;;)
   {
      std::map<Vertex, BiDijkType>::iterator it = closed_fwd.find(path_vs.front());
      if (it->second.v_parent == GraphTypes::null_vertex())
         break;
      path_vs.push_front(it->second.v_parent);
      path_es.push_front(it->second.e_to_parent);
   }
   for (;;)
   {
      std::map<Vertex, BiDijkType>::iterator it = closed_bck.find(path_vs.back());
      if (it->second.v_parent == GraphTypes::null_vertex())
         break;
      path_vs.push_back(it->second.v_parent);
      path_es.push_back(it->second.e_to_parent);
   }
   return true;
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
      if (!isvalid_vertex(g[*it_v], this->pdef_sii))
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
         if (!isvalid_vertex(g[*it_v], this->pdef_sii))
         {
            isvalid = false;
            break;
         }
         
         if (!isvalid_edge(g[*it_e], this->pdef_sii))
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
      if (!isvalid_vertex(g[*it_v], this->pdef_sii))
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
         if (!isvalid_vertex(g[*it_v], this->pdef_sii))
         {
            isvalid = false;
            break;
         }
         
         if (!isvalid_edge(g[*it_e], this->pdef_sii))
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
#ifdef DEBUG_DUMPFILE
   double * q = s->as<ompl::base::RealVectorStateSpace::StateType>()->values;
   fprintf(this->dump_fp, "add_vertex %f,%f\n", q[0], q[1]);
#endif
   // as soon as we add a vertex,
   // we must initialize the associated vertex properties
   this->g[v].s = s;
   this->g[v].estimates.clear();
   this->g[v].estimates.resize(this->sis.size()); // defaults to unknown, dirty
   // for now, we find nearest neighbors by just iterating over
   // all existing vertices
   for (std::pair<VertexIter,VertexIter> vp = boost::vertices(this->g);
      vp.first!=vp.second; vp.first++)
   {
      Vertex n = *vp.first;
      if (n == v)
         continue;
      double euc_dist = this->space->distance(this->g[v].s, this->g[n].s);
      if (0.2 < euc_dist)
         continue;
      this->add_edge(v, n);
   }
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
   double * qa = this->g[va].s->as<ompl::base::RealVectorStateSpace::StateType>()->values;
   double * qb = this->g[vb].s->as<ompl::base::RealVectorStateSpace::StateType>()->values;
   fprintf(this->dump_fp, "add_edge %f,%f %f,%f\n", qa[0], qa[1], qb[0], qb[1]);
#endif
   boost::tie(e,success) = boost::add_edge(va, vb, this->g);
   if (!success)
      throw ompl::Exception("edge already exists somehow!");
   euc_dist = this->space->distance(this->g[va].s, this->g[vb].s);
   this->g[e].euc_dist = euc_dist;
   // allocate internal states for this edge
   n = floor(euc_dist / 0.01);
   this->g[e].edgestates.clear();
   this->g[e].edgestates.resize(n);
   const std::vector<int> & perm = this->get_edgeperm(n);
   for (i=0; i<n; i++)
   {
      this->g[e].edgestates[i].s = this->space->allocState();
      space->interpolate(g[va].s, g[vb].s, 1.0*(perm[i]+1)/(n+1), this->g[e].edgestates[i].s);
      this->g[e].edgestates[i].statuses.clear();
      this->g[e].edgestates[i].statuses.resize(this->sis.size(), STATUS_UNKNOWN);
   }
   // set estimates
   this->g[e].estimates.clear();
   this->g[e].estimates.resize(this->sis.size()); // defaults to unknown, dirty
   if (n==0)
   {
      for (i=0; i<this->g[e].estimates.size(); i++)
         this->g[e].estimates[i].status = STATUS_VALID;
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
   this->edgeperms.insert(std::make_pair(n,perm));
   return this->edgeperms[n];
}

inline
void P::update_vertex_estimate(struct VertexProperties & vp, unsigned int sii)
{
   if (vp.estimates[sii].status != STATUS_UNKNOWN) return;
   if (vp.estimates[sii].cost_dirty == false) return;
   
   // for now, assume no relations between si's!
   vp.estimates[sii].cost_remaining = this->sis[sii].second;
   vp.estimates[sii].cost_dirty = false;
}

inline
void P::update_edge_estimate(struct EdgeProperties & ep, unsigned int sii)
{
   int i;
   if (ep.estimates[sii].status != STATUS_UNKNOWN) return;
   if (ep.estimates[sii].cost_dirty == false) return;
   
   // visit each state on the edge
   ep.estimates[sii].cost_remaining = 0.0;
   ep.estimates[sii].cost_dirty = false;
   for (i=0; i<ep.edgestates.size(); i++)
   switch (ep.edgestates[i].statuses[sii])
   {
   case STATUS_INVALID:
      ep.estimates[sii].status = STATUS_INVALID;
      return;
   case STATUS_VALID:
      continue;
   case STATUS_UNKNOWN:
      ep.estimates[sii].cost_remaining += this->sis[sii].second;
      break;
   }
}

inline
bool P::isvalid_vertex(struct VertexProperties & vp, unsigned int sii)
{
   if (vp.estimates[sii].status == STATUS_UNKNOWN)
   {
      if (this->sis[sii].first->isValid(vp.s))
         vp.estimates[sii].status = STATUS_VALID;
      else
         vp.estimates[sii].status = STATUS_INVALID;
      // eventually we will need to set the dirty bit for related si's!
   }
   if (vp.estimates[sii].status == STATUS_VALID)
      return true;
   else
      return false;
}

inline
bool P::isvalid_edge(struct EdgeProperties & ep, unsigned int sii)
{
   if (ep.estimates[sii].status == STATUS_UNKNOWN)
   {
      int i;
      for (i=0; i<ep.edgestates.size(); i++)
      {
         if (ep.edgestates[i].statuses[sii] == STATUS_UNKNOWN)
         {
            if (this->sis[sii].first->isValid(ep.edgestates[i].s))
               ep.edgestates[i].statuses[sii] = STATUS_VALID;
            else
               ep.edgestates[i].statuses[sii] = STATUS_INVALID;
         }
         if (ep.edgestates[i].statuses[sii] != STATUS_VALID)
         {
            ep.estimates[sii].status = STATUS_INVALID;
            break;
         }            
      }
      if (!(i<ep.edgestates.size()))
         ep.estimates[sii].status = STATUS_VALID;
      // eventually we will need to set the dirty bit for related si's!
   }
   if (ep.estimates[sii].status == STATUS_VALID)
      return true;
   else
      return false;
}






#if 0
void checkmask::GraphImpl::add_inclusion(std::pair<std::string,std::string> supersub)
{
}

void checkmask::GraphImpl::remove_inclusion(std::pair<std::string,std::string> supersub)
{
}

std::pair<std::string,std::string> checkmask::GraphImpl::get_inclusions()
{
}
#endif
