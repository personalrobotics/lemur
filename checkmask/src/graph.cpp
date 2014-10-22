#include <cstdio>
#include <string>
#include <vector>
#include <boost/function.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <ompl/base/Planner.h>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <checkmask/graph.h>

namespace checkmask
{

class GraphPlannerImpl : public checkmask::GraphPlanner
{
public:
   GraphPlannerImpl(const ompl::base::StateSpacePtr & space,
      const ompl::base::SpaceInformationPtr & si_bogus);
   ~GraphPlannerImpl(void);

   // ompl planner interface
   void setProblemDefinition(const ompl::base::ProblemDefinitionPtr & pdef);
   ompl::base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition & ptc);
   void clear(void);

#if 0
   void add_space(std::string spacename, double check_cost,
      boost::function<bool (const double * q)> is_valid);
   void remove_space(std::string spacename);
   std::vector<std::string> get_spaces();
   
   void add_inclusion(std::pair<std::string,std::string> supersub);
   void remove_inclusion(std::pair<std::string,std::string> supersub);
   std::pair<std::string,std::string> get_inclusions();
   
   void add_problem(std::string probname,
      std::vector<std::string> spaces,
      const double * q_start, const double * q_goal);
   
   int get_n_vertices();
   void add_vertices(int n);
   
   bool solve_problem(std::string probname);
#endif

private:

   // types
   struct VertexProperties
   {
      std::vector<double> q;
   };
   struct EdgeProperties
   {
      double dist;
   };
   typedef boost::adjacency_list<
      boost::vecS, // Edgelist ds, for per-vertex out-edges
      boost::vecS, // VertexList ds, for vertex set
      boost::undirectedS, // type of graph
      VertexProperties, EdgeProperties // bundled internal properties
      > Graph;
   typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
   typedef boost::graph_traits<Graph>::vertex_iterator VertexIter;
   typedef boost::graph_traits<Graph>::edge_descriptor Edge;
   typedef boost::graph_traits<Graph>::out_edge_iterator EdgeOutIter;
   
   // we have a graph g
   Graph g;
   
   // this planner can only work with a single space,
   // provided on construction
   // (of course we can handle many different spaceinformations,
   //  as long as they all share the same space and parameters
   ompl::base::StateSpacePtr space;
   
};

} // namespace checkmask

checkmask::GraphPlanner * checkmask::GraphPlanner::create(const ompl::base::StateSpacePtr & space)
{
   ompl::base::SpaceInformationPtr si_bogus(new ompl::base::SpaceInformation(space));
   si_bogus->setStateValidityChecker(
      ompl::base::StateValidityCheckerPtr(
         new ompl::base::AllValidStateValidityChecker(si_bogus)
      )
   );
   si_bogus->setup();
   return new checkmask::GraphPlannerImpl(space, si_bogus);
}

checkmask::GraphPlannerImpl::GraphPlannerImpl(
      const ompl::base::StateSpacePtr & space,
      const ompl::base::SpaceInformationPtr & si):
   checkmask::GraphPlanner(si), space(space)
{
   printf("constructor called!\n");
}

checkmask::GraphPlanner::GraphPlanner(const ompl::base::SpaceInformationPtr & si_bogus):
   ompl::base::Planner(si_bogus, "CheckMaskGraph")
{
}

checkmask::GraphPlannerImpl::~GraphPlannerImpl()
{
   printf("destructor called!\n");
}

void checkmask::GraphPlannerImpl::setProblemDefinition(const ompl::base::ProblemDefinitionPtr & pdef)
{
   throw std::runtime_error("setProblemDefinition not implemented!");
}

ompl::base::PlannerStatus checkmask::GraphPlannerImpl::solve(const ompl::base::PlannerTerminationCondition & ptc)
{
   throw std::runtime_error("solve not implemented!");
}

void checkmask::GraphPlannerImpl::clear(void)
{
   throw std::runtime_error("clear not implemented!");
}














#if 0

void checkmask::GraphImpl::add_space(std::string spacename, double check_cost,
   boost::function<bool (const double * q)> is_valid)
{
}

void checkmask::GraphImpl::remove_space(std::string spacename)
{
}

std::vector<std::string> checkmask::GraphImpl::get_spaces()
{
}

void checkmask::GraphImpl::add_inclusion(std::pair<std::string,std::string> supersub)
{
}

void checkmask::GraphImpl::remove_inclusion(std::pair<std::string,std::string> supersub)
{
}

std::pair<std::string,std::string> checkmask::GraphImpl::get_inclusions()
{
}

void checkmask::GraphImpl::add_problem(std::string probname,
   std::vector<std::string> spaces,
   const double * q_start, const double * q_goal)
{
}

int checkmask::GraphImpl::get_n_vertices()
{
}

void checkmask::GraphImpl::add_vertices(int n)
{
}

bool checkmask::GraphImpl::solve_problem(std::string probname)
{
}
#endif
