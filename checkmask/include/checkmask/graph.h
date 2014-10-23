/* requires:
#include <string>
#include <vector>
#include <boost/function.hpp>
 
#include <ompl/base/Planner.h>
*/

namespace checkmask
{

class GraphPlanner : public ompl::base::Planner
{
public:
   static GraphPlanner * create(const ompl::base::StateSpacePtr & space);
   GraphPlanner(const ompl::base::SpaceInformationPtr & si_bogus);
   virtual ~GraphPlanner(void) {};

   // ompl planner interface
   virtual void setProblemDefinition(const ompl::base::ProblemDefinitionPtr & pdef) = 0;
   virtual ompl::base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition & ptc) = 0;
   virtual void clear(void) = 0;

   // this will resize our list of sis,
   // update our check model,
   // and update the check mask on all vertices/edges;
   // if setProblemDefinition is called refrerncing an unknown si,
   // this function will be called with a default cost of 1.0
   virtual void add_si(const ompl::base::SpaceInformationPtr si, double check_cost) = 0;

#if 0
   /* each space has an associate checker for validity
    * these are Cfree's */
   virtual void add_space(std::string spacename, double check_cost,
      boost::function<bool (const double * q)> is_valid) = 0;
   virtual void remove_space(std::string spacename) = 0;
   virtual std::vector<std::string> get_spaces() = 0;
   
   
   /* these pairs are superset,subset */
   virtual void add_inclusion(std::pair<std::string,std::string> supersub) = 0;
   virtual void remove_inclusion(std::pair<std::string,std::string> supersub) = 0;
   virtual std::pair<std::string,std::string> get_inclusions() = 0;
   
   /* each problem is within the intersection of spaces */
   virtual void add_problem(std::string probname,
      std::vector<std::string> spaces,
      const double * q_start, const double * q_goal) = 0;
   
   /* eventually we'll also have problems that goes between
    * sets of starts and goals */
   
   /* this keeps working on a problem until it's solved */
   virtual int get_n_vertices() = 0;
   virtual void add_vertices(int n) = 0;
   
   /* attempt to solve the named problem;
    * returns true if path found, false otherwise */
   virtual bool solve_problem(std::string probname) = 0;
#endif

};

} // namespace checkmask

