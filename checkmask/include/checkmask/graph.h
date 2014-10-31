/* requires:
#include <string>
#include <vector>
#include <ompl/base/Planner.h>
*/

namespace checkmask
{

class GraphPlanner : public ompl::base::Planner
{
public:
   static GraphPlanner * create(const ompl::base::StateSpacePtr & space);
   virtual ~GraphPlanner(void) {};

   // ompl planner interface
   virtual void setProblemDefinition(const ompl::base::ProblemDefinitionPtr & pdef) = 0;
   virtual ompl::base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition & ptc) = 0;
   virtual void clear(void) = 0;
   
   // PRM parameters
   virtual void set_batchsize(int batchsize) = 0;
   virtual void set_radius(double radius) = 0;
   virtual void set_resolution(double resolution) = 0;
   
   virtual void set_dumpfile(const char * dumpfile) = 0;

   // this will resize our list of sis,
   // update our check model,
   // and update the check mask on all vertices/edges;
   // if setProblemDefinition is called refrerncing an unknown si,
   // this function will be called with a default cost of 1.0
   virtual void add_cfree(const ompl::base::SpaceInformationPtr si, std::string name, double check_cost) = 0;

   // add relations between cfrees
   virtual void add_inclusion(
      const ompl::base::SpaceInformationPtr si_superset,
      const ompl::base::SpaceInformationPtr si_subset) = 0;
   virtual void add_intersection(
      const ompl::base::SpaceInformationPtr si_a,
      const ompl::base::SpaceInformationPtr si_b,
      const ompl::base::SpaceInformationPtr si_intersection) = 0;

protected:
   GraphPlanner(const ompl::base::SpaceInformationPtr & si, const std::string & name):
      ompl::base::Planner(si,name) {}
};

} // namespace checkmask

