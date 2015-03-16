#include <openrave/openrave.h>
#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>

#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/Planner.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>

#include <ompl_multiset/Roadmap.h>
#include <ompl_multiset/RoadmapSampledConst.h>
#include <ompl_multiset/Cache.h>
#include <ompl_multiset/MultiSetPRM.h>

#include <or_multiset/inter_link_checks.h>

#include "params_checkmask.h"
#include "module_subset_manager.h"
#include "planner_multiset_prm.h"

namespace {

std::string sf(const char * fmt, ...)
{
  va_list ap;
  va_start(ap, fmt);
  int size = vsnprintf(0, 0, fmt, ap);
  va_end(ap);
  char * buf = new char[size+1];
  va_start(ap, fmt);
  vsnprintf(buf, size+1, fmt, ap);
  va_end(ap);
  std::string ret = std::string(buf);
  delete[] buf;
  return ret;
}

} // anonymous namespace


or_multiset::MultiSetPRM::MultiSetPRM(OpenRAVE::EnvironmentBasePtr penv):
   OpenRAVE::PlannerBase(penv), penv(penv)
{
   __description = "OmplCheckMask description";
   this->RegisterCommand("UseSubsetManager",
      boost::bind(&or_multiset::MultiSetPRM::UseSubsetManager,this,_1,_2),
      "UseSubsetManager");
   this->RegisterCommand("GetTimes",
      boost::bind(&or_multiset::MultiSetPRM::GetTimes,this,_1,_2),
      "GetTimes");
   printf("constructed!\n");
}

or_multiset::MultiSetPRM::~MultiSetPRM()
{
   //if (this->p) delete this->p;
   printf("destructed!\n");
}

bool or_multiset::MultiSetPRM::InitPlan(OpenRAVE::RobotBasePtr robot, std::istream & sin)
{
   or_multiset::PlannerParametersPtr my_params(new or_multiset::PlannerParameters());
   sin >> *my_params;
   RAVELOG_WARN("skipping custom PlannerParameters validation due to exception!\n");
   //my_params->Validate();
   return this->InitMyPlan(robot, my_params);
}

bool or_multiset::MultiSetPRM::InitPlan(OpenRAVE::RobotBasePtr robot, OpenRAVE::PlannerBase::PlannerParametersConstPtr params)
{
   // is this one of ours?
   or_multiset::PlannerParametersConstPtr my_params = boost::dynamic_pointer_cast<or_multiset::PlannerParameters const>(params);
   if (!my_params)
   {
      // if not, serialize and call our serialized constructor
      RAVELOG_WARN("Warning, MultiSetPRM planner passed an unknown PlannerParameters type! Attempting to serialize ...\n");
      std::stringstream ss;
      ss << *params;
      return this->InitPlan(robot, ss);
   }
   return this->InitMyPlan(robot, my_params);
}
   
bool or_multiset::MultiSetPRM::InitMyPlan(OpenRAVE::RobotBasePtr robot, or_multiset::PlannerParametersConstPtr params)
{   
   // save the robot and active dofs
   if (!this->setup_isvalid(robot))
   {
      RAVELOG_INFO("doing MultiSetPRM setup ...\n");
      this->setup(robot);
   }
   
   // get subset manager (will construct private if not attached)
   boost::shared_ptr<or_multiset::ModuleSubsetManager> ssm = this->get_subset_manager();
   if (!ssm)
      throw OpenRAVE::openrave_exception("could not retreive a subset manager!");
   
   // save params
   this->params = params;
   return true;
}

OpenRAVE::PlannerBase::PlannerParametersConstPtr or_multiset::MultiSetPRM::GetParameters() const
{
   return this->params;
}

OpenRAVE::PlannerStatus or_multiset::MultiSetPRM::PlanPath(OpenRAVE::TrajectoryBasePtr ptraj)
{
   ompl::base::PlannerStatus status;
   
   // get robot and ensure setup validity
   OpenRAVE::RobotBasePtr robot(this->robot.lock());
   if (!robot || !this->setup_isvalid(robot))
      throw OpenRAVE::openrave_exception("something changed with robot or active dofs or something!");
   
   if (!this->params)
      throw OpenRAVE::openrave_exception("no params! was initplan called?");
   
   // get subset manager (will construct private if not attached)
   boost::shared_ptr<or_multiset::ModuleSubsetManager> ssm = this->get_subset_manager();
   if (!ssm)
      throw OpenRAVE::openrave_exception("could not retreive a subset manager!");
   
   // get current subset report
   or_multiset::SubsetReport report;
   ssm->get_current_report(robot, report);
   
   // tell planner about any new subsets
   for (unsigned int si=0; si<report.subsets.size(); si++)
   {
      if (this->subsets.find(report.subsets[si].name) == this->subsets.end())
      {
         // create new space_info
         ompl::base::SpaceInformationPtr space_info(
            new ompl::base::SpaceInformation(this->ompl_space));
         // bind to the subset manarger's indicator function
         space_info->setStateValidityChecker(boost::bind(
            &or_multiset::MultiSetPRM::ompl_isvalid, this,
            report.subsets[si].indicator, _1));
         // tell the planner about the spaceinfo
         this->ompl_planner->add_cfree(
            space_info,
            report.subsets[si].name,
            report.subsets[si].cost);
         this->subsets[report.subsets[si].name] = space_info;
         RAVELOG_INFO("added si \"%s\" with cost %f\n",
            report.subsets[si].name.c_str(),
            report.subsets[si].cost);
      }
   }
   
   // tell planner about any new intersections
   RAVELOG_WARN("we're not yet telling planner about intersections ...\n");
   
   // get current subset
   RAVELOG_INFO("planning in subset \"%s\" ...\n", report.current_subset.c_str());
   std::map<std::string, ompl::base::SpaceInformationPtr>::iterator
      subset_it = this->subsets.find(report.current_subset);
   if (subset_it == this->subsets.end())
      throw OpenRAVE::openrave_exception("cannot find current subset!");
   
   // create a new problem definition
   ompl::base::ProblemDefinitionPtr pdef(
      new ompl::base::ProblemDefinition(subset_it->second));
   
   // add start states
   pdef->clearStartStates();
   for (unsigned int si=0; si<this->params->startstates.size(); si++)
   {
      if (params->startstates[si].size() != this->ompl_space->getDimension())
         throw OpenRAVE::openrave_exception("start state not the right dimension!");
      printf("found a start state:");
      ompl::base::ScopedState<ompl::base::RealVectorStateSpace> s_start(this->ompl_space);
      for (unsigned int j=0; j<this->params->startstates[si].size(); j++)
      {
         printf(" %f", this->params->startstates[si][j]);
         s_start->values[j] = this->params->startstates[si][j];
      }
      printf("\n");
      pdef->addStartState(s_start);
   }
   
   // add goal states
   ompl::base::GoalStates * gs = new ompl::base::GoalStates(pdef->getSpaceInformation());
   gs->clear();
   for (unsigned int si=0; si<this->params->goalstates.size(); si++)
   {
      if (params->goalstates[si].size() != this->ompl_space->getDimension())
         throw OpenRAVE::openrave_exception("goal state not the right dimension!");
      printf("found a goal state:");
      ompl::base::ScopedState<ompl::base::RealVectorStateSpace> s_goal(this->ompl_space);
      for (unsigned int j=0; j<this->params->goalstates[si].size(); j++)
      {
         printf(" %f", this->params->goalstates[si][j]);
         s_goal->values[j] = this->params->goalstates[si][j];
      }
      printf("\n");
      gs->addState(s_goal);
   }
   pdef->clearGoal();
   pdef->setGoal(ompl::base::GoalPtr(gs));
   
   // tell the planner about our problem definition
   this->ompl_planner->setProblemDefinition(pdef);
   
   // call planner
   this->n_checks = 0;
   this->checktime = 0;
   this->totaltime = 0;
   
   struct timespec tic;
   struct timespec toc;
   clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &tic);
   
   {
      // Don't check collision with inactive links.
      //OpenRAVE::CollisionCheckerBasePtr const collision_checker = GetEnv()->GetCollisionChecker();
      //OpenRAVE::CollisionOptionsStateSaver const collision_saver(collision_checker, OpenRAVE::CO_ActiveDOFs, false);

      status = this->ompl_planner->solve(ompl::base::timedPlannerTerminationCondition(600.0));
   }
   
   clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &toc);
   this->totaltime = (toc.tv_nsec - tic.tv_nsec) + 1000000000*(toc.tv_sec - tic.tv_sec);
   
   // check results
   printf("planner returned: %s\n", status.asString().c_str());
   if (status != ompl::base::PlannerStatus::EXACT_SOLUTION) return OpenRAVE::PS_Failed;
   
   // convert result
   ompl::base::PathPtr path = this->ompl_planner->getProblemDefinition()->getSolutionPath();
   ompl::geometric::PathGeometric * gpath = dynamic_cast<ompl::geometric::PathGeometric*>(path.get());
   if (!gpath)
      throw OpenRAVE::openrave_exception("ompl path is not geometric for some reason!");
      
   ptraj->Init(robot->GetActiveConfigurationSpecification());
   for (unsigned int i=0; i<gpath->getStateCount(); i++)
   {
      ompl::base::ScopedState<ompl::base::RealVectorStateSpace> s(this->ompl_space);
      s = gpath->getState(i);
      std::vector<OpenRAVE::dReal> vec(&s[0], &s[0]+robot->GetActiveDOF());
      ptraj->Insert(i, vec);
   }
   
   return OpenRAVE::PS_HasSolution;
}

bool or_multiset::MultiSetPRM::UseSubsetManager(std::ostream & sout, std::istream & sin)
{
   std::string name;
   
   // parse args
   {
      std::vector<std::string> args;
      for (;;)
      {
         std::string arg;
         sin >> arg;
         if (sin.fail())
            break;
         args.push_back(arg);
      }
      if (args.size() != 1)
         throw OpenRAVE::openrave_exception("TagCurrentSubset args not correct!");
      name = args[0];
   }
   
   // find the subset manager module in the env with this name
   std::vector< boost::shared_ptr<or_multiset::ModuleSubsetManager> > matches;
   std::list<OpenRAVE::ModuleBasePtr> penv_modules;
   std::list<OpenRAVE::ModuleBasePtr>::iterator it;
   this->penv->GetModules(penv_modules);
   for (it=penv_modules.begin(); it!=penv_modules.end(); it++)
   {
      boost::shared_ptr<or_multiset::ModuleSubsetManager> mod
         = boost::dynamic_pointer_cast<or_multiset::ModuleSubsetManager>(*it);
      if (!mod)
         continue;
      if (mod->name != name)
         continue;
      matches.push_back(mod);
   }
   
   if (matches.size() == 0)
   {
      RAVELOG_ERROR("No subset manager module has name \"%s\"!\n", name.c_str());
      return false;
   }
   
   if (matches.size() != 1)
   {
      RAVELOG_ERROR("Not exactly one subset manager module has name \"%s\"! (THIS SHOULD NEVER HAPPEN)\n", name.c_str());
      return false;
   }
   
   this->subset_manager = matches[0];
   this->private_subset_manager.reset();
   return true;
}

bool or_multiset::MultiSetPRM::GetTimes(std::ostream & sout, std::istream & sin)
{
   sout << "checktime " << (1.0e-9 * this->checktime);
   sout << " totaltime " << (1.0e-9 * this->totaltime);
   sout << " n_checks " << this->n_checks;
   return true;
}

boost::shared_ptr<or_multiset::ModuleSubsetManager>
or_multiset::MultiSetPRM::get_subset_manager()
{
   boost::shared_ptr<or_multiset::ModuleSubsetManager> ssm = this->subset_manager.lock();
   if (ssm)
      return ssm;
   RAVELOG_INFO("MultiSetPRM using private subset manager.\n");
   OpenRAVE::ModuleBasePtr mod = OpenRAVE::RaveCreateModule(this->penv, "SubsetManager");
   this->private_subset_manager = boost::dynamic_pointer_cast<or_multiset::ModuleSubsetManager>(mod);
   this->subset_manager = this->private_subset_manager;
   return this->private_subset_manager;
}

ompl::base::RealVectorBounds or_multiset::MultiSetPRM::ompl_bounds(OpenRAVE::RobotBasePtr robot)
{
   ompl::base::RealVectorBounds bounds(robot->GetActiveDOF());
   std::vector<OpenRAVE::dReal> lowers;
   std::vector<OpenRAVE::dReal> uppers;
   robot->GetActiveDOFLimits(lowers, uppers);
   for (int i=0; i<robot->GetActiveDOF(); i++)
   {
      bounds.setLow(i, lowers[i]);
      bounds.setHigh(i, uppers[i]);
   }
   return bounds;
}

void or_multiset::MultiSetPRM::setup(OpenRAVE::RobotBasePtr robot)
{
   this->robot = robot;
   this->adofs = robot->GetActiveDOFIndices();

   /* space: right arm dofs */
   this->ompl_space.reset(new ompl::base::RealVectorStateSpace(robot->GetActiveDOF()));
   
   /* state space set bounds */
   this->ompl_space->as<ompl::base::RealVectorStateSpace>()->setBounds(this->ompl_bounds(robot));
      
   /* set space resolution */
   std::vector<OpenRAVE::dReal> dof_resolutions;
   robot->GetActiveDOFResolutions(dof_resolutions);
   double resolution = HUGE_VAL;
   for (unsigned int i=0; i<dof_resolutions.size(); i++)
      resolution = dof_resolutions[i] < resolution ? dof_resolutions[i] : resolution;
   RAVELOG_INFO("WOULD set resolution to %f rad!\n", resolution);
   resolution = 0.05;
   RAVELOG_WARN("HARDCODING resolution to %f rad!\n", resolution);
   this->ompl_space->setLongestValidSegmentFraction(
      resolution / this->ompl_space->getMaximumExtent());

   // create planner itself
   ompl_multiset::RoadmapPtr roadmap(
      new ompl_multiset::RoadmapSampledConst(this->ompl_space, 419884521, 1000, 2.0));
   this->ompl_planner.reset(ompl_multiset::MultiSetPRM::create(this->ompl_space,
      roadmap, ompl_multiset::CachePtr()));
   this->ompl_planner->set_interroot_radius(2.0);
   this->ompl_planner->set_lambda(0.0001);
   this->subsets.clear();
}

// check of the robot, adofs, bounds
// TODO: resolutions?
// retrieved during initplan match that of the given robot
bool or_multiset::MultiSetPRM::setup_isvalid(OpenRAVE::RobotBasePtr robot)
{
   // check robot
   OpenRAVE::RobotBasePtr my_robot = this->robot.lock();
   if (!my_robot || !robot || my_robot != robot)
      return false;
   // check active dofs
   if (robot->GetActiveDOFIndices() != this->adofs)
      return false;
   // check space dimension, bounds
   if (!this->ompl_space)
      return false;
   if ((int)this->ompl_space->getDimension() != robot->GetActiveDOF())
      return false;
   ompl::base::RealVectorBounds robot_bounds = this->ompl_bounds(robot);
   ompl::base::RealVectorBounds current_bounds
      = this->ompl_space->as<ompl::base::RealVectorStateSpace>()->getBounds();
   if (robot_bounds.low != current_bounds.low) return false;
   if (robot_bounds.high != current_bounds.high) return false;
   return true;
}

bool or_multiset::MultiSetPRM::ompl_isvalid(
   boost::function<bool (std::vector<OpenRAVE::dReal> &)> indicator,
   const ompl::base::State * state)
{
   //printf("checking ...\n");
   if (!indicator)
   {
      RAVELOG_ERROR("No indicator function provided!\n");
      return false;
   }
   
   bool isvalid;
   
   double * q = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
   std::vector<OpenRAVE::dReal> adofvals(q, q+this->ompl_space->getDimension());
   
   struct timespec tic;
   struct timespec toc;
   clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &tic);
   
   isvalid = indicator(adofvals);

   clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &toc);
   this->checktime += (toc.tv_nsec - tic.tv_nsec) + 1000000000*(toc.tv_sec - tic.tv_sec);
   this->n_checks++;
   return isvalid;
}
