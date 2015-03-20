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

std::vector<std::string> args_from_sin(std::istream & sin)
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
   return args;
}

} // anonymous namespace


or_multiset::MultiSetPRM::PlannerParameters::PlannerParameters():
   eval_subgraphs(0), lambda(1.0), interroot_radius(1.0),
   timelimit(600.0)
{
   _vXMLParameters.push_back("startstate");
   _vXMLParameters.push_back("goalstate");
   _vXMLParameters.push_back("eval_subgraphs");
   _vXMLParameters.push_back("lambda");
   _vXMLParameters.push_back("interroot_radius");
   _vXMLParameters.push_back("timelimit");
}

void or_multiset::MultiSetPRM::PlannerParameters::serialize_startstates(std::ostream& sout) const
{
   for (unsigned int si=0; si<startstates.size(); si++)
   {
      sout << "<startstate>";
      for (unsigned int j=0; j<this->startstates[si].size(); j++)
      {
         if (j) sout << " ";
         sout << this->startstates[si][j];
      }
      sout << "</startstate>" << std::endl;
   }
}

void or_multiset::MultiSetPRM::PlannerParameters::serialize_goalstates(std::ostream& sout) const
{
   for (unsigned int si=0; si<goalstates.size(); si++)
   {
      sout << "<goalstate>";
      for (unsigned int j=0; j<this->goalstates[si].size(); j++)
      {
         if (j) sout << " ";
         sout << this->goalstates[si][j];
      }
      sout << "</goalstate>" << std::endl;
   }
}

void or_multiset::MultiSetPRM::PlannerParameters::serialize_eval_subgraphs(std::ostream & sout) const
{
   sout << "<eval_subgraphs>";
   sout << this->eval_subgraphs;
   sout << "</eval_subgraphs>";
}

void or_multiset::MultiSetPRM::PlannerParameters::serialize_lambda(std::ostream& sout) const
{
   sout << "<lambda>";
   sout << this->lambda;
   sout << "</lambda>";
}

void or_multiset::MultiSetPRM::PlannerParameters::serialize_interroot_radius(std::ostream& sout) const
{
   sout << "<interroot_radius>";
   sout << this->interroot_radius;
   sout << "</interroot_radius>";
}

void or_multiset::MultiSetPRM::PlannerParameters::serialize_timelimit(std::ostream& sout) const
{
   sout << "<timelimit>";
   sout << this->timelimit;
   sout << "</timelimit>";
}

void or_multiset::MultiSetPRM::PlannerParameters::deserialize_startstate(std::istream & sin)
{
   std::vector<OpenRAVE::dReal> state;
   while (sin.good())
   {
      OpenRAVE::dReal val;
      sin >> val;
      state.push_back(val);
   }
   this->startstates.push_back(state);
}

void or_multiset::MultiSetPRM::PlannerParameters::deserialize_goalstate(std::istream & sin)
{
   std::vector<OpenRAVE::dReal> state;
   while (sin.good())
   {
      OpenRAVE::dReal val;
      sin >> val;
      state.push_back(val);
   }
   this->goalstates.push_back(state);
}

void or_multiset::MultiSetPRM::PlannerParameters::deserialize_eval_subgraphs(std::istream & sin)
{
   sin >> this->eval_subgraphs;
}

void or_multiset::MultiSetPRM::PlannerParameters::deserialize_lambda(std::istream & sin)
{
   sin >> this->lambda;
}

void or_multiset::MultiSetPRM::PlannerParameters::deserialize_interroot_radius(std::istream & sin)
{
   sin >> this->interroot_radius;
}

void or_multiset::MultiSetPRM::PlannerParameters::deserialize_timelimit(std::istream & sin)
{
   sin >> this->timelimit;
}

bool or_multiset::MultiSetPRM::PlannerParameters::serialize(std::ostream& sout, int options) const
{
   if (!OpenRAVE::PlannerBase::PlannerParameters::serialize(sout))
      return false;
   this->serialize_startstates(sout);
   this->serialize_goalstates(sout);
   this->serialize_eval_subgraphs(sout);
   this->serialize_lambda(sout);
   this->serialize_interroot_radius(sout);
   this->serialize_timelimit(sout);
   return !!sout;
}

OpenRAVE::BaseXMLReader::ProcessElement or_multiset::MultiSetPRM::PlannerParameters::startElement(
   const std::string & name, const OpenRAVE::AttributesList & atts)
{
   if (el_deserializing.size())
      return PE_Ignore;
   // ask base calss
   enum OpenRAVE::BaseXMLReader::ProcessElement base;
   base = OpenRAVE::PlannerBase::PlannerParameters::startElement(name,atts);
   if (base != PE_Pass) return base;
   // can we handle it?
   if (name == "startstate"
      || name == "goalstate"
      || name == "eval_subgraphs"
      || name == "lambda"
      || name == "interroot_radius"
      || name == "timelimit")
   {
      el_deserializing = name;
      return PE_Support;
   }
   return PE_Pass;
}

bool or_multiset::MultiSetPRM::PlannerParameters::endElement(const std::string & name)
{
   if (!el_deserializing.size())
      return OpenRAVE::PlannerBase::PlannerParameters::endElement(name);
   if (name == el_deserializing)
   {
      if (el_deserializing == "startstate")
         this->deserialize_startstate(_ss);
      if (el_deserializing == "goalstate")
         this->deserialize_goalstate(_ss);
      if (el_deserializing == "eval_subgraphs")
         this->deserialize_eval_subgraphs(_ss);
      if (el_deserializing == "lambda")
         this->deserialize_lambda(_ss);
      if (el_deserializing == "interroot_radius")
         this->deserialize_interroot_radius(_ss);
      if (el_deserializing == "timelimit")
         this->deserialize_timelimit(_ss);
   }
   else
      RAVELOG_WARN("closing tag doesnt match opening tag!\n");
   el_deserializing.clear();
   return false;
}


or_multiset::MultiSetPRM::MultiSetPRM(OpenRAVE::EnvironmentBasePtr penv):
   OpenRAVE::PlannerBase(penv), penv(penv),
   roadmap_string("class=RoadmapSampledConst seed=419884521 batch_n=1000 radius=2")
{
   __description = "OmplCheckMask description";
   this->RegisterCommand("UseSubsetManager",
      boost::bind(&or_multiset::MultiSetPRM::UseSubsetManager,this,_1,_2),
      "UseSubsetManager");
   this->RegisterCommand("SetRoadmap",
      boost::bind(&or_multiset::MultiSetPRM::SetRoadmap,this,_1,_2),
      "SetRoadmap");
   this->RegisterCommand("GetTimes",
      boost::bind(&or_multiset::MultiSetPRM::GetTimes,this,_1,_2),
      "GetTimes");
   this->RegisterCommand("CacheSetLocation",
      boost::bind(&or_multiset::MultiSetPRM::CacheSetLocation,this,_1,_2),
      "CacheSetLocation");
   this->RegisterCommand("CacheLoad",
      boost::bind(&or_multiset::MultiSetPRM::CacheLoad,this,_1,_2),
      "CacheLoad");
   this->RegisterCommand("CacheSave",
      boost::bind(&or_multiset::MultiSetPRM::CacheSave,this,_1,_2),
      "CacheSave");
}

or_multiset::MultiSetPRM::~MultiSetPRM()
{
}

bool or_multiset::MultiSetPRM::InitPlan(OpenRAVE::RobotBasePtr robot, std::istream & sin)
{
   PlannerParametersPtr my_params(new PlannerParameters());
   sin >> *my_params;
   RAVELOG_WARN("skipping custom PlannerParameters validation due to exception!\n");
   //my_params->Validate();
   return this->InitMyPlan(robot, my_params);
}

bool or_multiset::MultiSetPRM::InitPlan(OpenRAVE::RobotBasePtr robot, OpenRAVE::PlannerBase::PlannerParametersConstPtr params)
{
   // is this one of ours?
   PlannerParametersConstPtr my_params = boost::dynamic_pointer_cast<PlannerParameters const>(params);
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
   
bool or_multiset::MultiSetPRM::InitMyPlan(OpenRAVE::RobotBasePtr robot, PlannerParametersConstPtr params)
{   
   // save the robot and active dofs
   if (!this->setup_isvalid(robot))
   {
      RAVELOG_INFO("doing MultiSetPRM setup ...\n");
      this->setup(robot);
   }

   // get subset manager (will construct private if not attached)
   // this can throw
   update_planner_current_subsets(robot);
   
   // save params
   this->params = params;
   return true;
}

OpenRAVE::PlannerBase::PlannerParametersConstPtr or_multiset::MultiSetPRM::GetParameters() const
{
   return this->params;
}

std::string or_multiset::MultiSetPRM::update_planner_current_subsets(
   OpenRAVE::RobotBasePtr robot)
{
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
   for (unsigned int si=0; si<report.intersections.size(); si++)
   {
      or_multiset::SubsetReport::Intersection & isec
         = report.intersections[si];
      std::pair<std::string, std::vector<std::string> > pair
         = std::make_pair(isec.subset, isec.supersets);
      if (this->intersections.find(pair) == this->intersections.end())
      {
         // find subset
         std::map<std::string, ompl::base::SpaceInformationPtr>::iterator
            info_subset = this->subsets.find(isec.subset);
         if (info_subset == this->subsets.end())
            throw OpenRAVE::openrave_exception(sf(
               "couldnt find subset %s!",isec.subset.c_str()));
         // find each superset
         std::string supersets_string;
         std::vector<ompl::base::SpaceInformationPtr> supersets;
         for (unsigned int ui=0; ui<isec.supersets.size(); ui++)
         {
            std::map<std::string, ompl::base::SpaceInformationPtr>::iterator
               info_superset = this->subsets.find(isec.supersets[ui]);
            if (info_superset == this->subsets.end())
               throw OpenRAVE::openrave_exception(sf(
                  "couldnt find subset %s!",isec.supersets[ui].c_str()));
            supersets.push_back(info_superset->second);
            if (supersets_string.size())
               supersets_string += " n ";
            supersets_string += "\"" + isec.supersets[ui] + "\"";
         }
         this->ompl_planner->add_intersection(
            info_subset->second, supersets);
         this->intersections.insert(pair);
         RAVELOG_INFO("added intersection \"%s\" = %s\n",
            isec.subset.c_str(),
            supersets_string.c_str());
      }
   }
   
   RAVELOG_WARN("we're not yet telling planner about inclusions ...\n");
   
   return report.current_subset;
}

OpenRAVE::PlannerStatus or_multiset::MultiSetPRM::PlanPath(OpenRAVE::TrajectoryBasePtr ptraj)
{
   //OpenRAVE::EnvironmentMutex::scoped_lock lock(penv->GetMutex());
   
   ompl::base::PlannerStatus status;
   
   // get robot and ensure setup validity
   OpenRAVE::RobotBasePtr robot(this->robot.lock());
   if (!robot || !this->setup_isvalid(robot))
      throw OpenRAVE::openrave_exception("something changed with robot or active dofs or something!");
   
   if (!this->params)
      throw OpenRAVE::openrave_exception("no params! was initplan called?");
   
   // this can throw
   std::string current_subset = update_planner_current_subsets(robot);
   
   // get current subset
   RAVELOG_INFO("planning in subset \"%s\" ...\n", current_subset.c_str());
   std::map<std::string, ompl::base::SpaceInformationPtr>::iterator
      subset_it = this->subsets.find(current_subset);
   if (subset_it == this->subsets.end())
      throw OpenRAVE::openrave_exception("cannot find current subset!");
      
   if (this->params->eval_subgraphs)
   {
      RAVELOG_INFO("evaluating everything ...\n");
      this->ompl_planner->use_num_subgraphs(this->params->eval_subgraphs);
      this->ompl_planner->eval_everything(subset_it->second);
      return OpenRAVE::PS_HasSolution;
   }

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
   
   // set planner parameters
   this->ompl_planner->set_interroot_radius(this->params->interroot_radius);
   this->ompl_planner->set_lambda(this->params->lambda);
   
   // call planner
   this->n_checks = 0;
   this->checktime = 0;
   this->totaltime = 0;
   
   struct timespec tic;
   struct timespec toc;
   clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &tic);
   
   status = this->ompl_planner->solve(ompl::base::timedPlannerTerminationCondition(this->params->timelimit));
   
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
   // parse args
   std::vector<std::string> args = args_from_sin(sin);
   if (args.size() != 1)
      throw OpenRAVE::openrave_exception("UseSubsetManager args not correct!");
   std::string name = args[0];
   
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

// e.g. SetRoadmap class=RoadmapSampledConst seed=419884521 batch_n=1000 radius=2
bool or_multiset::MultiSetPRM::SetRoadmap(std::ostream & sout, std::istream & sin)
{
   std::istreambuf_iterator<char> eos;
   this->roadmap_string =  std::string(std::istreambuf_iterator<char>(sin), eos);
   
   // clear planner??
   
   return true;
}

bool or_multiset::MultiSetPRM::CacheSetLocation(std::ostream & sout, std::istream & sin)
{
   // parse args
   std::vector<std::string> args = args_from_sin(sin);
   if (args.size() != 1)
      throw OpenRAVE::openrave_exception("CacheSetLocation args not correct!");
   std::string location = args[0];
   this->cache.reset(ompl_multiset::cache_create(location));
   return true;
}

bool or_multiset::MultiSetPRM::CacheLoad(std::ostream & sout, std::istream & sin)
{
   // parse args
   std::vector<std::string> args = args_from_sin(sin);
   if (args.size() != 1)
      throw OpenRAVE::openrave_exception("CacheLoad args not correct!");
   int use_num_subgraphs = atoi(args[0].c_str());
   // check internal state
   if (!this->ompl_planner)
      throw OpenRAVE::openrave_exception("CacheLoad called with no planner!");
   if (!this->cache)
      throw OpenRAVE::openrave_exception("CacheLoad called with no cache!");
   // do it!
   this->ompl_planner->use_num_subgraphs(use_num_subgraphs);
   this->ompl_planner->cache_load(this->cache);
   return true;
}

bool or_multiset::MultiSetPRM::CacheSave(std::ostream & sout, std::istream & sin)
{
   if (!this->ompl_planner)
      throw OpenRAVE::openrave_exception("CacheSave called with no planner!");
   if (!this->cache)
      throw OpenRAVE::openrave_exception("CacheSave called with no cache!");
   this->ompl_planner->cache_save(this->cache);
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
   RAVELOG_INFO("using space resolution of %f\n", resolution);
   this->ompl_space->setLongestValidSegmentFraction(
      resolution / this->ompl_space->getMaximumExtent());

   // create the roadmap object
   ompl_multiset::RoadmapPtr roadmap;
   // class=RoadmapSampledConst seed=419884521 batch_n=1000 radius=2
   {
      // parse roadmap args
      std::stringstream ss(this->roadmap_string);
      std::vector<std::string> args = args_from_sin(ss);
      if (args.size() < 1)
         throw OpenRAVE::openrave_exception("SetRoadmap args not correct, not at least one arg!");
      if (args[0] == "class=RoadmapSampledConst")
      {
         if (args.size() != 4)
            throw OpenRAVE::openrave_exception("SetRoadmap args not correct, not exactly four args!");
         if (args[1].substr(0,5) != "seed="
            || args[2].substr(0,8) != "batch_n="
            || args[3].substr(0,7) != "radius=")
            throw OpenRAVE::openrave_exception("SetRoadmap args not correct!");
         unsigned int seed = atoi(args[1].c_str()+5);
         unsigned int batch_n = atoi(args[2].c_str()+8);
         double radius = atof(args[3].c_str()+7);
         roadmap.reset(new ompl_multiset::RoadmapSampledConst(this->ompl_space, seed, batch_n, radius));
      }
      else
         throw OpenRAVE::openrave_exception("SetRoadmap args not correct, not a known class");
   }
   
   // create planner itself
   this->ompl_planner.reset(ompl_multiset::MultiSetPRM::create(
      this->ompl_space, roadmap));
   this->subsets.clear();
   this->intersections.clear();
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
