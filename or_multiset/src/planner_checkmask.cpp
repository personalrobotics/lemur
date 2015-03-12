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

#include "params_checkmask.h"
#include "planner_checkmask.h"

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
   OpenRAVE::PlannerBase(penv), p(0)
{
   __description = "OmplCheckMask description";
   this->RegisterCommand("SetOMPLSeed",boost::bind(&or_multiset::MultiSetPRM::SetOMPLSeed,this,_1,_2),"SetOMPLSeed");
   this->RegisterCommand("ListSpaces",boost::bind(&or_multiset::MultiSetPRM::ListSpaces,this,_1,_2),"ListSpaces");
   this->RegisterCommand("GetTimes",boost::bind(&or_multiset::MultiSetPRM::GetTimes,this,_1,_2),"GetTimes");
   printf("constructed!\n");
}

or_multiset::MultiSetPRM::~MultiSetPRM()
{
   if (this->p) delete this->p;
   printf("destructed!\n");
}

bool or_multiset::MultiSetPRM::InitPlan(OpenRAVE::RobotBasePtr robot, std::istream & sin)
{
   or_multiset::PlannerParametersPtr my_params(new or_multiset::PlannerParameters());
   sin >> *my_params;
   RAVELOG_WARN("skipping custom PlannerParameters validation due to exception!\n");
   //my_params->Validate();
   return this->InitPlan(robot, my_params);
}

bool or_multiset::MultiSetPRM::InitPlan(OpenRAVE::RobotBasePtr robot, OpenRAVE::PlannerBase::PlannerParametersConstPtr params)
{
   // is this one of ours?
   or_multiset::PlannerParametersConstPtr my_params = boost::dynamic_pointer_cast<or_multiset::PlannerParameters const>(params);
   if (!my_params)
   {
      // if not, serialize and call our serialized constructor
      RAVELOG_WARN("Warning, OmplCheckMask planner passed an unknown PlannerParameters type! Attempting to serialize ...\n");
      std::stringstream ss;
      ss << *params;
      return this->InitPlan(robot, ss);
   }

   //RAVELOG_WARN("OmplCheckMask planner does not support default PlannerParameters structure!\n");
   //throw OpenRAVE::openrave_exception(
   //   "OmplCheckMask planner does not support default PlannerParameters structure!");
   
   if (!this->robot)
   {
      this->robot = robot;
      this->adofs = robot->GetActiveDOFIndices();
      /* space: right arm dofs */
      this->ompl_space.reset(new ompl::base::RealVectorStateSpace(robot->GetActiveDOF()));
      {
         /* state space set bounds */
         ompl::base::RealVectorBounds bounds(robot->GetActiveDOF());
         std::vector<OpenRAVE::dReal> lowers;
         std::vector<OpenRAVE::dReal> uppers;
         robot->GetActiveDOFLimits(lowers, uppers);
         for (int i=0; i<robot->GetActiveDOF(); i++)
            { bounds.setLow(i, lowers[i]); bounds.setHigh(i, uppers[i]); }
         this->ompl_space->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);
         
         /* set space resolution */
         std::vector<OpenRAVE::dReal> dof_resolutions;
         robot->GetActiveDOFResolutions(dof_resolutions);
         double resolution = HUGE_VAL;
         for (unsigned int i=0; i<dof_resolutions.size(); i++)
            resolution = dof_resolutions[i] < resolution ? dof_resolutions[i] : resolution;
         RAVELOG_INFO("setting resolution to %f rad!\n", resolution);
         this->ompl_space->setLongestValidSegmentFraction(
            resolution / this->ompl_space->getMaximumExtent());
      }

      // create planner itself
      ompl_multiset::RoadmapPtr roadmap(
         new ompl_multiset::RoadmapSampledConst(this->ompl_space, 419884521, 1000, 2.0));
      this->p = ompl_multiset::MultiSetPRM::create(this->ompl_space,
         roadmap, ompl_multiset::CachePtr());
      this->p->set_interroot_radius(2.0);
   }
   
   if (robot != this->robot || robot->GetActiveDOFIndices() != this->adofs)
      throw OpenRAVE::openrave_exception("robot or active dofs not constant!");
   
   // get the current space from the environment
   Space s = this->get_current_space();
   if (!s.ilcs.size())
      throw OpenRAVE::openrave_exception("space requires no checks! not currently supported.");
   
   // insert the space into our spaces set and the underlying planner itself
   try
   {
      this->sidx_current = this->insert_space(s);
   }
   catch (const ompl::Exception & ex)
   {
      printf("exception: |%s|\n", ex.what());
      throw OpenRAVE::openrave_exception("caught ompl exception!");
   }
   printf("InitPlan sidx_current: %u\n", this->sidx_current);
   
   // create a new problem definition
   ompl::base::ProblemDefinitionPtr pdef(new ompl::base::ProblemDefinition(
      this->spaces[this->sidx_current].ompl_si));
   
   // add start states
   pdef->clearStartStates();
   for (unsigned int si=0; si<my_params->startstates.size(); si++)
   {
      if (my_params->startstates[si].size() != this->ompl_space->getDimension())
         throw OpenRAVE::openrave_exception("start state not the right dimension!");
      printf("found a start state:");
      ompl::base::ScopedState<ompl::base::RealVectorStateSpace> s_start(this->ompl_space);
      for (unsigned int j=0; j<my_params->startstates[si].size(); j++)
      {
         printf(" %f", my_params->startstates[si][j]);
         s_start->values[j] = my_params->startstates[si][j];
      }
      printf("\n");
      pdef->addStartState(s_start);
   }
   
   // add goal states
   ompl::base::GoalStates * gs = new ompl::base::GoalStates(pdef->getSpaceInformation());
   gs->clear();
   for (unsigned int si=0; si<my_params->goalstates.size(); si++)
   {
      if (my_params->goalstates[si].size() != this->ompl_space->getDimension())
         throw OpenRAVE::openrave_exception("goal state not the right dimension!");
      printf("found a goal state:");
      ompl::base::ScopedState<ompl::base::RealVectorStateSpace> s_goal(this->ompl_space);
      for (unsigned int j=0; j<my_params->goalstates[si].size(); j++)
      {
         printf(" %f", my_params->goalstates[si][j]);
         s_goal->values[j] = my_params->goalstates[si][j];
      }
      printf("\n");
      gs->addState(s_goal);
   }
   pdef->clearGoal();
   pdef->setGoal(ompl::base::GoalPtr(gs));
   
   // tell the planner about our problem definition
   this->p->setProblemDefinition(pdef);
   
   return true;
}

OpenRAVE::PlannerStatus or_multiset::MultiSetPRM::PlanPath(OpenRAVE::TrajectoryBasePtr ptraj)
{
   ompl::base::PlannerStatus status;
   
   // call planner
   this->n_checks = 0;
   this->checktime = 0;
   this->totaltime = 0;
   
   struct timespec tic;
   struct timespec toc;
   clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &tic);
   
   {
      // Don't check collision with inactive links.
      OpenRAVE::CollisionCheckerBasePtr const collision_checker = GetEnv()->GetCollisionChecker();
      OpenRAVE::CollisionOptionsStateSaver const collision_saver(collision_checker, OpenRAVE::CO_ActiveDOFs, false);

      status = this->p->solve(ompl::base::timedPlannerTerminationCondition(600.0));
   }
   
   clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &toc);
   this->totaltime = (toc.tv_nsec - tic.tv_nsec) + 1000000000*(toc.tv_sec - tic.tv_sec);
   
   // check results
   printf("planner returned: %s\n", status.asString().c_str());
   if (status != ompl::base::PlannerStatus::EXACT_SOLUTION) return OpenRAVE::PS_Failed;
   
   // convert result
   ompl::base::PathPtr path = this->p->getProblemDefinition()->getSolutionPath();
   ompl::geometric::PathGeometric * gpath = dynamic_cast<ompl::geometric::PathGeometric*>(path.get());
   if (!gpath)
      throw OpenRAVE::openrave_exception("ompl path is not geometric for some reason!");
      
   ptraj->Init(this->robot->GetActiveConfigurationSpecification());
   for (unsigned int i=0; i<gpath->getStateCount(); i++)
   {
      ompl::base::ScopedState<ompl::base::RealVectorStateSpace> s(this->ompl_space);
      s = gpath->getState(i);
      std::vector<OpenRAVE::dReal> vec(&s[0], &s[0]+this->robot->GetActiveDOF());
      ptraj->Insert(i, vec);
   }
   
   return OpenRAVE::PS_HasSolution;
}

OpenRAVE::PlannerBase::PlannerParametersConstPtr or_multiset::MultiSetPRM::GetParameters() const
{
   return OpenRAVE::PlannerBase::PlannerParametersConstPtr();
}

bool or_multiset::MultiSetPRM::SetOMPLSeed(std::ostream & sout, std::istream & sin)
{
   unsigned int seed;
   sin >> seed;
   printf("setting seed to %u!\n", seed);
   ompl::RNG::setSeed(seed);
   return true;
}

bool or_multiset::MultiSetPRM::ListSpaces(std::ostream & sout, std::istream & sin)
{
   printf("ListSpaces called!\n");
   
   for (unsigned int sidx=0; sidx<this->spaces.size(); sidx++)
   {
      if (this->base_spaces.find(sidx)!=this->base_spaces.end())
      {
         printf("space [%u] (base)\n", sidx);
         // base space
         for (std::set<unsigned int>::iterator ilc=this->spaces[sidx].ilcs.begin(); ilc!=this->spaces[sidx].ilcs.end(); ilc++)
         {
            printf("  ilc [%u] between '%s:%s' and '%s:%s'\n", *ilc,
               this->inter_link_checks[*ilc].link1->GetParent()->GetName().c_str(),
               this->inter_link_checks[*ilc].link1->GetName().c_str(),
               this->inter_link_checks[*ilc].link2->GetParent()->GetName().c_str(),
               this->inter_link_checks[*ilc].link2->GetName().c_str());
         }
      }
      else
      {
         // non-base space
         printf("space [%u] (intersection)\n", sidx);
      }
   }
   
   for (unsigned int isecidx=0; isecidx<this->intersections.size(); isecidx++)
   {
      printf("intersection [%u]: [%u] is the intersection of [%u] and [%u]!\n",
         isecidx,
         this->intersections[isecidx].intersection,
         this->intersections[isecidx].a,
         this->intersections[isecidx].b);
   }
   
   return true;
}

bool or_multiset::MultiSetPRM::GetTimes(std::ostream & sout, std::istream & sin)
{
   sout << "checktime " << (1.0e-9 * this->checktime);
   sout << " totaltime " << (1.0e-9 * this->totaltime);
   sout << " n_checks " << this->n_checks;
   return true;
}

or_multiset::Space or_multiset::MultiSetPRM::get_current_space(void)
{
   bool success;
   
   // get the list of all robots
   std::vector<OpenRAVE::RobotBasePtr> robots;
   this->robot->GetEnv()->GetRobots(robots);
   
   // get non-adjacent links
   std::set<int> non_adjacent_links = this->robot->GetNonAdjacentLinks(0);
   
   // get set of active joints (joints which contain active dofs)
   std::set<OpenRAVE::KinBody::JointPtr> ajoints;
   {
      for (std::vector<int>::const_iterator adof=this->adofs.begin(); adof!=this->adofs.end(); adof++)
         ajoints.insert(this->robot->GetJointFromDOFIndex(*adof));
      for (std::set<OpenRAVE::KinBody::JointPtr>::iterator ajoint=ajoints.begin(); ajoint!=ajoints.end(); ajoint++)
      {
         for (int i=0; i<(*ajoint)->GetDOF(); i++)
         {
            int dof = (*ajoint)->GetDOFIndex() + i;
            if (std::find(this->adofs.begin(), this->adofs.end(), dof) != this->adofs.end())
               continue;
            RAVELOG_WARN("Active joint %s includes non-active DOF %d!\n", (*ajoint)->GetName().c_str(), dof);
         }
      }
   }
   
   // get a list of all enabled links in the environment
   std::vector<OpenRAVE::KinBody::LinkPtr> links;
   {
      std::vector<OpenRAVE::KinBodyPtr> ks;
      this->robot->GetEnv()->GetBodies(ks);
      for (std::vector<OpenRAVE::KinBodyPtr>::iterator k=ks.begin(); k!=ks.end(); k++)
      {
         const std::vector<OpenRAVE::KinBody::LinkPtr> klinks = (*k)->GetLinks();
         for (std::vector<OpenRAVE::KinBody::LinkPtr>::const_iterator klink=klinks.begin(); klink!=klinks.end(); klink++)
         if ((*klink)->IsEnabled())
            links.push_back(*klink);
      }
   }
   
   // for each link, get its path to the environment root
   std::map<OpenRAVE::KinBody::LinkPtr, std::vector<TxAjoint> > link_paths;
   {
      for (std::vector<OpenRAVE::KinBody::LinkPtr>::iterator link_orig=links.begin(); link_orig!=links.end(); link_orig++)
      {
         std::vector<TxAjoint> link_path; // we'll fill this
         
         // start pointing to the original target link, with no sucessor joint
         // these will eventually be prepended to link_path
         OpenRAVE::KinBody::LinkPtr link_target = *link_orig;
         OpenRAVE::KinBody::JointPtr joint_target; // null
         
         // iteravely go backwards up the link chain to the environment root
         // on the way, we'll look for active joints
         for (OpenRAVE::KinBody::LinkPtr link = link_target; link; )
         {
#if 0
            printf("considering link %s:%s ...\n",
               link->GetParent()->GetName().c_str(),
               link->GetName().c_str());
#endif
            
            // find parent link
            OpenRAVE::KinBody::LinkPtr link_parent;
            
            // do we have a parent in our kinbody?
            std::vector<OpenRAVE::KinBody::JointPtr> parent_joints;
            success = link->GetParent()->GetChain(0, link->GetIndex(), parent_joints);
            if (!success)
               throw OPENRAVE_EXCEPTION_FORMAT("oops, GetChain failed to root link for link %s:%s!",
                  link->GetParent()->GetName().c_str() %
                  link->GetName().c_str(),
                  OpenRAVE::ORE_Failed);
            if (parent_joints.size())
            {
               OpenRAVE::KinBody::JointPtr parent_joint = *parent_joints.rbegin();
#if 0
               printf("  found parent joint %s in kinbody!\n", parent_joint->GetName().c_str());
#endif
               if (parent_joint->GetSecondAttached() != link)
                  throw OPENRAVE_EXCEPTION_FORMAT("oops, link %s:%s parent joint's second attached is not self!",
                     link->GetParent()->GetName().c_str() %
                     link->GetName().c_str(),
                     OpenRAVE::ORE_Failed);
               if (ajoints.find(parent_joint) != ajoints.end()) // parent joint is an active joint!
               {
                  // create new TxAjoint for things past this joint and add it to our link_path
                  TxAjoint txajoint;
                  txajoint.tx = link->GetTransform().inverse() * link_target->GetTransform();
                  txajoint.ajoint = joint_target;
                  link_path.insert(link_path.begin(), txajoint);
                  // start working on the next one
                  link_target = parent_joint->GetFirstAttached();
                  joint_target = parent_joint;
               }
               // go to previous link
               link = parent_joint->GetFirstAttached();
            }
            else // we're the root link!
            {
               // are we grabbed by a robot link?
               std::vector<OpenRAVE::KinBody::LinkPtr> links_grabbing;
               for (std::vector<OpenRAVE::RobotBasePtr>::iterator robot=robots.begin(); robot!=robots.end(); robot++)
               {
                  OpenRAVE::KinBody::LinkPtr link_grabbing = (*robot)->IsGrabbing(link->GetParent());
                  if (!link_grabbing)
                     continue;
                  links_grabbing.push_back(link_grabbing);
               }
               switch (links_grabbing.size())
               {
               case 0: // not grabbed
                  // we're done! insert last TxAjoint ...
                  {
                     TxAjoint txajoint;
                     txajoint.tx = link_target->GetTransform();
                     txajoint.ajoint = joint_target;
                     link_path.insert(link_path.begin(), txajoint);
                  }
                  link.reset(); // done
                  break;
               case 1: // grabbed
                  link = links_grabbing[0];
                  break;
               default:
                  throw OPENRAVE_EXCEPTION_FORMAT("oops, link %s:%s grabbed by more than one robot!",
                     link->GetParent()->GetName().c_str() %
                     link->GetName().c_str(),
                     OpenRAVE::ORE_Failed);
               }
            }
         }
#if 0
         printf("path to %s:%s ...\n",
            (*link_orig)->GetParent()->GetName().c_str(),
            (*link_orig)->GetName().c_str());
         for (std::vector<TxAjoint>::iterator txajoint=link_path.begin(); txajoint!=link_path.end(); txajoint++)
         {
            printf("  tx: %f %f %f %f %f %f %f, joint: %s\n",
               txajoint->tx.trans.x,
               txajoint->tx.trans.y,
               txajoint->tx.trans.z,
               txajoint->tx.rot.y,
               txajoint->tx.rot.z,
               txajoint->tx.rot.w,
               txajoint->tx.rot.x,
               txajoint->ajoint?txajoint->ajoint->GetName().c_str():"(none)");
         }
#endif
         link_paths[(*link_orig)] = link_path;
      }
   }
   
   // next, for each PAIR of links, create the InterLinkCheck structure for this space
   std::vector<InterLinkCheck> s_ilcs;
   for (std::vector<OpenRAVE::KinBody::LinkPtr>::iterator link1=links.begin(); link1!=links.end(); link1++)
   for (std::vector<OpenRAVE::KinBody::LinkPtr>::iterator link2=links.begin(); link2!=links.end(); link2++)
   {
      // ensure link1 < link2
      if (!((*link1) < (*link2)))
         continue;
      
      // if they're both robot links, ensure they're nonadjacent!
      if ((*link1)->GetParent() == this->robot && (*link2)->GetParent() == this->robot)
      {
         int idx1 = (*link1)->GetIndex();
         int idx2 = (*link2)->GetIndex();
         int set_key;
         if (idx1 < idx2)
            set_key = idx1|(idx2<<16);
         else
            set_key = idx2|(idx1<<16);
         if (non_adjacent_links.find(set_key) == non_adjacent_links.end())
         {
#if 0
            printf("skipping adjacent links %s and %s!\n",
               (*link1)->GetName().c_str(),
               (*link2)->GetName().c_str());
#endif
            continue;
         }
      }
      
      InterLinkCheck ilc;
      ilc.link1 = *link1;
      ilc.link2 = *link2;
      ilc.link1_path = link_paths[ilc.link1];
      ilc.link2_path = link_paths[ilc.link2];
      
      // remove common path prefix
      // (dont make them empty though -- leave the last path element with no joint)
      while (ilc.link1_path.size() > 1 && ilc.link2_path.size() > 1)
      {
         if (!(ilc.link1_path[0] == ilc.link2_path[0]))
            break;
         ilc.link1_path.erase(ilc.link1_path.begin());
         ilc.link2_path.erase(ilc.link2_path.begin());
      }
      
      // are there any active joints between these links?
      if (ilc.link1_path.size()-1 + ilc.link2_path.size()-1 == 0)
      {
#if 0
         printf("skipping non-active link pair %s and %s!\n",
            (*link1)->GetName().c_str(),
            (*link2)->GetName().c_str());
#endif
         continue;
      }
      
      // make first link1 tx identity
      ilc.link2_path[0].tx = ilc.link1_path[0].tx.inverse() * ilc.link2_path[0].tx;
      ilc.link1_path[0].tx.identity();
      
#if 0
      if (ilc.link1->GetParent()->GetName() == "plasticmug")
      {
         printf("path to %s:%s ...\n",
            ilc.link1->GetParent()->GetName().c_str(),
            ilc.link1->GetName().c_str());
         for (std::vector<TxAjoint>::iterator txajoint=ilc.link1_path.begin(); txajoint!=ilc.link1_path.end(); txajoint++)
         {
            printf("  tx: %f %f %f %f %f %f %f, joint: %s\n",
               txajoint->tx.trans.x,
               txajoint->tx.trans.y,
               txajoint->tx.trans.z,
               txajoint->tx.rot.y,
               txajoint->tx.rot.z,
               txajoint->tx.rot.w,
               txajoint->tx.rot.x,
               txajoint->ajoint?txajoint->ajoint->GetName().c_str():"(none)");
         }
      }
#endif
      
      // insert!
      s_ilcs.push_back(ilc);
   }
   
   // create a space which links to each required ilc
   Space s;
   for (std::vector<InterLinkCheck>::iterator it=s_ilcs.begin(); it!=s_ilcs.end(); it++)
   {
      // is it already in inter_link_checks?
      unsigned int ilci;
      for (ilci=0; ilci<this->inter_link_checks.size(); ilci++)
         if (*it == this->inter_link_checks[ilci])
            break;
      if (ilci == this->inter_link_checks.size())
      {
#if 0
         printf("new inter-link collision between %s:%s and %s:%s\n",
            it->link1->GetParent()->GetName().c_str(), it->link1->GetName().c_str(),
            it->link2->GetParent()->GetName().c_str(), it->link2->GetName().c_str());
#endif
         this->inter_link_checks.push_back(*it);
      }
      // add to my space
      s.ilcs.insert(ilci);
   }
#if 0
   printf("target space composed of checks:");
   for (std::set<unsigned int>::iterator it=s.ilcs.begin(); it!=s.ilcs.end(); it++)
      printf(" %u", *it);
   printf("\n");
#endif
   
   return s;
}

unsigned int or_multiset::MultiSetPRM::insert_space(Space s)
{
   // does this space already exist?
   unsigned int space_idx;
   for (space_idx=0; space_idx<this->spaces.size(); space_idx++)
      if (s == this->spaces[space_idx])
         break;
   //printf("space_idx: %u\n", space_idx);
   if (space_idx == this->spaces.size())
   {
      // does this space split any of the existing base spaces?
      std::set<unsigned int> bspaces_to_split;
      for (std::set<unsigned int>::iterator bspace=this->base_spaces.begin(); bspace!=this->base_spaces.end(); bspace++)
      {
         Space bs = this->spaces[*bspace];
         bool bspace_check_in_space = false;
         bool bspace_check_notin_space = false;
         
         for (std::set<unsigned int>::iterator bspace_ilc=bs.ilcs.begin(); bspace_ilc!=bs.ilcs.end(); bspace_ilc++)
         {
            if (s.ilcs.find(*bspace_ilc) != s.ilcs.end())
               bspace_check_in_space = true;
            else
               bspace_check_notin_space = true;
         }
         
         //printf("bspace_check_in_space: %s\n", bspace_check_in_space?"true":"false");
         //printf("bspace_check_notin_space: %s\n", bspace_check_notin_space?"true":"false");
         
         if (!bspace_check_in_space && !bspace_check_notin_space)
            throw OpenRAVE::openrave_exception("how did a space requiring no checks make it in?");
         if ( bspace_check_in_space && !bspace_check_notin_space) continue;
         if (!bspace_check_in_space &&  bspace_check_notin_space) continue;
         
         //printf("detected a base space split!\n");
         
         bspaces_to_split.insert(*bspace);
      }
      
      // eventually we split and then add relations
      for (std::set<unsigned int>::iterator bspace=bspaces_to_split.begin(); bspace!=bspaces_to_split.end(); bspace++)
      {
         Space bs = this->spaces[*bspace];
         Space bs_true;
         Space bs_false;
         for (std::set<unsigned int>::iterator bspace_ilc=bs.ilcs.begin(); bspace_ilc!=bs.ilcs.end(); bspace_ilc++)
         {
            if (s.ilcs.find(*bspace_ilc) != s.ilcs.end())
               bs_true.ilcs.insert(*bspace_ilc);
            else
               bs_false.ilcs.insert(*bspace_ilc);
         }
         
         // add new intersection and spaces!
         Intersection isec;
         
         this->base_spaces.erase(*bspace);
         isec.intersection = *bspace;
         
         isec.a = this->spaces.size();
         this->base_spaces.insert(this->spaces.size());
         bs_true.ompl_si.reset(new ompl::base::SpaceInformation(this->ompl_space));
         bs_true.ompl_si->setStateValidityChecker(
            boost::bind(&or_multiset::MultiSetPRM::ompl_isvalid, this, this->spaces.size(), _1));
         this->p->add_cfree(bs_true.ompl_si, sf("space-%u",this->spaces.size()), bs_true.ilcs.size()+1);
         this->spaces.push_back(bs_true);
         
         isec.b = this->spaces.size();
         this->base_spaces.insert(this->spaces.size());
         bs_false.ompl_si.reset(new ompl::base::SpaceInformation(this->ompl_space));
         bs_false.ompl_si->setStateValidityChecker(
            boost::bind(&or_multiset::MultiSetPRM::ompl_isvalid, this, this->spaces.size(), _1));
         this->p->add_cfree(bs_false.ompl_si, sf("space-%u",this->spaces.size()), bs_false.ilcs.size()+1);
         this->spaces.push_back(bs_false);
         
         this->intersections.push_back(isec);
         
         // tell planner about new intersection
         p->add_intersection(
            this->spaces[isec.a].ompl_si,
            this->spaces[isec.b].ompl_si,
            this->spaces[isec.intersection].ompl_si);
      }
      
      // figure out which bspaces we are the intersection of
      std::set<unsigned int> bspaces_intersection;
      for (std::set<unsigned int>::iterator bspace=this->base_spaces.begin(); bspace!=this->base_spaces.end(); bspace++)
      {
         Space bs = this->spaces[*bspace];
         // test against this base space
         std::set<unsigned int>::iterator bspace_ilc;
         for (bspace_ilc=bs.ilcs.begin(); bspace_ilc!=bs.ilcs.end(); bspace_ilc++)
            if (s.ilcs.find(*bspace_ilc) != s.ilcs.end())
               break;
         if (bspace_ilc!=bs.ilcs.end())
         {
            bspaces_intersection.insert(*bspace);
            //printf("we are at the intersection of %u!\n", *bspace);
         }
      }
      
      if (bspaces_intersection.size())
      {
         // add a base space for any checks not yet covered
         Space s_new = s;
         for (std::set<unsigned int>::iterator bspace=bspaces_intersection.begin(); bspace!=bspaces_intersection.end(); bspace++)
         {
            Space bs = this->spaces[*bspace];
            for (std::set<unsigned int>::iterator bspace_ilc=bs.ilcs.begin(); bspace_ilc!=bs.ilcs.end(); bspace_ilc++)
               s_new.ilcs.erase(*bspace_ilc);
         }
         if (s_new.ilcs.size())
         {
            //printf("added s_new [%lu]!\n", this->spaces.size());
            bspaces_intersection.insert(this->spaces.size());
            this->base_spaces.insert(this->spaces.size());
            s_new.ompl_si.reset(new ompl::base::SpaceInformation(this->ompl_space));
            s_new.ompl_si->setStateValidityChecker(
               boost::bind(&or_multiset::MultiSetPRM::ompl_isvalid, this, this->spaces.size(), _1));
            this->p->add_cfree(s_new.ompl_si, sf("space-%u",this->spaces.size()), s_new.ilcs.size()+1);
            this->spaces.push_back(s_new);
         }
         
         // add new intersection and spaces!
         if (bspaces_intersection.size() != 2)
         {
            printf("bspaces_intersection.size(): %lu\n", bspaces_intersection.size());
            throw OpenRAVE::openrave_exception("oops, need to add an intersection of not exactly two spaces!");
         }
         
         Intersection isec;
         std::set<unsigned int>::iterator bspace_intersection = bspaces_intersection.begin();
         isec.a = *bspace_intersection;
         bspace_intersection++;
         isec.b = *bspace_intersection;
         isec.intersection = this->spaces.size();
         this->intersections.push_back(isec);
         
         // we are a new space, in terms of some other base spaces!
         //printf("space %lu is inserted, at the intersection of:\n", this->spaces.size());
         //for (std::set<unsigned int>::iterator bspace=bspaces_intersection.begin(); bspace!=bspaces_intersection.end(); bspace++)
         //   printf("  [%u]\n", *bspace);
         s.ompl_si.reset(new ompl::base::SpaceInformation(this->ompl_space));
         s.ompl_si->setStateValidityChecker(
            boost::bind(&or_multiset::MultiSetPRM::ompl_isvalid, this, this->spaces.size(), _1));
         this->p->add_cfree(s.ompl_si, sf("space-%u",this->spaces.size()), s.ilcs.size()+1);
         space_idx = this->spaces.size();
         this->spaces.push_back(s);
         
         // tell planner about new intersection
         p->add_intersection(
            this->spaces[isec.a].ompl_si,
            this->spaces[isec.b].ompl_si,
            this->spaces[isec.intersection].ompl_si);
      }
      else
      {
         // its just us, no relations!
         // insert as a new separate base space
         //printf("inserting as new base space, no relations!\n");
         this->base_spaces.insert(this->spaces.size());
         s.ompl_si.reset(new ompl::base::SpaceInformation(this->ompl_space));
         s.ompl_si->setStateValidityChecker(
            boost::bind(&or_multiset::MultiSetPRM::ompl_isvalid, this, this->spaces.size(), _1));
         this->p->add_cfree(s.ompl_si, sf("space-%u",this->spaces.size()), s.ilcs.size());
         space_idx = this->spaces.size();
         this->spaces.push_back(s);
      }
   }
   
   return space_idx;
}

bool or_multiset::MultiSetPRM::ompl_isvalid(unsigned int sidx, const ompl::base::State * state)
{
   struct timespec tic;
   struct timespec toc;
   bool isvalid;
   clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &tic);
   
#if 0
   if (sidx != this->sidx_current)
   {
      // does sidx_current IMPLY sidx?
      unsigned int ii;
      for (ii=0; ii<this->intersections.size(); ii++)
      {
         if (this->intersections[ii].intersection != this->sidx_current)
            continue;
         if (this->intersections[ii].a == sidx)
            break;
         if (this->intersections[ii].b == sidx)
            break;
      }
      if (!(ii<this->intersections.size()))
      {
         printf("current space: %u\n", this->sidx_current);
         printf("requested space: %u\n", sidx);
         throw OpenRAVE::openrave_exception("cannot currently check against a non-current space! (maybe logic not complete enough yet?)");
      }
   }
#endif

   // set active dofs
   double * q = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
   std::vector<double> adofvals(q,q+this->ompl_space->getDimension());
   this->robot->SetActiveDOFValues(adofvals, OpenRAVE::KinBody::CLA_Nothing);
   
#if 0
   // do checks from space
   Space & s = this->spaces[sidx];
   isvalid = true;
   for (std::set<unsigned int>::iterator iilc=s.ilcs.begin(); isvalid && iilc!=s.ilcs.end(); iilc++)
   {
      InterLinkCheck & ilc = this->inter_link_checks[*iilc];
      isvalid = !(this->robot->GetEnv()->CheckCollision(ilc.link1, ilc.link2));
      if (!isvalid)
         break;
   }
#else
   isvalid = !GetEnv()->CheckCollision(this->robot) && !this->robot->CheckSelfCollision();
#endif

   clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &toc);
   this->checktime += (toc.tv_nsec - tic.tv_nsec) + 1000000000*(toc.tv_sec - tic.tv_sec);
   this->n_checks++;
   return isvalid;
}

bool or_multiset::fuzzy_equals(const OpenRAVE::Transform & tx1, const OpenRAVE::Transform & tx2, OpenRAVE::dReal fuzz)
{
   if (fabs(tx1.trans.x - tx2.trans.x) > fuzz) return false;
   if (fabs(tx1.trans.y - tx2.trans.y) > fuzz) return false;
   if (fabs(tx1.trans.z - tx2.trans.z) > fuzz) return false;
   if (  (fabs(tx1.rot.y - tx2.rot.y) < fuzz)
      && (fabs(tx1.rot.z - tx2.rot.z) < fuzz)
      && (fabs(tx1.rot.w - tx2.rot.w) < fuzz)
      && (fabs(tx1.rot.x - tx2.rot.x) < fuzz))
      return true;
   if (  (fabs(tx1.rot.y + tx2.rot.y) < fuzz)
      && (fabs(tx1.rot.z + tx2.rot.z) < fuzz)
      && (fabs(tx1.rot.w + tx2.rot.w) < fuzz)
      && (fabs(tx1.rot.x + tx2.rot.x) < fuzz))
      return true;
   return false;
}
