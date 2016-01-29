/* File: module_subset_manager.cpp
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

#include <cstdarg>
#include <openrave/openrave.h>
#include <or_lemur/inter_link_checks.h>
#include <or_lemur/module_subset_manager.h>

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

or_lemur::ModuleSubsetManager::ModuleSubsetManager(OpenRAVE::EnvironmentBasePtr penv):
   OpenRAVE::ModuleBase(penv), penv(penv), cost_per_ilc(60.0e-6)
{
   __description = "ModuleSubsetManager description";
   this->RegisterCommand("GetName",
      boost::bind(&or_lemur::ModuleSubsetManager::GetName,this,_1,_2),
      "GetName");
   this->RegisterCommand("SetCostPerIlc",
      boost::bind(&or_lemur::ModuleSubsetManager::SetCostPerIlc,this,_1,_2),
      "SetCostPerIlc");
   this->RegisterCommand("OverrideSubsetCost",
      boost::bind(&or_lemur::ModuleSubsetManager::OverrideSubsetCost,this,_1,_2),
      "OverrideSubsetCost");
   this->RegisterCommand("TagCurrentSubset",
      boost::bind(&or_lemur::ModuleSubsetManager::TagCurrentSubset,this,_1,_2),
      "TagCurrentSubset");
   this->RegisterCommand("GetCurrentReport",
      boost::bind(&or_lemur::ModuleSubsetManager::GetCurrentReport,this,_1,_2),
      "GetCurrentReport");
   this->RegisterCommand("DumpSubsets",
      boost::bind(&or_lemur::ModuleSubsetManager::DumpSubsets,this,_1,_2),
      "DumpSubsets");
}

or_lemur::ModuleSubsetManager::~ModuleSubsetManager()
{
   printf("SubsetManager module destructed!\n");
}

// on environment add
int or_lemur::ModuleSubsetManager::main(const std::string & cmd)
{
   if (!cmd.size())
   {
      RAVELOG_ERROR("You must provide a unique non-empty name for each subset manager!\n");
      return 1;
   }
   
   // check if any other subset manager in the env has this name
   std::list<OpenRAVE::ModuleBasePtr> penv_modules;
   std::list<OpenRAVE::ModuleBasePtr>::iterator it;
   this->penv->GetModules(penv_modules);
   for (it=penv_modules.begin(); it!=penv_modules.end(); it++)
   {
      boost::shared_ptr<or_lemur::ModuleSubsetManager> mod
         = boost::dynamic_pointer_cast<or_lemur::ModuleSubsetManager>(*it);
      if (!mod)
         continue;
      if (mod->name == cmd)
         break;
   }
   if (it!=penv_modules.end())
   {
      RAVELOG_ERROR("Environment already has a subset manager named \"%s\"!\n", cmd.c_str());
      return 1;
   }
   
   this->name = cmd;
   return 0;
}

void or_lemur::ModuleSubsetManager::Destroy()
{
   printf("SubsetManager module removed from environment!\n");
}

bool or_lemur::ModuleSubsetManager::GetName(std::ostream & sout, std::istream & sin)
{
   sout << this->name;
   return true;
}

bool or_lemur::ModuleSubsetManager::SetCostPerIlc(std::ostream & sout, std::istream & sin)
{
   std::vector<std::string> args = args_from_sin(sin);
   if (args.size() != 1)
      throw OpenRAVE::openrave_exception("SetCostPerIlc args not correct!");
   this->cost_per_ilc = atof(args[0].c_str());
   return true;
}

bool or_lemur::ModuleSubsetManager::OverrideSubsetCost(std::ostream & sout, std::istream & sin)
{
   std::vector<std::string> args = args_from_sin(sin);
   if (args.size() != 3)
      throw OpenRAVE::openrave_exception("OverrideSubsetCost args not correct!");
   std::string robot_name = args[0];
   std::string subset_name = args[1];
   double new_cost = atof(args[2].c_str());
   OpenRAVE::RobotBasePtr robot = this->penv->GetRobot(robot_name);
   if (!robot)
      throw OpenRAVE::openrave_exception("GetCurrentReport robot not found!");
   
   // get current space we're talking about
   Space & space = this->get_current_space(robot);
   
   // find the subset
   std::map< std::string, boost::weak_ptr<Subset> >::iterator it;
   it = space.named_subsets.find(subset_name);
   if (it == space.named_subsets.end())
      throw OpenRAVE::openrave_exception("subset not found!");
   boost::shared_ptr<Subset> subset = it->second.lock();
   if (!subset)
      throw OpenRAVE::openrave_exception("subset got deleted!");
   subset->cost_override = new_cost;
   
   return true;
}

bool or_lemur::ModuleSubsetManager::TagCurrentSubset(
   std::ostream & sout, std::istream & sin)
{
   OpenRAVE::RobotBasePtr robot;
   std::string tagname;
   bool persistent;
   
   // parse args
   {
      std::vector<std::string> args = args_from_sin(sin);
      if (args.size() != 3)
         throw OpenRAVE::openrave_exception("TagCurrentSubset args not correct!");
         
      // robot
      std::string robotname = args[0];
      robot = this->penv->GetRobot(robotname);
      if (!robot)
         throw OpenRAVE::openrave_exception("TagCurrentSubset robot not found!");
      
      // tag
      tagname = args[1];
      
      // persistent
      std::string boolstr = args[2];
      if (boolstr=="true" || boolstr=="True" || boolstr=="1")
         persistent = true;
      else if (boolstr=="false" || boolstr=="False" || boolstr=="0")
         persistent = false;
      else
         throw OpenRAVE::openrave_exception("TagCurrentSubset persistent not boolean!");
   }
   
   this->tag_current_subset(robot, tagname, persistent);
   
   return true;
}

bool or_lemur::ModuleSubsetManager::DumpSubsets(std::ostream & sout, std::istream & sin)
{
   OpenRAVE::RobotBasePtr robot;
   std::string dotfile;
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
      if (args.size() != 2)
         throw OpenRAVE::openrave_exception("DumpSubsets args not correct!");
      // robot
      std::string robotname = args[0];
      robot = this->penv->GetRobot(robotname);
      if (!robot)
         throw OpenRAVE::openrave_exception("DumpSubsets robot not found!");
      dotfile = args[1];
   }
   this->dump_subsets(robot, dotfile);
   return true;
}

or_lemur::ModuleSubsetManager::Space &
or_lemur::ModuleSubsetManager::get_current_space(const OpenRAVE::RobotBasePtr robot)
{
   struct SpaceKey space_key;
   space_key.robot_name = robot->GetName();
   space_key.active_dofs = robot->GetActiveDOFIndices();
   std::map<SpaceKey, Space>::iterator space_it = this->spaces.find(space_key);
   if (space_it != this->spaces.end() && space_it->second.robot.lock() != robot)
   {
      this->spaces.erase(space_it);
      space_it = this->spaces.end();
   }
   if (space_it == this->spaces.end())
   {
      Space space;
      space.robot = robot;
      space_it = this->spaces.insert(std::pair<SpaceKey,Space>(space_key,space)).first;
   }
   return space_it->second;
}

std::set< boost::shared_ptr<or_lemur::ModuleSubsetManager::Subset> >
or_lemur::ModuleSubsetManager::get_existing_subsets(
   const or_lemur::ModuleSubsetManager::Space & space)
{
   std::set< boost::shared_ptr<Subset> >::iterator sspp;
   std::set< boost::shared_ptr<Subset> >::iterator sspp2;
   std::set< boost::shared_ptr<Subset> > existing_subsets;
   for (sspp=space.persistent_subsets.begin(); sspp!=space.persistent_subsets.end(); sspp++)
   {
      existing_subsets.insert(*sspp);
      for (sspp2=(**sspp).base_subsets.begin(); sspp2!=(**sspp).base_subsets.end(); sspp2++)
         existing_subsets.insert(*sspp2);
   }
   return existing_subsets;
}

boost::shared_ptr<or_lemur::ModuleSubsetManager::Subset>
or_lemur::ModuleSubsetManager::retrieve_subset(
   or_lemur::ModuleSubsetManager::Space & space,
   const std::vector<or_lemur::InterLinkCheck> ilcs_vec,
   bool make_persistent)
{
   std::set< boost::shared_ptr<InterLinkCheck> >::iterator ilcpp;
   std::set< boost::shared_ptr<Subset> >::iterator sspp;
   std::set< boost::weak_ptr<Subset> >::iterator sswp;
   
   // construct the set of all currently known subsets
   // (only need to go two layers deep due to derived/base structure)
   std::set< boost::shared_ptr<Subset> > existing_subsets
      = this->get_existing_subsets(space);
   
   // get all exising managed ilcs
   std::set< boost::shared_ptr<InterLinkCheck> > existing_ilcs;
   for (sspp=existing_subsets.begin(); sspp!=existing_subsets.end(); sspp++)
      existing_ilcs.insert((*sspp)->ilcs.begin(), (*sspp)->ilcs.end());
   
   // construct a potential current subset
   // with managed ilcs
   boost::shared_ptr<Subset> ss_current(new Subset());
   ss_current->cost_override = std::numeric_limits<double>::quiet_NaN();
   for (unsigned int i=0; i<ilcs_vec.size(); i++)
   {
      for (ilcpp=existing_ilcs.begin(); ilcpp!=existing_ilcs.end(); ilcpp++)
         if (**ilcpp == ilcs_vec[i])
            break;
      if (ilcpp!=existing_ilcs.end()) // we broke
         ss_current->ilcs.insert(*ilcpp);
      else
         ss_current->ilcs.insert(boost::shared_ptr<InterLinkCheck>(
            new InterLinkCheck(ilcs_vec[i])));
   }
   
   // does this subset already exist? if so, we're done!
   for (sspp=existing_subsets.begin(); sspp!=existing_subsets.end(); sspp++)
      if (*ss_current == **sspp)
         break;
   if (sspp!=existing_subsets.end())
      return *sspp;
      
   // ok, subset doesnt already exist
   
   if (!make_persistent)
      return boost::shared_ptr<Subset>();
   
   // add subset ss_current, and set as permanent!
   space.persistent_subsets.insert(ss_current);
   
   // ok, this is current a BASE subset
   // (since it doesnt point to anything),
   // but we must ensure that base subsets are DISJOINT,
   // so lets iterate over all existing base subsets;
   // if we find relations, we'll turn ourself into a DERIVED subset instead!
   for (sspp=existing_subsets.begin(); sspp!=existing_subsets.end(); sspp++)
   if ((**sspp).base_subsets.size() == 0)
   {
      // compute intersection of my ilcs and this base subset's ilcs
      std::set< boost::shared_ptr<InterLinkCheck> > ilcs_insersection;
      std::set_intersection(
         ss_current->ilcs.begin(), ss_current->ilcs.end(),
         (*sspp)->ilcs.begin(), (*sspp)->ilcs.end(),
         std::inserter(ilcs_insersection, ilcs_insersection.begin()));
      
      // skip if there's no relation to this base
      if (ilcs_insersection.size() == 0)
         continue;
      
      // are we using EVERYTHING in this base?
      // this makes us a simple derived subset (if we werent already)
      if (ilcs_insersection.size() == (*sspp)->ilcs.size())
      {
         ss_current->base_subsets.insert(*sspp);
         (**sspp).derived_subsets.insert(ss_current);
         continue;
      }
      
      // ok, we need to split this base subset **sspp
      // into two fundamental new bases,
      // one which includes my checks,
      // and one that excludes them
      boost::shared_ptr<Subset> ss_newbase_inc(new Subset());
      boost::shared_ptr<Subset> ss_newbase_exc(new Subset());
      ss_newbase_inc->cost_override = std::numeric_limits<double>::quiet_NaN();
      ss_newbase_exc->cost_override = std::numeric_limits<double>::quiet_NaN();
      
      // first, get the ilcs correct
      ss_newbase_inc->ilcs = ilcs_insersection;
      std::set_difference(
         (*sspp)->ilcs.begin(), (*sspp)->ilcs.end(),
         ilcs_insersection.begin(), ilcs_insersection.end(),
         std::inserter(ss_newbase_exc->ilcs, ss_newbase_exc->ilcs.begin()));
      
      // the old base now points to these two
      (**sspp).base_subsets.insert(ss_newbase_inc);
      (**sspp).base_subsets.insert(ss_newbase_exc);
      ss_newbase_inc->derived_subsets.insert(*sspp);
      ss_newbase_exc->derived_subsets.insert(*sspp);
      
      // any deriveds of the old base now derives from both the new ones instead
      for (sswp=(**sspp).derived_subsets.begin(); sswp!=(**sspp).derived_subsets.end(); sswp++)
      {
         boost::shared_ptr<Subset> ss_derived = (*sswp).lock();
         if (!ss_derived)
            continue;
         ss_derived->base_subsets.erase(*sspp);
         ss_derived->base_subsets.insert(ss_newbase_inc);
         ss_derived->base_subsets.insert(ss_newbase_exc);
         ss_newbase_inc->derived_subsets.insert(ss_derived);
         ss_newbase_exc->derived_subsets.insert(ss_derived);
      }
      (**sspp).derived_subsets.clear();
      
      // and we can now derive solely from the inclusive base!
      ss_current->base_subsets.insert(ss_newbase_inc);
      ss_newbase_inc->derived_subsets.insert(ss_current);
   }
   
   // if we're now derived,
   // make a new base for any new checks!
   if (ss_current->base_subsets.size())
   {
      boost::shared_ptr<Subset> ss_newbase(new Subset());
      ss_newbase->cost_override = std::numeric_limits<double>::quiet_NaN();
      ss_newbase->ilcs = ss_current->ilcs;
      for (sspp=ss_current->base_subsets.begin(); sspp!=ss_current->base_subsets.end(); sspp++)
         for (ilcpp=(**sspp).ilcs.begin(); ilcpp!=(**sspp).ilcs.end(); ilcpp++)
            ss_newbase->ilcs.erase(*ilcpp);
      if (ss_newbase->ilcs.size())
      {
         ss_current->base_subsets.insert(ss_newbase);
         ss_newbase->derived_subsets.insert(ss_current);
      }
   }
   
   // SPECIAL CASE: if I *am* the included one,
   // (such that I am derived from exactly one base)
   // (this happens when I am a proper subset of an existing base)
   // I am actually the base, not the bogus derived.
   if (ss_current->base_subsets.size() == 1)
   {
      // decouple and replace!
      boost::shared_ptr<Subset> ss_base = *(ss_current->base_subsets.begin());
      ss_base->derived_subsets.erase(ss_current);
      space.persistent_subsets.erase(ss_current);
      space.persistent_subsets.insert(ss_base);
      ss_current = ss_base;
   }
   
   return ss_current;
}


// tag current subset if it exists
void or_lemur::ModuleSubsetManager::tag_current_subset(
   const OpenRAVE::RobotBasePtr robot,
   std::string new_tag,
   bool persistent)
{
   Space & space = this->get_current_space(robot);
   
   // get current subset
   std::vector<InterLinkCheck> current_ilcs_vec;
   or_lemur::compute_checks(robot, current_ilcs_vec);
   boost::shared_ptr<Subset> subset = this->retrieve_subset(space, current_ilcs_vec, persistent);
   if (!subset)
   {
      RAVELOG_WARN("asked to tag subset which doesnt yet exist, and not asked to make persistent! doing nothing ...\n");
      return;
   }

   // are we trying to assign a new tag?
   if (new_tag.size())
   {
      // does this subset already have a tag?
      if (subset->tag.size())
      {
         if (new_tag == subset->tag)
            return;
         // are we trying to tag to an already tagged subset?
         throw OpenRAVE::openrave_exception(sf(
            "trying to assign a subset tag %s, but it already has one %s!",
            new_tag.c_str(), subset->tag.c_str()));
      }
      // make sure this space doesnt already have a subset with this tag
      bool success = this->set_subset_tag(space, subset, new_tag);
      if (!success)
         throw OpenRAVE::openrave_exception(
            "trying to assign a subset tag, but exising subset already has this tag!");
   }
}

bool or_lemur::ModuleSubsetManager::GetCurrentReport(
   std::ostream & sout, std::istream & sin)
{
   unsigned int ui;
   unsigned int ui2;
   std::vector<std::string> args = args_from_sin(sin);
   if (args.size() != 1)
      throw OpenRAVE::openrave_exception("GetCurrentReport args not correct!");
   std::string robotname = args[0];
   OpenRAVE::RobotBasePtr robot = this->penv->GetRobot(robotname);
   if (!robot)
      throw OpenRAVE::openrave_exception("GetCurrentReport robot not found!");
   or_lemur::SubsetReport report;
   this->get_current_report(robot, report);
   // dump subsets
   std::streamsize old_precision = sout.precision();
   sout.precision(2 + std::numeric_limits<OpenRAVE::dReal>::digits10);
   for (ui=0; ui<report.subsets.size(); ui++)
      sout << "subset " << report.subsets[ui].name << " cost " << report.subsets[ui].cost << std::endl;
   sout.precision(old_precision);
   sout << "current_subset " << report.current_subset << std::endl;
   // dump relations
   for (ui=0; ui<report.inclusions.size(); ui++)
      sout << "inclusion subset " << report.inclusions[ui].subset << " superset " << report.inclusions[ui].superset << std::endl;
   for (ui=0; ui<report.intersections.size(); ui++)
   {
      sout << "intersection subset " << report.intersections[ui].subset << " supersets";
      for (ui2=0; ui2<report.intersections[ui].supersets.size(); ui2++)
         sout << " " << report.intersections[ui].supersets[ui2];
      sout << std::endl;
   }
   return true;
}

// given current active dofs
// this will auto-tag if necessary
// this will add persistency to this set!
// this is primarily where names come from!
// once a name comes out of this, it's persistent!
void or_lemur::ModuleSubsetManager::get_current_report(
      const OpenRAVE::RobotBasePtr robot,
      or_lemur::SubsetReport & report)
{
   std::set< boost::shared_ptr<Subset> >::iterator sspp;
   std::set< boost::shared_ptr<Subset> >::iterator sspp2;
   std::set< boost::shared_ptr<InterLinkCheck> >::iterator iilc;
   
   // get current space we're talking about
   Space & space = this->get_current_space(robot);
   
   // step 1: ensure there's a subset for the current config,
   // and get its name
   std::vector<InterLinkCheck> current_ilcs_vec;
   or_lemur::compute_checks(robot, current_ilcs_vec);
   boost::shared_ptr<Subset> subset = this->retrieve_subset(space, current_ilcs_vec, true); // persistent=true
   
   // get existing subsets
   std::set< boost::shared_ptr<Subset> > existing_subsets = this->get_existing_subsets(space);
   
   // update live checks
   std::vector<struct LiveCheck> live_checks;
   or_lemur::compute_live_checks(robot, live_checks);
   for (sspp=existing_subsets.begin(); sspp!=existing_subsets.end(); sspp++)
   {
      (*sspp)->live_checks.clear();
      
      // our our ilcs INCLUDED in the current subset's ilcs?
      // if not, abort!
      if (!std::includes(subset->ilcs.begin(),subset->ilcs.end(),
            (*sspp)->ilcs.begin(), (*sspp)->ilcs.end()))
         continue;
      
      // ok, then all of this subset's links are in the right positions;
      // what checks should we perform to cover them?
      
      std::set<
         std::pair<OpenRAVE::KinBody::LinkConstPtr,OpenRAVE::KinBody::LinkConstPtr>
         >::iterator cit;
      
      // all checks in this subset
      std::set<
         std::pair<OpenRAVE::KinBody::LinkConstPtr,OpenRAVE::KinBody::LinkConstPtr>
         > subset_checks;
      for (iilc=(*sspp)->ilcs.begin(); iilc!=(*sspp)->ilcs.end(); iilc++)
         subset_checks.insert(std::make_pair((*iilc)->link1, (*iilc)->link2));
      
      std::set<
         std::pair<OpenRAVE::KinBody::LinkConstPtr,OpenRAVE::KinBody::LinkConstPtr>
         > checks_unclaimed = subset_checks;
      
      // what about this live check?
      for (std::vector<struct LiveCheck>::iterator
         lc=live_checks.begin(); lc!=live_checks.end(); lc++)
      {
         std::set<
            std::pair<OpenRAVE::KinBody::LinkConstPtr,OpenRAVE::KinBody::LinkConstPtr>
            > checks_claimed;
         
         // are its links checked all a part of this subset?
         for (cit=lc->links_checked.begin(); cit!=lc->links_checked.end(); cit++)
         {
            if (subset_checks.find(*cit) == subset_checks.end())
               break; // no!
            if (checks_unclaimed.find(*cit) != checks_unclaimed.end())
               checks_claimed.insert(*cit);
         }
         if (cit!=lc->links_checked.end()) // we broke, this live check does too much!
            continue;
         
         // ok, this live check would work!
         // does it claim at least one unclaimed 
         if (checks_claimed.size() <= 1)
            continue;
         
         // ok, add this lc!
         (*sspp)->live_checks.push_back(*lc);
         
         // remove from unclaimed
         for (cit=checks_claimed.begin(); cit!=checks_claimed.end(); cit++)
            checks_unclaimed.erase(*cit);
      }
      
      // deal with unclaimed
      for (cit=checks_unclaimed.begin(); cit!=checks_unclaimed.end(); cit++)
      {
         or_lemur::LiveCheck lc;
         lc.type = or_lemur::LiveCheck::TYPE_LINK_LINK;
         lc.link = cit->first;
         lc.link_other = cit->second;
         (*sspp)->live_checks.push_back(lc);
      }
   }
   
   // compose report
   // subsets (this will also assign names if necessary)
   for (sspp=existing_subsets.begin(); sspp!=existing_subsets.end(); sspp++)
   {
      // subsets
      or_lemur::SubsetReport::Subset report_subset;
      
      // name
      if (!(*sspp)->tag.size())
      {
         // make sure name is unique!
         unsigned int i;
         for (i=0; i<100; i++)
         {
            std::string new_tag = sf("untagged_%02u", i);
            bool success = this->set_subset_tag(space, *sspp, new_tag);
            if (success)
               break;
         }
         if (!(i<100))
            throw OpenRAVE::openrave_exception(
               "couldnt generate a unique untagged name!");
      }
      report_subset.name = (*sspp)->tag;
      
      // cost
      if (std::isnan((*sspp)->cost_override))
         report_subset.cost = this->cost_per_ilc * ((*sspp)->ilcs.size()+1);
      else
         report_subset.cost = (*sspp)->cost_override;
      
      // we need to validate that this check CAN be performed
      // in the current environment!
      report_subset.indicator = boost::bind(
            &or_lemur::ModuleSubsetManager::indicator, this,
            (*sspp), robot, _1);
      
      report.subsets.push_back(report_subset);
   }
   
   for (sspp=existing_subsets.begin(); sspp!=existing_subsets.end(); sspp++)
   {   
      // intersections if we're derived
      if ((*sspp)->base_subsets.size())
      {
         or_lemur::SubsetReport::Intersection report_intersection;
         report_intersection.subset = (*sspp)->tag;
         for (sspp2=(*sspp)->base_subsets.begin(); sspp2!=(*sspp)->base_subsets.end(); sspp2++)
            report_intersection.supersets.push_back((*sspp2)->tag);
         report.intersections.push_back(report_intersection);
      }
   }
   
   report.current_subset = subset->tag;
   
   // get ready for checks
   this->checker = penv->GetCollisionChecker();
}

void or_lemur::ModuleSubsetManager::dump_subsets(const OpenRAVE::RobotBasePtr robot, std::string dotfile)
{
   std::set< boost::shared_ptr<Subset> >::iterator sspp;
   std::set< boost::shared_ptr<Subset> >::iterator sspp2;
   std::set< boost::shared_ptr<InterLinkCheck> >::iterator ilcpp;
   
   FILE * fp = 0;
   if (dotfile != "-")
      fp = fopen(dotfile.c_str(), "w");
      
   if (fp)
   {
      fprintf(fp, "digraph subsets {\n");
   }
   
   // get current space we're talking about
   Space & space = this->get_current_space(robot);
   
   std::set< boost::shared_ptr<Subset> > existing_subsets = this->get_existing_subsets(space);
   for (sspp=existing_subsets.begin(); sspp!=existing_subsets.end(); sspp++)
   {
      std::string my_name = (*sspp)->tag.size() ? (*sspp)->tag : sf("%p",(*sspp).get());
      if ((*sspp)->base_subsets.size())
      {
         // derived space
         if (fp)
            fprintf(fp,"   node_%s [label=\"%s\",shape=box];\n", my_name.c_str(), my_name.c_str());
         else
            printf("subset '%s' (%p) is intersection of %lu:\n",
               (*sspp)->tag.c_str(), (*sspp).get(), (*sspp)->base_subsets.size());
         for (sspp2=(*sspp)->base_subsets.begin(); sspp2!=(*sspp)->base_subsets.end(); sspp2++)
         {
            if (fp)
            {
               std::string base_name = (*sspp2)->tag.size() ? (*sspp2)->tag : sf("%p",(*sspp2).get());
               fprintf(fp,"   node_%s -> node_%s;\n", my_name.c_str(), base_name.c_str());
            }
            else
               printf("  '%s' (%p)\n", (*sspp2)->tag.c_str(), (*sspp2).get());
         }
         
         if (!fp)
         {
            printf("  live checks:\n");
            for (unsigned int ui=0; ui<(*sspp)->live_checks.size(); ui++)
            {
               switch ((*sspp)->live_checks[ui].type)
               {
               case or_lemur::LiveCheck::TYPE_KINBODY:
                  printf("  CheckCollision(%s)\n",
                     (*sspp)->live_checks[ui].kinbody->GetName().c_str());
                  break;
               case or_lemur::LiveCheck::TYPE_LINK:
                  printf("  CheckCollision(%s:%s)\n",
                     (*sspp)->live_checks[ui].link->GetParent()->GetName().c_str(),
                     (*sspp)->live_checks[ui].link->GetName().c_str());
                  break;
               case or_lemur::LiveCheck::TYPE_LINK_LINK:
                  printf("  CheckCollision(%s:%s, %s:%s)\n",
                     (*sspp)->live_checks[ui].link->GetParent()->GetName().c_str(),
                     (*sspp)->live_checks[ui].link->GetName().c_str(),
                     (*sspp)->live_checks[ui].link_other->GetParent()->GetName().c_str(),
                     (*sspp)->live_checks[ui].link_other->GetName().c_str());
                  break;
               case or_lemur::LiveCheck::TYPE_SELFSA_KINBODY:
                  printf("  CheckStandaloneSelfCollision(%s)\n",
                     (*sspp)->live_checks[ui].kinbody->GetName().c_str());
                  break;
               default:
                  printf("  UNKNOWN LIVE-CHECK!\n");
               }
            }
         }
      }
      else
      {
         // base space
         if (fp)
            fprintf(fp,"   node_%s [label=\"%s (%lu)\",shape=box];\n", my_name.c_str(), my_name.c_str(), (*sspp)->ilcs.size());
         else
         {
            printf("subset '%s' (%p) has %lu ilcs:\n",
               (*sspp)->tag.c_str(), (*sspp).get(), (*sspp)->ilcs.size());
            for (ilcpp=(*sspp)->ilcs.begin(); ilcpp!=(*sspp)->ilcs.end(); ilcpp++)
               printf("  ilc between '%s:%s' and '%s:%s'\n",
                  (*ilcpp)->link1->GetParent()->GetName().c_str(),
                  (*ilcpp)->link1->GetName().c_str(),
                  (*ilcpp)->link2->GetParent()->GetName().c_str(),
                  (*ilcpp)->link2->GetName().c_str());

            printf("  live checks:\n");
            for (unsigned int ui=0; ui<(*sspp)->live_checks.size(); ui++)
            {
               switch ((*sspp)->live_checks[ui].type)
               {
               case or_lemur::LiveCheck::TYPE_KINBODY:
                  printf("  CheckCollision(%s)\n",
                     (*sspp)->live_checks[ui].kinbody->GetName().c_str());
                  break;
               case or_lemur::LiveCheck::TYPE_LINK:
                  printf("  CheckCollision(%s:%s)\n",
                     (*sspp)->live_checks[ui].link->GetParent()->GetName().c_str(),
                     (*sspp)->live_checks[ui].link->GetName().c_str());
                  break;
               case or_lemur::LiveCheck::TYPE_LINK_LINK:
                  printf("  CheckCollision(%s:%s, %s:%s)\n",
                     (*sspp)->live_checks[ui].link->GetParent()->GetName().c_str(),
                     (*sspp)->live_checks[ui].link->GetName().c_str(),
                     (*sspp)->live_checks[ui].link_other->GetParent()->GetName().c_str(),
                     (*sspp)->live_checks[ui].link_other->GetName().c_str());
                  break;
               case or_lemur::LiveCheck::TYPE_SELFSA_KINBODY:
                  printf("  CheckStandaloneSelfCollision(%s)\n",
                     (*sspp)->live_checks[ui].kinbody->GetName().c_str());
                  break;
               default:
                  printf("  UNKNOWN LIVE-CHECK!\n");
               }
            }
         }
      }
   }
   
   if (fp)
   {
      fprintf(fp, "}\n");
      fclose(fp);
   }
}

bool or_lemur::ModuleSubsetManager::set_subset_tag(
   Space & space, boost::shared_ptr<Subset> subset, std::string new_tag)
{
   // does this space already have a subset with this tag?
   std::map< std::string, boost::weak_ptr<Subset> >::iterator it;
   it = space.named_subsets.find(new_tag);
   if (it != space.named_subsets.end())
   {
      boost::shared_ptr<Subset> existing_subset = it->second.lock();
      if (existing_subset)
      {
         if (existing_subset->tag == new_tag)
            return false;
      }
      else
         space.named_subsets.erase(it);
   }
   // ok, we're good to go!
   subset->tag = new_tag;
   space.named_subsets[new_tag] = subset;
   return true;
}

bool or_lemur::ModuleSubsetManager::indicator(
   boost::shared_ptr<Subset> subset,
   OpenRAVE::RobotBasePtr robot,
   std::vector<OpenRAVE::dReal> & adofvals)
{
   robot->SetActiveDOFValues(adofvals, OpenRAVE::KinBody::CLA_Nothing);
   
   if (!subset->live_checks.size())
      throw OpenRAVE::openrave_exception("cant check subset, no live checks!");
   
   for (std::vector<struct or_lemur::LiveCheck>::iterator
      it=subset->live_checks.begin(); it!=subset->live_checks.end(); it++)
   switch (it->type)
   {
   case or_lemur::LiveCheck::TYPE_KINBODY:
      if (this->checker->CheckCollision(it->kinbody)) return false; break;
   case or_lemur::LiveCheck::TYPE_LINK:
      if (this->checker->CheckCollision(it->link)) return false; break;
   case or_lemur::LiveCheck::TYPE_LINK_LINK:
      if (this->checker->CheckCollision(it->link, it->link_other)) return false; break;
   case or_lemur::LiveCheck::TYPE_SELFSA_KINBODY:
      if (this->checker->CheckStandaloneSelfCollision(it->kinbody)) return false; break;
   default:
      throw OpenRAVE::openrave_exception("unknown livecheck type!");
   }
   
   return true;

}
