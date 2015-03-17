#include <cstdarg>
#include <openrave/openrave.h>
#include <or_multiset/inter_link_checks.h>
#include "module_subset_manager.h"

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

or_multiset::ModuleSubsetManager::ModuleSubsetManager(OpenRAVE::EnvironmentBasePtr penv):
   OpenRAVE::ModuleBase(penv), penv(penv)
{
   __description = "ModuleSubsetManager description";
   this->RegisterCommand("GetName",
      boost::bind(&or_multiset::ModuleSubsetManager::GetName,this,_1,_2),
      "GetName");
   this->RegisterCommand("TagCurrentSubset",
      boost::bind(&or_multiset::ModuleSubsetManager::TagCurrentSubset,this,_1,_2),
      "TagCurrentSubset");
   this->RegisterCommand("DumpSubsets",
      boost::bind(&or_multiset::ModuleSubsetManager::DumpSubsets,this,_1,_2),
      "DumpSubsets");
}

or_multiset::ModuleSubsetManager::~ModuleSubsetManager()
{
   printf("SubsetManager module destructed!\n");
}

// on environment add
int or_multiset::ModuleSubsetManager::main(const std::string & cmd)
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
      boost::shared_ptr<or_multiset::ModuleSubsetManager> mod
         = boost::dynamic_pointer_cast<or_multiset::ModuleSubsetManager>(*it);
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

void or_multiset::ModuleSubsetManager::Destroy()
{
   printf("SubsetManager module removed from environment!\n");
}

bool or_multiset::ModuleSubsetManager::GetName(std::ostream & sout, std::istream & sin)
{
   sout << this->name;
   return true;
}

bool or_multiset::ModuleSubsetManager::TagCurrentSubset(
   std::ostream & sout, std::istream & sin)
{
   OpenRAVE::RobotBasePtr robot;
   std::string tagname;
   bool persistent;
   
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

bool or_multiset::ModuleSubsetManager::DumpSubsets(std::ostream & sout, std::istream & sin)
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

or_multiset::ModuleSubsetManager::Space &
or_multiset::ModuleSubsetManager::get_current_space(const OpenRAVE::RobotBasePtr robot)
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

std::set< boost::shared_ptr<or_multiset::ModuleSubsetManager::Subset> >
or_multiset::ModuleSubsetManager::get_existing_subsets(
   const or_multiset::ModuleSubsetManager::Space & space)
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

boost::shared_ptr<or_multiset::ModuleSubsetManager::Subset>
or_multiset::ModuleSubsetManager::retrieve_subset(
   or_multiset::ModuleSubsetManager::Space & space,
   const std::vector<or_multiset::InterLinkCheck> ilcs_vec,
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
void or_multiset::ModuleSubsetManager::tag_current_subset(
   const OpenRAVE::RobotBasePtr robot,
   std::string new_tag,
   bool persistent)
{
   Space & space = this->get_current_space(robot);
   
   // get current subset
   std::vector<InterLinkCheck> current_ilcs_vec;
   or_multiset::compute_checks(robot, current_ilcs_vec);
   boost::shared_ptr<Subset> subset = this->retrieve_subset(space, current_ilcs_vec, persistent);
   if (!subset)
   {
      RAVELOG_WARN("asked to tag subset which doesnt yet exist, and not asked to make persistent!\n");
      return;
   }

   // are we trying to assign a new tag?
   if (new_tag.size())
   {
      // are we trying to assign to an already tagged subset?
      if (subset->tag.size() && new_tag != subset->tag)
         throw OpenRAVE::openrave_exception(sf(
            "trying to assign a subset tag %s, but it already has one %s!",
            new_tag.c_str(), subset->tag.c_str()));
      // make sure this space doesnt already have a subset with this tag
      bool success = this->set_subset_tag(space, subset, new_tag);
      if (!success)
         throw OpenRAVE::openrave_exception(
            "trying to assign a subset tag, but exising subset already has this tag!");
   }
}

// given current active dofs
// this will auto-tag if necessary
// this will add persistency to this set!
// this is primarily where names come from!
// once a name comes out of this, it's persistent!
void or_multiset::ModuleSubsetManager::get_current_report(
      const OpenRAVE::RobotBasePtr robot,
      or_multiset::SubsetReport & report)
{
   std::set< boost::shared_ptr<Subset> >::iterator sspp;
   std::set< boost::shared_ptr<Subset> >::iterator sspp2;
   
   // get current space we're talking about
   Space & space = this->get_current_space(robot);
   
   // step 1: ensure there's a subset for the current config,
   // and get its name
   std::vector<InterLinkCheck> current_ilcs_vec;
   or_multiset::compute_checks(robot, current_ilcs_vec);
   boost::shared_ptr<Subset> subset = this->retrieve_subset(space, current_ilcs_vec, true); // persistent=true
   
   // compose report
   std::set< boost::shared_ptr<Subset> > existing_subsets = this->get_existing_subsets(space);
   
   // subsets (this will also assign names if necessary)
   for (sspp=existing_subsets.begin(); sspp!=existing_subsets.end(); sspp++)
   {
      // subsets
      or_multiset::SubsetReport::Subset report_subset;
      
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
      report_subset.cost = 60.0e-6 * ((*sspp)->ilcs.size()+1);
      
      // we need to validate that this check CAN be performed
      // in the current environment!
      report_subset.indicator = boost::bind(
            &or_multiset::ModuleSubsetManager::indicator, this,
            (*sspp), robot, _1);
      
      report.subsets.push_back(report_subset);
   }
   
   for (sspp=existing_subsets.begin(); sspp!=existing_subsets.end(); sspp++)
   {   
      // intersections if we're derived
      if ((*sspp)->base_subsets.size())
      {
         or_multiset::SubsetReport::Intersection report_intersection;
         report_intersection.subset = (*sspp)->tag;
         for (sspp2=(*sspp)->base_subsets.begin(); sspp2!=(*sspp)->base_subsets.end(); sspp2++)
            report_intersection.supersets.push_back((*sspp2)->tag);
         report.intersections.push_back(report_intersection);
      }
   }
   
   report.current_subset = subset->tag;
}

void or_multiset::ModuleSubsetManager::dump_subsets(const OpenRAVE::RobotBasePtr robot, std::string dotfile)
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
         }
      }
   }
   
   if (fp)
   {
      fprintf(fp, "}\n");
      fclose(fp);
   }
}

bool or_multiset::ModuleSubsetManager::set_subset_tag(
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

bool or_multiset::ModuleSubsetManager::indicator(
   boost::shared_ptr<Subset> subset,
   OpenRAVE::RobotBasePtr robot,
   std::vector<OpenRAVE::dReal> & adofvals)
{
   bool isvalid;
   std::set< boost::shared_ptr<InterLinkCheck> >::iterator iilc;
   
   robot->SetActiveDOFValues(adofvals, OpenRAVE::KinBody::CLA_Nothing);
   
   isvalid = true;
   for (iilc=subset->ilcs.begin(); iilc!=subset->ilcs.end(); iilc++)
   {
      isvalid = !(this->penv->CheckCollision((*iilc)->link1, (*iilc)->link2));
      if (!isvalid)
         break;
   }
   
   return isvalid;
}
