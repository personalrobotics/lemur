#include <openrave/openrave.h>
#include <or_multiset/inter_link_checks.h>
#include "module_subset_manager.h"

or_multiset::ModuleSubsetManager::ModuleSubsetManager(OpenRAVE::EnvironmentBasePtr penv):
   OpenRAVE::ModuleBase(penv), penv(penv)
{
   __description = "ModuleSubsetManager description";
   this->RegisterCommand("TagCurrentSubset",
      boost::bind(&or_multiset::ModuleSubsetManager::TagCurrentSubset,this,_1,_2),
      "TagCurrentSubset");
}

or_multiset::ModuleSubsetManager::~ModuleSubsetManager()
{
   printf("SubsetManager module destructed!\n");
}

// on environment add/remove
int or_multiset::ModuleSubsetManager::main(const std::string& cmd)
{
   printf("SubsetManager module added to environment with string: |%s|\n", cmd.c_str());
   return 0;
}

void or_multiset::ModuleSubsetManager::Destroy()
{
   printf("SubsetManager module removed from environment!\n");
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

// tag current subset if it exists
void or_multiset::ModuleSubsetManager::tag_current_subset(
   const OpenRAVE::RobotBasePtr robot,
   std::string new_tag,
   bool persistent)
{
   std::set< boost::shared_ptr<InterLinkCheck> >::iterator ilcpp;
   std::set< boost::shared_ptr<Subset> >::iterator sspp;
   std::set< boost::shared_ptr<Subset> >::iterator sspp2;
   std::set< boost::weak_ptr<Subset> >::iterator sswp;
   
   // construct the set of all currently known subsets
   // (only need to go two layers deep due to derived/base structure)
   std::set< boost::shared_ptr<Subset> > existing_subsets;
   for (sspp=this->persistent_subsets.begin(); sspp!=this->persistent_subsets.end(); sspp++)
   {
      existing_subsets.insert(*sspp);
      for (sspp2=(**sspp).base_subsets.begin(); sspp2!=(**sspp).base_subsets.end(); sspp2++)
         existing_subsets.insert(*sspp2);
   }
   
   // get all exising managed ilcs
   std::set< boost::shared_ptr<InterLinkCheck> > existing_ilcs;
   for (sspp=existing_subsets.begin(); sspp!=existing_subsets.end(); sspp++)
      existing_ilcs.insert((*sspp)->ilcs.begin(), (*sspp)->ilcs.end());
   
   // get current ilcs
   std::vector<InterLinkCheck> current_ilcs_vec;
   or_multiset::compute_checks(robot, current_ilcs_vec);
   
   // construct a potential current subset
   // with managed ilcs
   boost::shared_ptr<Subset> ss_current(new Subset());
   for (unsigned int i=0; i<current_ilcs_vec.size(); i++)
   {
      for (ilcpp=existing_ilcs.begin(); ilcpp!=existing_ilcs.end(); ilcpp++)
         if (**ilcpp == current_ilcs_vec[i])
            break;
      if (ilcpp!=existing_ilcs.end()) // we broke
         ss_current->ilcs.insert(*ilcpp);
      else
         ss_current->ilcs.insert(boost::shared_ptr<InterLinkCheck>(
            new InterLinkCheck(current_ilcs_vec[i])));
   }
   
   // does this subset already exist?
   for (sspp=existing_subsets.begin(); sspp!=existing_subsets.end(); sspp++)
      if (*ss_current == **sspp)
         break;
   if (sspp!=existing_subsets.end()) // we broke
   {
      // are we trying to assign a new tag?
      if (new_tag.size())
      {
         if ((**sspp).tag.size())
            throw OpenRAVE::openrave_exception("trying to assign a subset tag, but it already has one!");
         (**sspp).tag = new_tag;
      }
      // are we trying to make it persistent?
      if (persistent)
         this->persistent_subsets.insert(*sspp);
      // done!
      return;
   }
   
   // ok, subset doesnt already exist
   if (!persistent)
   {
      RAVELOG_WARN("tag_current_subset called, but subset doesnt exist yet, and not persistent, so skipping!\n");
      return;
   }
   
   // add subset ss_current, and set as permanent!
   this->persistent_subsets.insert(ss_current);
   
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
      for (sspp=ss_current->base_subsets.begin(); sspp!=ss_current->base_subsets.end(); sspp++)
         for (ilcpp=(**sspp).ilcs.begin(); ilcpp!=(**sspp).ilcs.end(); ilcpp++)
            ss_newbase->ilcs.erase(*ilcpp);
      if (ss_newbase->ilcs.size())
      {
         ss_current->base_subsets.insert(ss_newbase);
         ss_newbase->derived_subsets.insert(ss_current);
      }
   }
}

// given current active dofs
// this will auto-tag if necessary
// this will add persistency to this set!
// this is primarily where names come from!
// once a name comes out of this, it's persistent!
void or_multiset::ModuleSubsetManager::get_report(
      const OpenRAVE::RobotBasePtr robot,
      or_multiset::SubsetReport & report)
{
}

