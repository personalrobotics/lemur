/* File: module_subset_manager.h
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

namespace or_multiset
{

// this is the data structure produced by the manager
// on request of the planner
// for a particular vector of robot active dofs.
struct SubsetReport
{
   struct Subset
   {
      std::string name;
      
      OpenRAVE::dReal cost;
      
      // function will be empty if subset membership can't be tested
      // note this will hold shared pointers to subsets and robots and the like
      boost::function<bool (std::vector<OpenRAVE::dReal> &)> indicator;
   };
   std::vector<Subset> subsets;
   
   std::string current_subset;
   
   struct Inclusion
   {
      std::string subset;
      std::string superset;
   };
   std::vector<Inclusion> inclusions;
   
   struct Intersection
   {
      std::string subset;
      std::vector<std::string> supersets;
   };
   std::vector<Intersection> intersections;
};

/* for now, we maintain separate subsets for each
 * (robot/adof) specification;
 * also multi-dof joints with heterogeneous activeness not yet supported
 * base affine dofs not yet supported
 * 
 * in general,
 * 
 * input is either explicit names/tags,
 * or automatically determined;
 * 
 * output is many named subsets,
 * each of which with its checks to perform on the current environment
 * it if can be currently checked
 */
class ModuleSubsetManager : public OpenRAVE::ModuleBase
{
public:
   const OpenRAVE::EnvironmentBasePtr penv;
   
   // all modules in an environment must have a unique non-empty name
   // this is set on add
   std::string name;

   // subsets point to ilcs
   struct Subset
   {
      // the ilcs uniquely identifies the subset
      std::set< boost::shared_ptr<InterLinkCheck> > ilcs;
      
      // the set of live checks that can be performed to validate the ilcs
      // WHEN IS THIS UPDATED?
      // for now, updated by get_current_report()
      std::vector<struct LiveCheck> live_checks;
      
      // at some point we should do something to ensure
      // uniqueness of tags,
      // and facilitate lookups by tag
      std::string tag;
      
      // empty for base subsets;
      // a derived subset is the intersection of these bases
      std::set< boost::shared_ptr<Subset> > base_subsets;
      
      // empty for derived subsets (and some base subsets)
      // base subsets are pointed to by intersection by these deriveds
      std::set< boost::weak_ptr<Subset> > derived_subsets;
      
      bool operator==(const Subset & rhs) const
      {
         return (ilcs == rhs.ilcs);
      }
      bool operator<(const Subset & rhs) const
      {
         return (ilcs < rhs.ilcs);
      }
   };
   
   // base subsets MUST be disjoint!
   
   struct SpaceKey
   {
      std::string robot_name;
      std::vector<int> active_dofs;
      bool operator==(const SpaceKey & rhs) const
      {
         return robot_name == rhs.robot_name && active_dofs == rhs.active_dofs;
      }
      bool operator<(const SpaceKey & rhs) const
      {
         return robot_name < rhs.robot_name || (robot_name == rhs.robot_name && active_dofs < rhs.active_dofs);
      }
   };
   struct Space
   {
      OpenRAVE::RobotBaseWeakPtr robot;
      
      // subsets that upstream cares about,
      // so we keep pointers to them;
      // these are often derived subsets,
      // but can also point directly to base subsets
      std::set< boost::shared_ptr<Subset> > persistent_subsets;
      
      // subsets that have names
      std::map< std::string, boost::weak_ptr<Subset> > named_subsets;
   };
   std::map<SpaceKey, Space> spaces;
   
   double cost_per_ilc;
   
   OpenRAVE::CollisionCheckerBasePtr checker;
   
   // methods

   ModuleSubsetManager(OpenRAVE::EnvironmentBasePtr penv);
   virtual ~ModuleSubsetManager();
   
   // on environment add/remove
   virtual int main(const std::string & cmd);
   virtual void Destroy();
   
   bool GetName(std::ostream & sout, std::istream & sin);
   bool SetCostPerIlc(std::ostream & sout, std::istream & sin);
   // usage:
   // "TagCurrentSubset robotname tagname true|false"
   bool TagCurrentSubset(std::ostream & sout, std::istream & sin);
   bool DumpSubsets(std::ostream & sout, std::istream & sin);
   
   // space guaranteed to match passed robot
   // as long as we have a pointer to it
   Space & get_current_space(const OpenRAVE::RobotBasePtr robot);
   
   std::set< boost::shared_ptr<Subset> >
   get_existing_subsets(
      const Space & space);
   
   boost::shared_ptr<Subset>
   retrieve_subset(
      Space & space,
      const std::vector<InterLinkCheck> ilcs_vec,
      bool make_persistent);
   
   // tag current subset if it exists
   // and create it, if persistent
   void tag_current_subset(
      const OpenRAVE::RobotBasePtr robot,
      std::string new_tag,
      bool persistent);
   
   // temp function which simulates what the planner would call
   bool GetCurrentReport(std::ostream & sout, std::istream & sin);
   
   // given current active dofs
   // this will auto-tag if necessary
   // this will add persistency to this set!
   // this is primarily where names come from!
   // once a name comes out of this, it's persistent!
   void get_current_report(
      const OpenRAVE::RobotBasePtr robot,
      SubsetReport & report);
   
   void dump_subsets(const OpenRAVE::RobotBasePtr robot, std::string dotfile);
   
   // attempt to add a tag to a subset,
   // guarantees that the tag is unique within the space
   bool set_subset_tag(
      Space & space,
      boost::shared_ptr<Subset> subset,
      std::string new_tag);
   
   bool indicator(
      boost::shared_ptr<Subset> subset,
      OpenRAVE::RobotBasePtr robot,
      std::vector<OpenRAVE::dReal> & adofvals);
};

} // namespace or_multiset
