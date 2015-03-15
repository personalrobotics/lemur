
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
      boost::function<bool (std::vector<OpenRAVE::dReal> &)> indicator;
   };
   std::set<Subset> subsets;
   
   std::string current_subset;
   
   struct Inclusion
   {
      std::string subset;
      std::string superset;
   };
   std::set<Inclusion> inclusions;
   
   struct Intersection
   {
      std::string subset;
      std::vector<std::string> supersets;
   };
   std::set<Intersection> intersections;
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

   // subsets point to ilcs
   struct Subset
   {
      // the ilcs uniquely identifies the subset
      std::set< boost::shared_ptr<InterLinkCheck> > ilcs;
      
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
   
   // subsets that upstream cares about,
   // so we keep pointers to them;
   // these are often derived subsets,
   // but can also point directly to base subsets
   std::set< boost::shared_ptr<Subset> > persistent_subsets;
   
   // methods

   ModuleSubsetManager(OpenRAVE::EnvironmentBasePtr penv);
   virtual ~ModuleSubsetManager();
   
   // on environment add/remove
   virtual int main(const std::string& cmd);
   virtual void Destroy();
   
   // usage:
   // "TagCurrentSubset robotname tagname true|false"
   bool TagCurrentSubset(std::ostream & sout, std::istream & sin);

   
   // tag current subset if it exists
   // and create it, if persistent
   void tag_current_subset(
      const OpenRAVE::RobotBasePtr robot,
      std::string new_tag,
      bool persistent);
   
   // given current active dofs
   // this will auto-tag if necessary
   // this will add persistency to this set!
   // this is primarily where names come from!
   // once a name comes out of this, it's persistent!
   void get_report(
      const OpenRAVE::RobotBasePtr robot,
      SubsetReport & report);
};

} // namespace or_multiset
