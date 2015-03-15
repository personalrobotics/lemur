
namespace or_multiset
{

bool fuzzy_equals(const OpenRAVE::Transform & tx1, const OpenRAVE::Transform & tx2, OpenRAVE::dReal fuzz);

struct TxAjoint
{
   OpenRAVE::Transform tx;
   OpenRAVE::KinBody::JointPtr ajoint;
   bool operator==(const TxAjoint & rhs) const
   {
      if (!fuzzy_equals(tx, rhs.tx, 0.000001)) return false;
      if (ajoint != rhs.ajoint) return false;
      return true;
   }
};

// check between something on the robot (a link or grabbed body)
// and something in the environment
struct InterLinkCheck
{
   // two links, with link1 < link2
   OpenRAVE::KinBody::LinkPtr link1;
   OpenRAVE::KinBody::LinkPtr link2;
   
   // common prefixes removed, first link1 tx is guaranteed identity
   std::vector<TxAjoint> link1_path;
   std::vector<TxAjoint> link2_path;
   
   bool operator==(const InterLinkCheck & rhs) const
   {
      if (link1 != rhs.link1) return false;
      if (link2 != rhs.link2) return false;
      if (link1_path != rhs.link1_path) return false;
      if (link2_path != rhs.link2_path) return false;
      return true;
   }
};

// for this robot's current active dofs
void compute_checks(
   const OpenRAVE::RobotBasePtr robot,
   std::vector<InterLinkCheck> & ilcs);

} // namespace or_multiset
