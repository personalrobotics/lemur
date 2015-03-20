/* File: inter_link_checks.h
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2014, 2015 Carnegie Mellon University
 * License: None
 */

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
   OpenRAVE::KinBody::LinkConstPtr link1;
   OpenRAVE::KinBody::LinkConstPtr link2;
   
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
// this clears ilcs
// for now, this DOES NOT return single link-link checks!
void compute_checks(
   const OpenRAVE::RobotBasePtr robot,
   std::vector<InterLinkCheck> & ilcs);




struct LiveCheck
{
   // all these checks are on the collision checker itself
   enum
   {
      TYPE_KINBODY,
      TYPE_KINBODY_KINBODY,
      TYPE_LINK,
      TYPE_LINK_LINK,
      TYPE_LINK_KINBODY,
      TYPE_SELFSA_KINBODY,
      TYPE_SELFSA_LINK
   } type;
   OpenRAVE::KinBodyPtr kinbody;
   OpenRAVE::KinBodyPtr kinbody_other;
   OpenRAVE::KinBody::LinkConstPtr link;
   OpenRAVE::KinBody::LinkConstPtr link_other;
   std::set<
      std::pair<OpenRAVE::KinBody::LinkConstPtr,OpenRAVE::KinBody::LinkConstPtr>
      > links_checked;
   bool is_self;
};

// this clears live_checks
// this uses whatever the robot's environment's collision checker's
// collision options are (e.g. CO_ActiveDOFs)
// this the robot's live checks are assumed to be
// CheckCollision(robot)
// CheckStandaloneSelfCollision(robot)
void compute_live_checks(
   const OpenRAVE::RobotBasePtr robot,
   std::vector<LiveCheck> & live_checks);





} // namespace or_multiset
