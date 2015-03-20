#include <algorithm> // std::set_intersection
#include <openrave/openrave.h>
#include <or_multiset/inter_link_checks.h>

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

void or_multiset::compute_checks(
   const OpenRAVE::RobotBasePtr robot,
   std::vector<InterLinkCheck> & ilcs)
{
   ilcs.clear();
   
   bool success;
   
   // get the list of all robots in the environment
   // (is this necessary??)
   std::vector<OpenRAVE::RobotBasePtr> robots;
   robot->GetEnv()->GetRobots(robots);
   
   // get non-adjacent links
   std::set<int> non_adjacent_links = robot->GetNonAdjacentLinks(0);
   
   // get active dofs
   std::vector<int> adofs = robot->GetActiveDOFIndices();
   
   // get set of active joints (joints which contain active dofs)
   std::set<OpenRAVE::KinBody::JointPtr> ajoints;
   {
      for (std::vector<int>::const_iterator adof=adofs.begin(); adof!=adofs.end(); adof++)
         ajoints.insert(robot->GetJointFromDOFIndex(*adof));
      for (std::set<OpenRAVE::KinBody::JointPtr>::iterator ajoint=ajoints.begin(); ajoint!=ajoints.end(); ajoint++)
      {
         for (int i=0; i<(*ajoint)->GetDOF(); i++)
         {
            int dof = (*ajoint)->GetDOFIndex() + i;
            if (std::find(adofs.begin(), adofs.end(), dof) != adofs.end())
               continue;
            RAVELOG_WARN("Active joint %s includes non-active DOF %d!\n", (*ajoint)->GetName().c_str(), dof);
         }
      }
   }
   
   // get a list of all enabled links in the environment
   std::vector<OpenRAVE::KinBody::LinkConstPtr> links;
   {
      std::vector<OpenRAVE::KinBodyPtr> ks;
      robot->GetEnv()->GetBodies(ks);
      for (std::vector<OpenRAVE::KinBodyPtr>::iterator k=ks.begin(); k!=ks.end(); k++)
      {
         const std::vector<OpenRAVE::KinBody::LinkPtr> klinks = (*k)->GetLinks();
         for (std::vector<OpenRAVE::KinBody::LinkPtr>::const_iterator klink=klinks.begin(); klink!=klinks.end(); klink++)
         if ((*klink)->IsEnabled())
            links.push_back(*klink);
      }
   }
   
   // for each link, get its path to the environment root
   std::map<OpenRAVE::KinBody::LinkConstPtr, std::vector<TxAjoint> > link_paths;
   {
      for (std::vector<OpenRAVE::KinBody::LinkConstPtr>::iterator link_orig=links.begin(); link_orig!=links.end(); link_orig++)
      {
         std::vector<TxAjoint> link_path; // we'll fill this
         
         // start pointing to the original target link, with no sucessor joint
         // these will eventually be prepended to link_path
         OpenRAVE::KinBody::LinkConstPtr link_target = *link_orig;
         OpenRAVE::KinBody::JointPtr joint_target; // null
         
         // iteravely go backwards up the link chain to the environment root
         // on the way, we'll look for active joints
         for (OpenRAVE::KinBody::LinkConstPtr link = link_target; link; )
         {
#if 0
            printf("considering link %s:%s ...\n",
               link->GetParent()->GetName().c_str(),
               link->GetName().c_str());
#endif
            
            // find parent link
            OpenRAVE::KinBody::LinkConstPtr link_parent;
            
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
               std::vector<OpenRAVE::KinBody::LinkConstPtr> links_grabbing;
               for (std::vector<OpenRAVE::RobotBasePtr>::iterator robot=robots.begin(); robot!=robots.end(); robot++)
               {
                  OpenRAVE::KinBody::LinkConstPtr link_grabbing = (*robot)->IsGrabbing(link->GetParent());
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
   for (std::vector<OpenRAVE::KinBody::LinkConstPtr>::iterator link1=links.begin(); link1!=links.end(); link1++)
   for (std::vector<OpenRAVE::KinBody::LinkConstPtr>::iterator link2=links.begin(); link2!=links.end(); link2++)
   {
      // ensure link1 < link2
      if (!((*link1).get() < (*link2).get()))
         continue;
      
      // if they're both robot links, ensure they're nonadjacent!
      if ((*link1)->GetParent() == robot && (*link2)->GetParent() == robot)
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
      ilcs.push_back(ilc);
   }
}


/*
struct LiveCheck
{
   enum
   {
      TYPE_KINBODY,
      TYPE_KINBODY_KINBODY,
      TYPE_LINK,
      TYPE_LINK_KINBODY,
      TYPE_SELF_KINBODY,
      TYPE_SELF_LINK
   } type;
   OpenRAVE::KinBody kinbody;
   OpenRAVE::KinBody kinbody_other;
   OpenRAVE::KinBody::LinkPtr link;
};
*/
void or_multiset::compute_live_checks(
   const OpenRAVE::RobotBasePtr robot,
   std::vector<LiveCheck> & live_checks)
{
   live_checks.clear();
   
   int collision_options = robot->GetEnv()->GetCollisionChecker()->GetCollisionOptions();

   std::vector<bool> robot_active_links;
   if ((collision_options&OpenRAVE::CO_ActiveDOFs) && !robot->GetAffineDOF())
   {
      // get active joint indices
      std::set<int> active_joints;
      const std::vector<int> & adofs = robot->GetActiveDOFIndices();
      for (unsigned int di=0; di<adofs.size(); di++)
         active_joints.insert(robot->GetJointFromDOFIndex(adofs[di])->GetJointIndex());
      // set active links
      robot_active_links.resize(robot->GetLinks().size(), false);
      for (unsigned int li=0; li<robot_active_links.size(); li++)
         for (std::set<int>::iterator ji=active_joints.begin(); ji!=active_joints.end(); ji++)
            if (robot->DoesAffect(*ji, li))
               robot_active_links[li] = true;
   }
   else
      robot_active_links.resize(robot->GetLinks().size(), true);
   
   // CheckCollision(kb)
   // checks between all pairwise links between
   // all my attached bodies with all non-attached bodies
   // for now, just with kb=robot
   // and respect CO_ActiveDOFs if its set
   {
      or_multiset::LiveCheck lc;
      lc.type = or_multiset::LiveCheck::TYPE_KINBODY;
      lc.kinbody = robot;
      std::vector<OpenRAVE::KinBodyPtr> kinbodies;
      robot->GetEnv()->GetBodies(kinbodies);
      std::set<OpenRAVE::KinBody::LinkConstPtr> links_me;
      std::set<OpenRAVE::KinBody::LinkConstPtr> links_other;
      for (std::vector<OpenRAVE::KinBodyPtr>::iterator
         kb=kinbodies.begin(); kb!=kinbodies.end(); kb++)
      {
         bool kb_is_attached = robot->IsAttached(*kb);
         const std::vector<OpenRAVE::KinBody::LinkPtr> & kb_links = (*kb)->GetLinks();
         for (std::vector<OpenRAVE::KinBody::LinkPtr>::const_iterator
            link=kb_links.begin(); link!=kb_links.end(); link++)
         {
            if (kb_is_attached)
            {
               // skip if active
               if ((*kb) == robot)
               {
                  if (!robot_active_links[(*link)->GetIndex()])
                     continue;
               }
               else
               {
                  OpenRAVE::KinBody::LinkPtr robot_link = robot->IsGrabbing(*kb);
                  if (robot_link && !robot_active_links[robot_link->GetIndex()])
                     continue;
               }
               links_me.insert(*link);
            }
            else
               links_other.insert(*link);
         }
      }
      for (std::set<OpenRAVE::KinBody::LinkConstPtr>::iterator
         link1=links_me.begin(); link1!=links_me.end(); link1++)
      for (std::set<OpenRAVE::KinBody::LinkConstPtr>::iterator
         link2=links_other.begin(); link2!=links_other.end(); link2++)
      {
         if ((*link1).get() < (*link2).get())
            lc.links_checked.insert(std::make_pair(*link1, *link2));
         else
            lc.links_checked.insert(std::make_pair(*link2, *link1));
      }
      live_checks.push_back(lc);
   }
   
   // CheckCollision(link)
   // each robot link
   // checks against the rest of the environment,
   // but not grabbed bodies?
   {
      or_multiset::LiveCheck lc;
      lc.type = or_multiset::LiveCheck::TYPE_LINK;
      
      // get all enabled links not attached to the robot
      std::set<OpenRAVE::KinBody::LinkConstPtr> links_other;
      std::vector<OpenRAVE::KinBodyPtr> kinbodies;
      robot->GetEnv()->GetBodies(kinbodies);
      for (std::vector<OpenRAVE::KinBodyPtr>::iterator
         kb=kinbodies.begin(); kb!=kinbodies.end(); kb++)
      {
         if (robot->IsAttached(*kb))
            continue;
         const std::vector<OpenRAVE::KinBody::LinkPtr> & kb_links = (*kb)->GetLinks();
         for (std::vector<OpenRAVE::KinBody::LinkPtr>::const_iterator
            link=kb_links.begin(); link!=kb_links.end(); link++)
         {
            if ((*link)->IsEnabled())
               links_other.insert(*link);
         }
      }
      // get all robot links, but only ENABLED links that are ACTIVE
      const std::vector<OpenRAVE::KinBody::LinkPtr> & my_links = robot->GetLinks();
      for (unsigned int li=0; li<my_links.size(); li++)
      {
         if (!robot_active_links[li])
            continue;
         if (!my_links[li]->IsEnabled())
            continue;
         lc.link = my_links[li];
         lc.links_checked.clear();
         for (std::set<OpenRAVE::KinBody::LinkConstPtr>::iterator
            link_other=links_other.begin(); link_other!=links_other.end(); link_other++)
         {
            if ((my_links[li]).get() < (*link_other).get())
               lc.links_checked.insert(std::make_pair(my_links[li], *link_other));
            else
               lc.links_checked.insert(std::make_pair(*link_other, my_links[li]));
         }
         live_checks.push_back(lc);
      }
   }
   
   // CheckStandaloneSelfCollision(robot)
   {
      or_multiset::LiveCheck lc;
      lc.type = or_multiset::LiveCheck::TYPE_SELFSA_KINBODY;
      lc.kinbody = robot;
      int adjacentoptions = OpenRAVE::KinBody::AO_Enabled;
      if (collision_options & OpenRAVE::CO_ActiveDOFs)
         adjacentoptions |= OpenRAVE::KinBody::AO_ActiveDOFs;
      const std::set<int>& nonadjacent = robot->GetNonAdjacentLinks(adjacentoptions);
      for (std::set<int>::iterator it=nonadjacent.begin(); it!=nonadjacent.end(); it++)
      {
         OpenRAVE::KinBody::LinkConstPtr link1(robot->GetLinks().at(*it&0xffff));
         OpenRAVE::KinBody::LinkConstPtr link2(robot->GetLinks().at(*it>>16));
         if (!link1->IsEnabled())
            continue;
         if (!link2->IsEnabled())
            continue;
         // ensure link1 < link2
         if (!((link1.get()) < (link2.get())))
            link1.swap(link2);
         lc.links_checked.insert(std::make_pair(link1, link2));
      }
      live_checks.push_back(lc);
   }
}
