/*! \file module_family.cpp
 * \author Chris Dellin <cdellin@gmail.com>
 * \copyright 2015 Carnegie Mellon University
 * \copyright License: BSD
 */

#include <algorithm>
#include <iostream>

#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>

#include <openrave/openrave.h>
#include <openrave/utils.h>

#include <ompl_lemur/util.h>
#include <or_lemur/parse_args.h>
#include <or_lemur/module_family.h>
#include <or_lemur/link_indicators.h>


or_lemur::FamilyModule::FamilyModule(OpenRAVE::EnvironmentBasePtr penv):
   OpenRAVE::ModuleBase(penv),
   _initialized(false),
   _has_use_baked_checker(false),
   _cost_per_ilc(60.0e-6)
{
   RegisterCommand("GetInstanceId",boost::bind(&or_lemur::FamilyModule::CmdGetInstanceId,this,_1,_2),"GetInstanceId");
   RegisterCommand("GetRobotName",boost::bind(&or_lemur::FamilyModule::CmdGetRobotName,this,_1,_2),"GetRobotName");
   RegisterCommand("SetCostPerIlc",boost::bind(&or_lemur::FamilyModule::CmdSetCostPerIlc,this,_1,_2),"SetCostPerIlc");
   RegisterCommand("GetFamilyId",boost::bind(&or_lemur::FamilyModule::CmdGetFamilyId,this,_1,_2),"GetFamilyId");
   RegisterCommand("Let",boost::bind(&or_lemur::FamilyModule::CmdLet,this,_1,_2),"Let");
   RegisterCommand("NamesOf",boost::bind(&or_lemur::FamilyModule::CmdNamesOf,this,_1,_2),"NamesOf");
   RegisterCommand("PrintCurrentFamily",boost::bind(&or_lemur::FamilyModule::CmdPrintCurrentFamily,this,_1,_2),"PrintCurrentFamily");
   RegisterCommand("GetHeaderFromSet",boost::bind(&or_lemur::FamilyModule::CmdGetHeaderFromSet,this,_1,_2),"GetHeaderFromSet");
   RAVELOG_INFO("Constructed family \"%s\".\n", GetInstanceId().c_str());
}


or_lemur::FamilyModule::~FamilyModule()
{
   RAVELOG_INFO("Destructed family \"%s\".\n", GetInstanceId().c_str());
}


std::string or_lemur::FamilyModule::GetInstanceId()
{
   std::stringstream ss;
   ss << this;
   return ss.str();
}


void or_lemur::FamilyModule::SetCostPerIlc(double cost_per_ilc)
{
   _cost_per_ilc = cost_per_ilc;
}


int or_lemur::FamilyModule::main(const std::string & cmd)
{
   // ensure we're not yet initialized
   if (_initialized)
   {
      RAVELOG_ERROR("Family object already initialized!\n");
      throw OpenRAVE::openrave_exception("Family object already initialized");
   }
   
   // arguments
   boost::program_options::options_description desc;
   desc.add_options()
      ("help", "produce help message")
      ("robot-name", boost::program_options::value<std::string>(), "robot name in environment")
      ("use-baked-checker", boost::program_options::value<bool>(), "whether to used a baked checker");
   boost::program_options::variables_map args;
   try
   {
      args = or_lemur::parse_args(desc, cmd);
   }
   catch (const boost::program_options::error & ex)
   {
      RAVELOG_ERROR("Argument error: %s\n", ex.what());
      OR_LEMUR_PRINT_USAGE(desc);
      throw OpenRAVE::openrave_exception("Argument error");
   }
   
   if (args.count("help"))
   {
      OR_LEMUR_PRINT_USAGE(desc);
      return 1;
   }
   
   // parse robot
   OpenRAVE::RobotBasePtr robot;
   if (args.count("robot-name") != 1)
   {
      RAVELOG_ERROR("--robot-name must be passed!\n");
      OR_LEMUR_PRINT_USAGE(desc);
      throw OpenRAVE::openrave_exception("--robot-name must be passed");
   }
   robot = GetEnv()->GetRobot(args["robot-name"].as<std::string>());
   if (!robot)
   {
      RAVELOG_ERROR("--robot-name \"%s\" not found!\n", args["robot-name"].as<std::string>().c_str());
      throw OpenRAVE::openrave_exception("--robot-name not found");
   }
   
   // get robot's current active dofs
   std::vector<int> active_dofs = robot->GetActiveDOFIndices();
   
   // compose this family's id
   std::string id;
   // serialize kinematics hash
   {
      std::stringstream ss;
      robot->serialize(ss, OpenRAVE::SO_Kinematics);
      id += OpenRAVE::utils::GetMD5HashString(ss.str());
   }
   // serialize active dofs
   for (unsigned int ui=0; ui<active_dofs.size(); ui++)
      id += ompl_lemur::util::sf(",%d", active_dofs[ui]);
   
   RAVELOG_INFO("Family id: \"%s\".\n", id.c_str());
   
   // get active joints
   std::set<OpenRAVE::KinBody::JointPtr> active_joints;
   for (unsigned int ui=0; ui<active_dofs.size(); ui++)
      active_joints.insert(robot->GetJointFromDOFIndex(active_dofs[ui]));
   
   // compute proxidxs (map from robot link index -> proximal rigid link index)
   std::vector<int> proxidxs(robot->GetLinks().size());
   for (int idx=0; idx<(int)proxidxs.size(); idx++)
   {
      OpenRAVE::KinBody::LinkPtr proxlink = robot->GetLinks()[idx];
      for (;;)
      {
         // get parent link
         std::vector<OpenRAVE::KinBody::LinkPtr> parents;
         proxlink->GetParentLinks(parents);
         if (parents.size() == 0)
            break;
         if (parents.size() != 1)
         {
            RAVELOG_ERROR("Link [%d] \"%s\" has %lu parents!\n",
               proxlink->GetIndex(), proxlink->GetName().c_str(), parents.size());
            throw OpenRAVE::openrave_exception("Link has multiple parents");
         }
         
         // if we are the child of an active joint, then we're done
         std::set<OpenRAVE::KinBody::JointPtr>::iterator it;
         for (it=active_joints.begin(); it!=active_joints.end(); it++)
            if ((*it)->GetHierarchyChildLink() == proxlink)
               break;
         if (it!=active_joints.end())
            break;
         // consider parent link instead
         proxlink = parents[0];
      }
      
      proxidxs[idx] = proxlink->GetIndex();
   }
   
   // parse use-baked-checker
   if (args.count("use-baked-checker") == 1)
   {
      _use_baked_checker = args["use-baked-checker"].as<bool>();
      _has_use_baked_checker = true;
   }
   
   // save values
   _robot = robot;
   _active_dofs = active_dofs;
   _id = id;
   _proxidxs = proxidxs;
   _initialized = true;
   return 0;
}


void or_lemur::FamilyModule::Destroy()
{
}


std::string or_lemur::FamilyModule::GetFamilyId()
{
   return _id;
}


// can return an empty pointer
or_lemur::FamilyModule::SetPtr
or_lemur::FamilyModule::GetCurrentSet()
{
   if (!_initialized)
   {
      RAVELOG_ERROR("Family object not initialized!\n");
      throw OpenRAVE::openrave_exception("Family object not initialized");
   }
   OpenRAVE::RobotBasePtr robot = _robot.lock();
   if (!robot)
   {
      RAVELOG_ERROR("Robot not found!\n");
      throw OpenRAVE::openrave_exception("Robot not found");
   }
   
   // collect all pairs of links that should be currently checked
   
   // step 1: attach all enabled links in the environment to a robot proxlink
   // note that links wither earlier indices are always added first
   std::vector< std::pair<int,OpenRAVE::KinBody::LinkPtr> > links;
   
   {
      std::vector<OpenRAVE::KinBodyPtr> kinbodies;
      GetEnv()->GetBodies(kinbodies);
      for (unsigned int ki=0; ki<kinbodies.size(); ki++)
      {
         // ignore disabled bodies
         if (!kinbodies[ki]->IsEnabled())
            continue;
         if (kinbodies[ki] == robot) // robot itself
         {
            const std::vector<OpenRAVE::KinBody::LinkPtr> & robot_links = robot->GetLinks();
            for (int li=0; li<(int)robot_links.size(); li++)
            {
               if (!robot_links[li]->IsEnabled())
                  continue;
               links.push_back(std::make_pair(_proxidxs[li],robot_links[li]));
            }
         }
         else // some other kinbody
         {
            int proxidx;
            OpenRAVE::KinBody::LinkPtr grabbinglink = robot->IsGrabbing(kinbodies[ki]);
            if (!grabbinglink)
               proxidx = 0;
            else
               proxidx = _proxidxs[grabbinglink->GetIndex()];
            // find all kinbody links
            const std::vector<OpenRAVE::KinBody::LinkPtr> & kb_links = kinbodies[ki]->GetLinks();
            for (int li=0; li<(int)kb_links.size(); li++)
            {
               if (!kb_links[li]->IsEnabled())
                  continue;
               links.push_back(std::make_pair(proxidx,kb_links[li]));
            }
         }
      }
   }
   
   // step 2: list all pairs of checks
   std::vector< std::pair<
      std::pair<int,OpenRAVE::KinBody::LinkPtr>,
      std::pair<int,OpenRAVE::KinBody::LinkPtr> > > link_checks;
   std::set<int> nonadj = robot->GetNonAdjacentLinks(
      OpenRAVE::KinBody::AO_Enabled | OpenRAVE::KinBody::AO_ActiveDOFs);
   for (unsigned int li1=0; li1<links.size(); li1++)
   for (unsigned int li2=li1+1; li2<links.size(); li2++)
   {
      // skip checks on the same proxlink
      if (links[li1].first == links[li2].first)
         continue;
      // if both links are part of the robot,
      // check for adjacency
      if (links[li1].second->GetParent()==robot && links[li2].second->GetParent()==robot)
      {
         int idx1 = links[li1].second->GetIndex();
         int idx2 = links[li2].second->GetIndex();
         int key = idx1 | (idx2<<16);
         if (!nonadj.count(key))
            continue; // adjacent!
      }
      // add
      link_checks.push_back(std::make_pair(links[li1],links[li2]));
   }
   
   // step 3: collect all links involved in these checks
   // links_used will be a subset of links
   // this points to the posedlink data structure (initially null since not found)
   std::map< std::pair<int,OpenRAVE::KinBody::LinkPtr>, PosedLink * > links_used;
   for (unsigned int ci=0; ci<link_checks.size(); ci++)
   {
      links_used.insert(std::make_pair(link_checks[ci].first, (PosedLink *)0));
      links_used.insert(std::make_pair(link_checks[ci].second, (PosedLink *)0));
   }
   
   // ok, ensure that all posedlinks are in our datastructure
   // this is where the deduplication happens
   for (std::map< std::pair<int,OpenRAVE::KinBody::LinkPtr>, PosedLink * >::iterator
      it=links_used.begin(); it!=links_used.end(); it++)
   {
      // get link geom hash
      std::stringstream ss;
      it->first.second->serialize(ss, OpenRAVE::SO_Geometry);
      std::string geomhash = OpenRAVE::utils::GetMD5HashString(ss.str());
      
      // get link from-prox pose
      int proxidx = it->first.first;
      OpenRAVE::Transform tx_prox = robot->GetLinks()[proxidx]->GetTransform();
      OpenRAVE::Transform tx_link = it->first.second->GetTransform();
      OpenRAVE::Transform tx_prox_link = tx_prox.inverse() * tx_link;
      
      // look up (proxidx,geomhash) in index
      std::map< std::pair<int,std::string>, std::set<PosedLink *> >::iterator
         indexkey = _posedlinks.find(std::make_pair(proxidx,geomhash));
      if (indexkey == _posedlinks.end())
         indexkey = _posedlinks.insert(std::make_pair(
            std::make_pair(proxidx,geomhash), std::set<PosedLink *>())).first;
      
      // search for matching posedlink object
      std::set<PosedLink *>::iterator needle;
      for (needle=indexkey->second.begin(); needle!=indexkey->second.end(); needle++)
      {
         OpenRAVE::Transform tx_rel = tx_prox_link.inverse() * (*needle)->pose;
         // is this different?
         if (tx_rel.trans.lengthsqr3() > 0.0000001)
            continue;
         OpenRAVE::dReal quat_xxyyzz = 0.0;
         quat_xxyyzz += tx_rel.rot.y * tx_rel.rot.y; // qx
         quat_xxyyzz += tx_rel.rot.z * tx_rel.rot.z; // qy
         quat_xxyyzz += tx_rel.rot.w * tx_rel.rot.w; // qz
         if (quat_xxyyzz > 0.0000001)
            continue;
         // the same!
         //printf("found already!\n");
         break;
      }
      
      // if we didn't find it, allocate a new one here!
      if (needle == indexkey->second.end())
      {
         //printf("not found already!\n");
         PosedLink * posedlink = new PosedLink;
         posedlink->proxidx = proxidx;
         posedlink->geomhash = geomhash;
         posedlink->pose = tx_prox_link;
         needle = indexkey->second.insert(posedlink).first;
      }
      
      // set links_used pointer
      it->second = *needle;
   }
   
   // get checks required for this set
   std::set<Check> checks;
   for (unsigned int ci=0; ci<link_checks.size(); ci++)
   {
      PosedLink * posedlink1 = links_used[link_checks[ci].first];
      PosedLink * posedlink2 = links_used[link_checks[ci].second];
      if (posedlink1 < posedlink2)
         checks.insert(std::make_pair(posedlink1,posedlink2));
      else
         checks.insert(std::make_pair(posedlink2,posedlink1));
   }
   
   return set_from_checks(checks);
}


or_lemur::FamilyModule::SetPtr
or_lemur::FamilyModule::GetSetFromExpression(std::string input)
{
   // parse input into tokens
   std::string delims = "()\\&";
   boost::tokenizer< boost::char_separator<char> > tokenizer(
      input, boost::char_separator<char>(" ", delims.c_str()));
   std::vector<std::string> strtokens(tokenizer.begin(),tokenizer.end());
   
   // process right-hand-side strings into bound tokens
   // first: operator (or 0 if none); second: bound set
   std::list< std::pair<char, SetPtr> > tokens;
   for (unsigned int ui=0; ui<strtokens.size(); ui++)
   {
      if (strtokens[ui].size()==1 && delims.find(strtokens[ui][0])!=std::string::npos)
      {
         // operator
         tokens.push_back(std::make_pair(strtokens[ui][0],SetPtr()));
      }
      else if (strtokens[ui][0] == '$')
      {
         if (strtokens[ui] == "$live")
         {
            tokens.push_back(std::make_pair(0,GetCurrentSet()));
         }
         else
         {
            RAVELOG_ERROR("Expression syntax: Unknown function \"%s\"\n", strtokens[ui].c_str());
            throw OpenRAVE::openrave_exception("Expression syntax: Unknown function");
         }
      }
      else
      {
         std::map<std::string, SetPtr>::iterator
            needle = _sets_bound.find(strtokens[ui]);
         if (needle == _sets_bound.end())
         {
            RAVELOG_ERROR("Expression syntax: Unknown literal \"%s\"\n", strtokens[ui].c_str());
            throw OpenRAVE::openrave_exception("Expression syntax: Unknown literal");
         }
         tokens.push_back(std::make_pair(0,needle->second));
      }
   }
   
   // process rhs
   RAVELOG_WARN("Warning: Skipping expression simplification ...\n");
   
   // check that remaining rhs is a single set,
   // and make the new binding
   if (tokens.size() != 1 || !tokens.front().second)
   {
      RAVELOG_ERROR("Expression syntax: Error evaluating expression.\n");
      throw OpenRAVE::openrave_exception("Expression syntax: Error evaluating expression");
   }
   
   return tokens.front().second;
}


or_lemur::FamilyModule::SetPtr
or_lemur::FamilyModule::GetSet(std::string literal)
{
   std::map<std::string, SetPtr>::iterator
      needle = _sets_bound.find(literal);
   if (needle == _sets_bound.end())
      return SetPtr();
   return needle->second;
}


void or_lemur::FamilyModule::Let(std::string literal, or_lemur::FamilyModule::SetPtr set)
{
   std::map<std::string, SetPtr>::iterator needle;
   needle = _sets_bound.find(literal);
   if (needle != _sets_bound.end())
      needle->second->literals.erase(literal);
   _sets_bound[literal] = set;
   set->literals.insert(literal);
}


void or_lemur::FamilyModule::Del(std::string literal)
{
   std::map<std::string, SetPtr>::iterator needle;
   needle = _sets_bound.find(literal);
   if (needle == _sets_bound.end())
   {
      RAVELOG_ERROR("Literal \"%s\" not found!\n", literal.c_str());
      throw OpenRAVE::openrave_exception("Literal not found");
   }
   needle->second->literals.erase(literal);
   _sets_bound.erase(needle);
}


std::string
or_lemur::FamilyModule::GetHeaderFromSet(or_lemur::FamilyModule::SetPtr set)
{
   std::stringstream ret;
   
   ret << "family " << _id << "\n";
   
   // collect all posedlinks needed by checks in this set
   std::set<PosedLink *> posedlinks;
   for (std::set<Check>::iterator
      it=set->checks.begin(); it!=set->checks.end(); it++)
   {
      posedlinks.insert(it->first);
      posedlinks.insert(it->second);
   }
   
   // compute the string version of each
   std::map<std::string, PosedLink *> posedlink_sorted;
   for (std::set<PosedLink *>::iterator
      it=posedlinks.begin(); it!=posedlinks.end(); it++)
   {
      std::stringstream ss;
      ss << (*it)->proxidx
         << " " << (*it)->geomhash
         << " " << (*it)->pose.trans.x
         << " " << (*it)->pose.trans.y
         << " " << (*it)->pose.trans.z
         << " " << (*it)->pose.rot.y // qx
         << " " << (*it)->pose.rot.z // qy
         << " " << (*it)->pose.rot.w // qz
         << " " << (*it)->pose.rot.x; // qw
      posedlink_sorted.insert(std::make_pair(ss.str(), *it));
   }
   
   // first part of header -- posedlinks (also assign indices)
   ret << "num_posedlinks " << posedlink_sorted.size() << "\n";
   std::map<PosedLink *, size_t> posedlink_indices;
   for (std::map<std::string, PosedLink *>::iterator
      it=posedlink_sorted.begin(); it!=posedlink_sorted.end(); it++)
   {
      size_t idx = posedlink_indices.size();
      ret << "posedlink " << idx << " " << it->first << "\n";
      posedlink_indices.insert(std::make_pair(it->second, idx));
   }
   
   // collect and print all checks
   std::set< std::pair<size_t,size_t> > idx_checks;
   for (std::set<Check>::iterator
      it=set->checks.begin(); it!=set->checks.end(); it++)
   {
      size_t idx1 = posedlink_indices[it->first];
      size_t idx2 = posedlink_indices[it->second];
      if (idx1 < idx2)
         idx_checks.insert(std::make_pair(idx1,idx2));
      else
         idx_checks.insert(std::make_pair(idx2,idx1));
   }
   ret << "checks " << idx_checks.size();
   for (std::set< std::pair<size_t,size_t> >::iterator
      it=idx_checks.begin(); it!=idx_checks.end(); it++)
   {
      ret << " " << it->first << "x" << it->second;
   }
   ret << "\n";
   
   return ret.str();
}


or_lemur::FamilyModule::SetPtr
or_lemur::FamilyModule::GetSetFromHeader(std::string set_header)
{
   std::stringstream ss(set_header);
   std::string str;
   ss >> str;
   if (str != "family")
   {
      RAVELOG_ERROR("Could not find family id!\n");
      return or_lemur::FamilyModule::SetPtr();
   }
   ss >> str;
   if (str != _id)
   {
      RAVELOG_ERROR("Family id mismatch:\n");
      RAVELOG_ERROR("  family has id: \"%s\"\n", _id.c_str());
      RAVELOG_ERROR("  header has id: \"%s\"\n", str.c_str());
      return or_lemur::FamilyModule::SetPtr();
   }
   
   size_t num_posedlinks;
   ss >> str;
   if (str != "num_posedlinks")
   {
      RAVELOG_ERROR("Parse error!\n");
      return or_lemur::FamilyModule::SetPtr();
   }
   ss >> num_posedlinks;
   
   // read each posed link
   // this will create a new posedlink if necessary
   std::vector<PosedLink *> posedlinks;
   for (size_t idx=0; idx<num_posedlinks; idx++)
   {
      size_t read_idx;
      size_t proxidx;
      std::string geomhash;
      double x, y, z, qx, qy, qz, qw;
      ss >> str >> read_idx >> proxidx >> geomhash >> x >> y >> z >> qx >> qy >> qz >> qw;
      if (str != "posedlink" || read_idx != idx)
      {
         // TODO: delete allocated posedlinks!
         RAVELOG_ERROR("posedlink parse error!\n");
         return or_lemur::FamilyModule::SetPtr();
      }
      OpenRAVE::Transform tx_prox_link(OpenRAVE::Vector(qw, qx, qy, qz), OpenRAVE::Vector(x, y, z));
      
      // TODO: the following is duplicated with GetCurrentSet() implementation
      
      // look up (proxidx,geomhash) in index
      std::map< std::pair<int,std::string>, std::set<PosedLink *> >::iterator
         indexkey = _posedlinks.find(std::make_pair(proxidx,geomhash));
      if (indexkey == _posedlinks.end())
         indexkey = _posedlinks.insert(std::make_pair(
            std::make_pair(proxidx,geomhash), std::set<PosedLink *>())).first;
      
      // search for matching posedlink object
      std::set<PosedLink *>::iterator needle;
      for (needle=indexkey->second.begin(); needle!=indexkey->second.end(); needle++)
      {
         OpenRAVE::Transform tx_rel = tx_prox_link.inverse() * (*needle)->pose;
         // is this different?
         if (tx_rel.trans.lengthsqr3() > 0.0000001)
            continue;
         OpenRAVE::dReal quat_xxyyzz = 0.0;
         quat_xxyyzz += tx_rel.rot.y * tx_rel.rot.y; // qx
         quat_xxyyzz += tx_rel.rot.z * tx_rel.rot.z; // qy
         quat_xxyyzz += tx_rel.rot.w * tx_rel.rot.w; // qz
         if (quat_xxyyzz > 0.0000001)
            continue;
         // the same!
         break;
      }
      
      // if we didn't find it, allocate a new one here!
      if (needle == indexkey->second.end())
      {
         PosedLink * posedlink = new PosedLink;
         posedlink->proxidx = proxidx;
         posedlink->geomhash = geomhash;
         posedlink->pose = tx_prox_link;
         needle = indexkey->second.insert(posedlink).first;
      }
      
      posedlinks.push_back(*needle);
   }
   
   // next, read checks
   size_t num_checks;
   ss >> str >> num_checks;
   if (str != "checks")
   {
      // TODO: delete allocated posedlinks!
      RAVELOG_ERROR("checks parse error!\n");
      return or_lemur::FamilyModule::SetPtr();
   }
   std::set<Check> checks;
   for (size_t ic=0; ic<num_checks; ic++)
   {
      ss >> str;
      const char * s = str.c_str();
      char * end;
      long int idx1 = strtol(s, &end, 10);
      if (!(s<end) || *end != 'x')
      {
         // TODO: delete allocated posedlinks!
         RAVELOG_ERROR("checks parse error!\n");
         return or_lemur::FamilyModule::SetPtr();
      }
      long int idx2 = strtol(end+1, &end, 10);
      if ((end-s) != (int)str.size())
      {
         // TODO: delete allocated posedlinks!
         RAVELOG_ERROR("checks parse error!\n");
         return or_lemur::FamilyModule::SetPtr();
      }
      PosedLink * posedlink1 = posedlinks[idx1];
      PosedLink * posedlink2 = posedlinks[idx2];
      if (posedlink1 < posedlink2)
         checks.insert(std::make_pair(posedlink1,posedlink2));
      else
         checks.insert(std::make_pair(posedlink2,posedlink1));
   }
   
   return set_from_checks(checks);
}


or_lemur::FamilyModule::Family
or_lemur::FamilyModule::GetCurrentFamily(std::set<SetPtr> sets_ext)
{
   Family family;
   
   // add all bound sets
   for (std::map<std::string, SetPtr>::iterator
      sit=_sets_bound.begin(); sit!=_sets_bound.end(); sit++)
   {
      family.sets.insert(sit->second);
   }
   
   // add all input sets
   for (std::set<SetPtr>::iterator
      sit=sets_ext.begin(); sit!=sets_ext.end(); sit++)
   {
      family.sets.insert(*sit);
   }
   
   // if we don't have any sets yet, then we're done!
   if (!family.sets.size())
      return family;
   
   // compute all atoms used
   std::set<Atom *> atoms_used;
   for (std::set<SetPtr>::iterator
      sit=family.sets.begin(); sit!=family.sets.end(); sit++)
   {
      for (std::set<Atom *>::iterator
         ait=(*sit)->atoms.begin(); ait!=(*sit)->atoms.end(); ait++)
      {
         atoms_used.insert(*ait);
      }
   }
   
   // ok, since our family will be a subsets of _sets_all,
   // our atoms will be too small
   // here, split them into molecules
   
   // compute split of atoms used
   // start with a single molecule
   std::set< std::set<Atom *> > molecules;
   molecules.insert(atoms_used);
   for (std::set<SetPtr>::iterator sit=family.sets.begin(); sit!=family.sets.end(); sit++)
   {
      // for each split element, determine if we need to split it further
      std::vector< std::set<Atom *> > molecules_new;
      for (std::set< std::set<Atom *> >::iterator
         mit=molecules.begin(); mit!=molecules.end();)
      {
         const std::set<Atom *> & atoms = *mit;
         std::set<Atom *> atoms_inside;
         std::set<Atom *> atoms_outside;
         for (std::set<Atom *>::iterator ait=atoms.begin(); ait!=atoms.end(); ait++)
         {
            if ((*sit)->atoms.find(*ait) != (*sit)->atoms.end())
               atoms_inside.insert(*ait);
            else
               atoms_outside.insert(*ait);
         }
         if (atoms_inside.size()==0 || atoms_outside.size()==0)
         {
            mit++; // no change
         }
         else
         {
            molecules.erase(mit++); // erase
            molecules_new.push_back(atoms_inside);
            molecules_new.push_back(atoms_outside);
         }
      }
      for (unsigned int ui=0; ui<molecules_new.size(); ui++)
      {
         molecules.insert(molecules_new[ui]);
      }
   }
   
   // ok, for each molecule
   // add to the family the corresponding set
   // (create it if it doesn't yet exist)
   // also, maintain the set of base molecule sets
   std::set<SetPtr> molecule_sets;
   for (std::set< std::set<Atom *> >::iterator
      mit=molecules.begin(); mit!=molecules.end(); mit++)
   {
      const std::set<Atom *> & atoms = *mit;
      
      // do any existing sets match this?
      std::set<SetPtr>::iterator itset;
      for (itset=_sets_all.begin(); itset!=_sets_all.end(); itset++)
         if ((*itset)->atoms == atoms)
            break;
      if (itset!=_sets_all.end())
      {
         // set already exists!
         // just add it
         family.sets.insert(*itset);
         molecule_sets.insert(*itset);
      }
      else
      {
         // ok, so we need to make a new set
         // add it to all
         SetPtr set(new Set);
         set->atoms = atoms;
         for (std::set<Atom *>::iterator ait=atoms.begin(); ait!=atoms.end(); ait++)
         {
            set->checks.insert((*ait)->checks.begin(), (*ait)->checks.end());
         }
         _sets_all.insert(set);
         family.sets.insert(set);
         molecule_sets.insert(set);
      }
   }
   
   // compute non-molecule sets
   std::set<SetPtr> non_molecule_sets;
   std::set_difference(
      family.sets.begin(), family.sets.end(),
      molecule_sets.begin(), molecule_sets.end(),
      std::inserter(non_molecule_sets, non_molecule_sets.end()));
   
   // ok, add relations showing that each non-molecule set
   // is equal to the intersection of its constituent molecule sets
   for (std::set<SetPtr>::iterator
      nmit=non_molecule_sets.begin(); nmit!=non_molecule_sets.end(); nmit++)
   {
      std::set<SetPtr> constituent_molecules;
      for (std::set<SetPtr>::iterator
         mit=molecule_sets.begin(); mit!=molecule_sets.end(); mit++)
      {
         Atom * atom_first = *(*mit)->atoms.begin();
         if ((*nmit)->atoms.find(atom_first) != (*nmit)->atoms.end())
         {
            constituent_molecules.insert(*mit);
         }
      }
      // add relations
      family.relations.insert(std::make_pair(constituent_molecules, *nmit));
      std::set<SetPtr> antecedents;
      antecedents.insert(*nmit);
      for (std::set<SetPtr>::iterator
         mit=constituent_molecules.begin(); mit!=constituent_molecules.end(); mit++)
      {
         family.relations.insert(std::make_pair(antecedents, *mit));
      }
   }
   
   return family;
}


// should return a map from SetPtr -> bool (*)(vec<dReal> q)
// internally, this delegates to something else?
std::map<or_lemur::FamilyModule::SetPtr, std::pair<double,or_lemur::FamilyModule::Indicator> >
or_lemur::FamilyModule::GetIndicators(const or_lemur::FamilyModule::Family & family)
{
   if (!_initialized)
   {
      RAVELOG_ERROR("Family object not initialized!\n");
      throw OpenRAVE::openrave_exception("Family object not initialized");
   }
   OpenRAVE::RobotBasePtr robot = _robot.lock();
   if (!robot)
   {
      RAVELOG_ERROR("Robot not found!\n");
      throw OpenRAVE::openrave_exception("Robot not found");
   }
   
   // check whether the collision checker supports baked checks
   OpenRAVE::CollisionCheckerBasePtr cc = GetEnv()->GetCollisionChecker();
   if (!cc)
      throw OpenRAVE::openrave_exception("No collision checker set!");
   
   // communicate with baked checker
   std::string baked_kinbody_type;
   if (!_has_use_baked_checker || _use_baked_checker) do {
      bool success;
      std::stringstream sinput("BakeGetType"), soutput;
      try
      {
         success = cc->SendCommand(soutput, sinput); // BakeGetType
      }
      catch (const OpenRAVE::openrave_exception & exc)
      {
         break;
      }
      // start baking
      if (!success) break;
      baked_kinbody_type = soutput.str();
   } while (0);
   if (baked_kinbody_type.size())
   {
      RAVELOG_INFO("Using baking collision checker interface.\n");
   }
   else
   {
      if (_has_use_baked_checker && _use_baked_checker)
      {
         RAVELOG_ERROR("No baking collision checker interface found, although it was required.\n");
         throw OpenRAVE::openrave_exception("No baking collision checker interface found, although it was required");
      }
      RAVELOG_INFO("Not using baking collision checker interface (ignored or not found).\n");
   }

   // get live checks
   std::map<PosedLink *, OpenRAVE::KinBody::LinkPtr> livelinks = live_links();
   
   // compose output
   std::map<SetPtr, std::pair<double,Indicator> > indicators;
   
   for (std::set<SetPtr>::const_iterator sit=family.sets.begin(); sit!=family.sets.end(); sit++)
   {
      const SetPtr & set = (*sit);
      
      // iterate over set's checks
      std::set< std::pair<OpenRAVE::KinBody::LinkConstPtr, OpenRAVE::KinBody::LinkConstPtr> > pairs;
      std::set<Check>::iterator cit;
      for (cit=set->checks.begin(); cit!=set->checks.end(); cit++)
      {
         std::map<PosedLink *, OpenRAVE::KinBody::LinkPtr>::iterator needle1 = livelinks.find(cit->first);
         if (needle1 == livelinks.end())
            break;
         std::map<PosedLink *, OpenRAVE::KinBody::LinkPtr>::iterator needle2 = livelinks.find(cit->second);
         if (needle2 == livelinks.end())
            break;
         pairs.insert(std::make_pair(needle1->second, needle2->second));
      }
      
      if (cit!=set->checks.end()) // failed to find some checks
      {
         indicators.insert(std::make_pair(set, std::make_pair(
            std::numeric_limits<double>::infinity(), or_lemur::AbortingIndicator())));
      }
      else // all found!
      {
         if (baked_kinbody_type.size())
         {
            // start baking (no error checking yet!)
            std::stringstream sinput("BakeBegin BakeEnd"), soutput;
            cc->SendCommand(soutput, sinput); // BakeBegin
            OpenRAVE::KinBodyPtr baked_kinbody = OpenRAVE::RaveCreateKinBody(GetEnv(), baked_kinbody_type);
            for (std::set< std::pair<OpenRAVE::KinBody::LinkConstPtr, OpenRAVE::KinBody::LinkConstPtr> >::iterator
               it=pairs.begin(); it!=pairs.end(); it++)
            {
               cc->CheckCollision(it->first, it->second);
            }
            cc->SendCommand(soutput, sinput); // BakeEnd
            
            indicators.insert(std::make_pair(set, std::make_pair(
               _cost_per_ilc*(1.0+pairs.size()), or_lemur::BakedCheckIndicator(robot, cc, baked_kinbody))));
         }
         else
         {
            indicators.insert(std::make_pair(set, std::make_pair(
               _cost_per_ilc*(1.0+pairs.size()), or_lemur::AllPairsIndicator(robot, pairs))));
         }
      }
   }
   
   return indicators;
}


std::map<or_lemur::FamilyModule::SetPtr,std::string>
or_lemur::FamilyModule::GetCanonicalNames(const Family & family)
{
   std::map<SetPtr,std::string> names;
   int unbound_count = 0;
   for (std::set<SetPtr>::iterator it=family.sets.begin(); it!=family.sets.end(); it++)
   {
      if ((*it)->literals.size())
      {
         names.insert(std::make_pair(*it, *(*it)->literals.begin()));
      }
      else
      {
         std::stringstream ss;
         ss << "unbound" << unbound_count;
         names.insert(std::make_pair(*it, ss.str()));
         unbound_count++;
      }
   }
   return names;
}


bool or_lemur::FamilyModule::CmdGetInstanceId(std::ostream & soutput, std::istream & sinput)
{
   soutput << GetInstanceId();
   return true;
}


bool or_lemur::FamilyModule::CmdGetRobotName(std::ostream & soutput, std::istream & sinput)
{
   if (!_initialized)
   {
      RAVELOG_ERROR("Family object not initialized!\n");
      throw OpenRAVE::openrave_exception("Family object not initialized");
   }
   OpenRAVE::RobotBasePtr robot = _robot.lock();
   if (!robot)
   {
      RAVELOG_ERROR("Robot not found!\n");
      throw OpenRAVE::openrave_exception("Robot not found");
   }
   soutput << robot->GetName();
   return true;
}


bool or_lemur::FamilyModule::CmdSetCostPerIlc(std::ostream & soutput, std::istream & sinput)
{
   double cost_per_ilc;
   sinput >> cost_per_ilc;
   if (sinput.fail())
   {
      RAVELOG_ERROR("SetCostPerIlc input error.\n");
      return false;
   }
   SetCostPerIlc(cost_per_ilc);
   return true;
}


bool or_lemur::FamilyModule::CmdGetFamilyId(std::ostream & soutput, std::istream & sinput)
{
   soutput << GetFamilyId();
   return true;
}


bool or_lemur::FamilyModule::CmdLet(std::ostream & soutput, std::istream & sinput)
{
   std::string new_literal;
   std::string eq;
   sinput >> new_literal;
   sinput >> eq;
   
   if (eq != "=")
   {
      RAVELOG_ERROR("Let syntax: NEWLITERAL = EXPRESSION.\n");
      throw OpenRAVE::openrave_exception("Let syntax: NEWLITERAL = EXPRESSION");
   }
   
   std::ostringstream oss;
   oss << sinput.rdbuf();
   std::string expression = oss.str();
   
   SetPtr set = GetSetFromExpression(expression);
   
   Let(new_literal, set);
   
   // make sure to clean up, even on throw!
   
   return true;
}


bool or_lemur::FamilyModule::CmdNamesOf(std::ostream & soutput, std::istream & sinput)
{
   std::ostringstream oss;
   oss << sinput.rdbuf();
   std::string expression = oss.str();
   
   SetPtr set = GetSetFromExpression(expression);
   
   bool already = false;
   for (std::set<std::string>::iterator
      it=set->literals.begin(); it!=set->literals.end(); it++)
   {
      if (already)
         soutput << " ";
      soutput << *it;
      already = true;
   }
   
   // make sure to clean up, even on throw!
   
   return true;
}


bool or_lemur::FamilyModule::CmdPrintCurrentFamily(std::ostream & soutput, std::istream & sinput)
{
   // get current family
   Family family = GetCurrentFamily();
   SetPtr set_current = GetCurrentSet();
   
   // get names for each set
   RAVELOG_INFO("Sets:\n");
   std::map<SetPtr, std::string> names;
   int unbound_count = 0;
   for (std::set<SetPtr>::iterator it=family.sets.begin(); it!=family.sets.end(); it++)
   {
      std::stringstream ss;
      int name_count = 0;
      for (std::set<std::string>::iterator
         lit=(*it)->literals.begin(); lit!=(*it)->literals.end(); lit++)
      {
         if (name_count == 0)
            ss << *lit;
         else if (name_count == 1)
            ss << " (aka " << *lit;
         else
            ss << " aka " << *lit;
         name_count++;
      }
      if (1 < name_count)
         ss << ")";
      if (0 < name_count)
      {
         RAVELOG_INFO("- %s [%lu checks]%s\n",
            ss.str().c_str(), (*it)->checks.size(),
            (*it)==set_current ? " (current)" : "");
         names.insert(std::make_pair(*it, *(*it)->literals.begin()));
      }
      else
      {
         std::stringstream ss;
         ss << "unbound" << unbound_count; 
         RAVELOG_INFO("- %s [%lu checks]%s\n",
            ss.str().c_str(), (*it)->checks.size(),
            (*it)==set_current ? " (current)" : "");
         names.insert(std::make_pair(*it, ss.str()));
         unbound_count++;
      }   
   }
   
   RAVELOG_INFO("Relations:\n");
   for (std::set<Relation>::iterator
      rit=family.relations.begin(); rit!=family.relations.end(); rit++)
   {
      std::stringstream ss;
      int antecedents_count = 0;
      for (std::set<SetPtr>::iterator
         ait=rit->first.begin(); ait!=rit->first.end(); ait++)
      {
         if (antecedents_count)
            ss << " ^ ";
         ss << names[*ait];
         antecedents_count++;
      }
      ss << " < " << names[rit->second];
      RAVELOG_INFO("- %s\n", ss.str().c_str());
   }
   
   return true;
}


bool or_lemur::FamilyModule::CmdGetHeaderFromSet(std::ostream & soutput, std::istream & sinput)
{
   std::ostringstream oss;
   oss << sinput.rdbuf();
   std::string expression = oss.str();
   
   SetPtr set = GetSetFromExpression(expression);
   
   std::string header = GetHeaderFromSet(set);
   
   soutput << header;
   
   return true;
}


std::map<or_lemur::FamilyModule::PosedLink *, OpenRAVE::KinBody::LinkPtr>
or_lemur::FamilyModule::live_links()
{
   if (!_initialized)
   {
      RAVELOG_ERROR("Family object not initialized!\n");
      throw OpenRAVE::openrave_exception("Family object not initialized");
   }
   OpenRAVE::RobotBasePtr robot = _robot.lock();
   if (!robot)
   {
      RAVELOG_ERROR("Robot not found!\n");
      throw OpenRAVE::openrave_exception("Robot not found");
   }
   
   // first, collect all environment links attached to the appropriate proxlink
   std::vector< std::pair<int,OpenRAVE::KinBody::LinkPtr> > links;
   std::vector<OpenRAVE::KinBodyPtr> kinbodies;
   GetEnv()->GetBodies(kinbodies);
   for (unsigned int ki=0; ki<kinbodies.size(); ki++)
   {
      if (kinbodies[ki] == robot) // robot itself
      {
         const std::vector<OpenRAVE::KinBody::LinkPtr> & robot_links = robot->GetLinks();
         for (int li=0; li<(int)robot_links.size(); li++)
         {
            links.push_back(std::make_pair(_proxidxs[li],robot_links[li]));
         }
      }
      else // some other kinbody
      {
         int proxidx;
         OpenRAVE::KinBody::LinkPtr grabbinglink = robot->IsGrabbing(kinbodies[ki]);
         if (!grabbinglink)
            proxidx = 0;
         else
            proxidx = _proxidxs[grabbinglink->GetIndex()];
         // find all kinbody links
         const std::vector<OpenRAVE::KinBody::LinkPtr> & kb_links = kinbodies[ki]->GetLinks();
         for (int li=0; li<(int)kb_links.size(); li++)
         {
            links.push_back(std::make_pair(proxidx,kb_links[li]));
         }
      }
   }
   
   // for each link,
   // find a matching existing proxlink object
   std::map<PosedLink *, OpenRAVE::KinBody::LinkPtr> map;
   
   // ok, ensure that all posedlinks are in our datastructure
   // this is where the deduplication happens
   for (unsigned int ui=0; ui<links.size(); ui++)
   {
      // get link geom hash
      std::stringstream ss;
      links[ui].second->serialize(ss, OpenRAVE::SO_Geometry);
      std::string geomhash = OpenRAVE::utils::GetMD5HashString(ss.str());
      
      // get link from-prox pose
      int proxidx = links[ui].first;
      OpenRAVE::Transform tx_prox = robot->GetLinks()[proxidx]->GetTransform();
      OpenRAVE::Transform tx_link = links[ui].second->GetTransform();
      OpenRAVE::Transform tx_prox_link = tx_prox.inverse() * tx_link;
      
      // look up (proxidx,geomhash) in index
      std::map< std::pair<int,std::string>, std::set<PosedLink *> >::iterator
         indexkey = _posedlinks.find(std::make_pair(proxidx,geomhash));
      if (indexkey == _posedlinks.end())
         indexkey = _posedlinks.insert(std::make_pair(
            std::make_pair(proxidx,geomhash), std::set<PosedLink *>())).first;
      
      // search for matching posedlink object
      std::set<PosedLink *>::iterator needle;
      for (needle=indexkey->second.begin(); needle!=indexkey->second.end(); needle++)
      {
         OpenRAVE::Transform tx_rel = tx_prox_link.inverse() * (*needle)->pose;
         // is this different?
         if (tx_rel.trans.lengthsqr3() > 0.0000001)
            continue;
         OpenRAVE::dReal quat_xxyyzz = 0.0;
         quat_xxyyzz += tx_rel.rot.y * tx_rel.rot.y; // qx
         quat_xxyyzz += tx_rel.rot.z * tx_rel.rot.z; // qy
         quat_xxyyzz += tx_rel.rot.w * tx_rel.rot.w; // qz
         if (quat_xxyyzz > 0.0000001)
            continue;
         // the same!
         //printf("found already!\n");
         break;
      }
      
      // found one!
      if (needle!=indexkey->second.end())
      {
         map.insert(std::make_pair(*needle, links[ui].second));
      }
   }
   
   return map;
}

or_lemur::FamilyModule::SetPtr
or_lemur::FamilyModule::set_from_checks(const std::set<or_lemur::FamilyModule::Check> & checks)
{
   // do any existing sets match this?
   std::set<SetPtr>::iterator itset;
   for (itset=_sets_all.begin(); itset!=_sets_all.end(); itset++)
      if ((*itset)->checks == checks)
         break;
   if (itset!=_sets_all.end())
   {
      // set already exists!
      // just return it
      return *itset;
   }
   
   // ok, so we need to make a new set
   // add it to all
   SetPtr set(new Set);
   set->checks = checks;
   _sets_all.insert(set);
   
   // ok, do the atom stuff!
   
   // are there any checks not owned by any current atom?
   
   // find out which atom owns each check
   std::map<Check, Atom *> check_owner;
   for (std::set< Check >::iterator itcheck=checks.begin(); itcheck!=checks.end(); itcheck++)
   {
      std::set< Atom * >::iterator itatom;
      for (itatom=_atoms.begin(); itatom!=_atoms.end(); itatom++)
         if ((*itatom)->checks.count(*itcheck))
            break;
      if (itatom!=_atoms.end())
         check_owner[*itcheck] = (*itatom);
      else
         check_owner[*itcheck] = 0;
   }
   
   // create a new atom for any check not owned
   // also get the list of other owners
   Atom * newatom = 0;
   std::set<Atom *> owners;
   for (std::map<Check, Atom *>::iterator
      itown=check_owner.begin(); itown!=check_owner.end(); itown++)
   {
      if (itown->second)
      {
         owners.insert(itown->second);
         continue;
      }
      if (!newatom)
         newatom = new Atom;
      newatom->checks.insert(itown->first);
   }
   if (newatom)
   {
      _atoms.insert(newatom);
      set->atoms.insert(newatom);
   }
   
   // ok, go through owners;
   // for each, if it is completely within the set,
   // as it as one of its atom;
   // otherwise, split it!
   for (std::set<Atom *>::iterator
      itowner=owners.begin(); itowner!=owners.end(); itowner++)
   {
      std::set<Check> checks_inside;
      std::set<Check> checks_outside;
      for (std::set<Check>::iterator
         itcheck=(*itowner)->checks.begin(); itcheck!=(*itowner)->checks.end(); itcheck++)
      {
         if (checks.count(*itcheck))
            checks_inside.insert(*itcheck);
         else
            checks_outside.insert(*itcheck);
      }
      if (!checks_outside.size())
      {
         set->atoms.insert(*itowner);
         continue;
      }
      // ok, we need to split!
      Atom * otheratom = new Atom;
      (*itowner)->checks = checks_inside;
      otheratom->checks = checks_outside;
      set->atoms.insert(*itowner);
      // fix existing sets relying on (*itowner)
      for (std::set<SetPtr>::iterator
         itset=_sets_all.begin(); itset!=_sets_all.end(); itset++)
      {
         if ((*itset) == set)
            continue;
         if ((*itset)->atoms.count(*itowner))
            (*itset)->atoms.insert(otheratom);
      }
   }
   
   return set;
}
