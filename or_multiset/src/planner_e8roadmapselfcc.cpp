/* File: planner_e8roadmapselfcc.cpp
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

#include <openrave/openrave.h>

#include <boost/chrono.hpp>
#include <boost/filesystem.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphml.hpp>
#include <boost/property_map/dynamic_property_map.hpp>

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
#include <ompl/datastructures/NearestNeighbors.h>

#include <pr_bgl/compose_property_map.hpp>
#include <pr_bgl/edge_indexed_graph.h>
#include <pr_bgl/overlay_manager.h>
#include <pr_bgl/string_map.h>
#include <pr_bgl/heap_indexed.h>

#include <ompl_multiset/util.h>
#include <ompl_multiset/rvstate_map_string_adaptor.h>
#include <ompl_multiset/EffortModel.h>
#include <ompl_multiset/Family.h>
#include <ompl_multiset/FamilyEffortModel.h>
#include <ompl_multiset/FnString.h>
#include <ompl_multiset/SpaceID.h>
#include <ompl_multiset/SamplerGenMonkeyPatch.h>
#include <ompl_multiset/Roadmap.h>
#include <ompl_multiset/RoadmapAAGrid.h>
#include <ompl_multiset/RoadmapFromFile.h>
#include <ompl_multiset/RoadmapHalton.h>
#include <ompl_multiset/RoadmapHaltonDens.h>
#include <ompl_multiset/RoadmapHaltonOffDens.h>
#include <ompl_multiset/RoadmapRGG.h>
#include <ompl_multiset/RoadmapRGGDensConst.h>
#include <ompl_multiset/RoadmapID.h>
#include <ompl_multiset/BisectPerm.h>
#include <ompl_multiset/E8Roadmap.h>

#include <openrave/utils.h>
#include <or_multiset/inter_link_checks.h>

#include <or_multiset/or_checker.h>
#include <or_multiset/params_e8roadmap.h>
#include <or_multiset/planner_e8roadmapselfcc.h>


namespace {

ompl::base::RealVectorBounds ompl_bounds(OpenRAVE::RobotBasePtr robot)
{
   ompl::base::RealVectorBounds bounds(robot->GetActiveDOF());
   std::vector<OpenRAVE::dReal> lowers;
   std::vector<OpenRAVE::dReal> uppers;
   robot->GetActiveDOFLimits(lowers, uppers);
   for (int i=0; i<robot->GetActiveDOF(); i++)
   {
      bounds.setLow(i, lowers[i]);
      bounds.setHigh(i, uppers[i]);
   }
   return bounds;
}

double ompl_resolution(OpenRAVE::RobotBasePtr robot)
{
   std::vector<OpenRAVE::dReal> dof_resolutions;
   robot->GetActiveDOFResolutions(dof_resolutions);
   double resolution = HUGE_VAL;
   for (unsigned int i=0; i<dof_resolutions.size(); i++)
      resolution = dof_resolutions[i] < resolution ? dof_resolutions[i] : resolution;
   return resolution;
}

void ompl_set_roots(ompl::base::ProblemDefinitionPtr ompl_pdef,
   OpenRAVE::PlannerBase::PlannerParametersConstPtr params)
{
   ompl::base::SpaceInformationPtr space_si = ompl_pdef->getSpaceInformation();
   ompl::base::StateSpacePtr space = space_si->getStateSpace();
   unsigned int dim = space->getDimension();
   
   // add start states
   ompl_pdef->clearStartStates();
   if (params->vinitialconfig.size() % dim != 0)
      throw OpenRAVE::openrave_exception("vector of initial states is not the right size!");
   unsigned int num_starts = params->vinitialconfig.size() / dim;
   for (unsigned int istart=0; istart<num_starts; istart++)
   {
      ompl::base::ScopedState<ompl::base::RealVectorStateSpace> s_start(space);
      for (unsigned int j=0; j<dim; j++)
         s_start->values[j] = params->vinitialconfig[istart*dim + j];
      ompl_pdef->addStartState(s_start);
   }
   
   // add goal states
   ompl::base::GoalStates * gs = new ompl::base::GoalStates(space_si);
   gs->clear();
   if (params->vgoalconfig.size() % dim != 0)
      throw OpenRAVE::openrave_exception("vector of goal states is not the right size!");
   unsigned int num_goals = params->vgoalconfig.size() / dim;
   for (unsigned int igoal=0; igoal<num_goals; igoal++)
   {
      ompl::base::ScopedState<ompl::base::RealVectorStateSpace> s_goal(space);
      for (unsigned int j=0; j<dim; j++)
         s_goal->values[j] = params->vgoalconfig[igoal*dim + j];
      gs->addState(s_goal);
   }
   ompl_pdef->setGoal(ompl::base::GoalPtr(gs));
}

inline std::string hashable_double(const double & in)
{
   std::stringstream ss;
   ss << std::fixed << std::setprecision(7);
   ss << in;
   std::string res = ss.str();
   if (res == "-0.0000000")
      res = "0.0000000";
   return res;
}

// does not print first path tx;
// instead, prints tx_first if it's passed
void ilc_link_path_hash(std::ostream & sout,
   const OpenRAVE::Transform * tx_first,
   const std::vector<or_multiset::TxAjoint> & link_path)
{
   if (tx_first)
   {
      sout << " tx";
      sout << " " << hashable_double(tx_first->trans.x);
      sout << " " << hashable_double(tx_first->trans.y);
      sout << " " << hashable_double(tx_first->trans.z);
      std::string quat1
         = hashable_double(tx_first->rot.y)
         + " " + hashable_double(tx_first->rot.z)
         + " " + hashable_double(tx_first->rot.w)
         + " " + hashable_double(tx_first->rot.x);
      std::string quat2
         = hashable_double(-tx_first->rot.y)
         + " " + hashable_double(-tx_first->rot.z)
         + " " + hashable_double(-tx_first->rot.w)
         + " " + hashable_double(-tx_first->rot.x);
      if (quat1 < quat2)
         sout << " " + quat1;
      else
         sout << " " + quat2;
   }
   for (unsigned int j=0; j<link_path.size(); j++)
   {
      if (j)
      {
         OpenRAVE::Transform tx = link_path[j].tx;
         sout << " tx";
         sout << " " << hashable_double(tx.trans.x);
         sout << " " << hashable_double(tx.trans.y);
         sout << " " << hashable_double(tx.trans.z);
         std::string quat1
            = hashable_double(tx.rot.y)
            + " " + hashable_double(tx.rot.z)
            + " " + hashable_double(tx.rot.w)
            + " " + hashable_double(tx.rot.x);
         std::string quat2
            = hashable_double(-tx.rot.y)
            + " " + hashable_double(-tx.rot.z)
            + " " + hashable_double(-tx.rot.w)
            + " " + hashable_double(-tx.rot.x);
         if (quat1 < quat2)
            sout << " " + quat1;
         else
            sout << " " + quat2;
      }
      OpenRAVE::KinBody::JointPtr joint = link_path[j].ajoint;
      if (joint)
      {
         sout << " j";
         const OpenRAVE::KinBody::JointInfo & info = joint->GetInfo();
         sout << " " << info._type;
         sout << " " << hashable_double(info._vanchor.x);
         sout << " " << hashable_double(info._vanchor.y);
         sout << " " << hashable_double(info._vanchor.z);
         for (int k=0; k<joint->GetDOF(); k++)
         {
            sout << " " << (info._bIsCircular[k] ? "1" : "0");
            sout << " " << hashable_double(info._vaxes[k].x);
            sout << " " << hashable_double(info._vaxes[k].y);
            sout << " " << hashable_double(info._vaxes[k].z);
         }
      }
   }
}

class IlcChecker: public ompl::base::StateValidityChecker
{
public:
   OpenRAVE::CollisionCheckerBasePtr checker;
   const OpenRAVE::RobotBasePtr robot;
   const size_t dim;
   const std::vector<or_multiset::InterLinkCheck> & ilcs;
   mutable size_t num_checks;
   IlcChecker(const ompl::base::SpaceInformationPtr & si,
      OpenRAVE::CollisionCheckerBasePtr checker,
      const OpenRAVE::RobotBasePtr robot,
      const size_t dim,
      const std::vector<or_multiset::InterLinkCheck> & ilcs):
      ompl::base::StateValidityChecker(si),
      checker(checker), robot(robot), dim(dim), ilcs(ilcs), num_checks(0)
   {}
   bool isValid(const ompl::base::State * state) const
   {
      num_checks++;
      double * q = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
      std::vector<OpenRAVE::dReal> adofvals(q, q+dim);
      robot->SetActiveDOFValues(adofvals, OpenRAVE::KinBody::CLA_Nothing);
      for (unsigned int i=0; i<ilcs.size(); i++)
         //if (checker->CheckCollision(ilcs[i].link1, ilcs[i].link2))
         if (robot->GetEnv()->CheckCollision(ilcs[i].link1, ilcs[i].link2))
            return false;
      return true;
   }
};

} // anonymous namespace


or_multiset::E8RoadmapSelfCC::TagCache::TagCache()
{
}

void or_multiset::E8RoadmapSelfCC::TagCache::load_begin(void)
{
   std::string selffname = "e8/set-" + selffile_header_md5 + ".txt";
   printf("reading from cache file: |%s|\n", selffname.c_str());
   std::string path = OpenRAVE::RaveFindDatabaseFile(selffname, true); // bRead
   if (path == "")
   {
      printf("no cache file to read from found!\n");
      fp = 0;
      return;
   }
   fp = fopen(path.c_str(), "r");
   if (!fp) throw OpenRAVE::openrave_exception("couldn't read from file, should never happen!");
   char * s = (char *) malloc(selffile_header.length()+1);
   size_t n_read = fread(s, 1, selffile_header.length(), fp);
   s[selffile_header.length()] = '\0';
   if (n_read != selffile_header.length() || std::string(s) != selffile_header)
   {
      fclose(fp);
      fp = 0;
      throw OpenRAVE::openrave_exception("header mismatch!");
   }
   printf("header matches!\n");
}

void or_multiset::E8RoadmapSelfCC::TagCache::load_end(void)
{
   if (!fp) return;
   fclose(fp);
   fp = 0;
}

void or_multiset::E8RoadmapSelfCC::TagCache::load_vertices(
   ompl_multiset::E8Roadmap::VIdxTagMap v_tag_map, size_t v_from, size_t v_to)
{
   if (!fp) return;
   for (size_t v_index=v_from; v_index<v_to; v_index++)
   {
      if (v_tag_map[v_index] != 0)
      {
         printf("we only know how to read data into tag=0!\n");
         continue;
      }
      size_t v_index_read;
      char char_read;
      int n_stored = fscanf(fp, "vprop %lu validity %c\n", &v_index_read, &char_read);
      if (n_stored != 2 || v_index != v_index_read)
      {
         printf("vertex read mismatch from file!\n");
         continue;
      }
      switch (char_read)
      {
      case 'U': break;
      case 'V': v_tag_map[v_index] = tag_self_valid; break;
      case 'I': v_tag_map[v_index] = tag_self_invalid; break;
      default:
         printf("unknown char: %c\n", char_read);
      }
   }
}

void or_multiset::E8RoadmapSelfCC::TagCache::load_edges(
   ompl_multiset::E8Roadmap::EIdxTagsMap e_tags_map, size_t e_from, size_t e_to)
{
   if (!fp) return;
   for (size_t e_index=e_from; e_index<e_to; e_index++)
   {
      std::vector<size_t> & e_tags = e_tags_map[e_index];
      char fmt[256];
      sprintf(fmt, "eprop %%lu validity %%%luc\n", e_tags.size());
      size_t e_index_read;
      char * s = (char *) malloc(e_tags.size());
      int n_stored = fscanf(fp, fmt, &e_index_read, s);
      if (n_stored != 2 || e_index != e_index_read)
      {
         printf("edge read mismatch from file!\n");
         free(s);
         continue;
      }
      for (unsigned int i=0; i<e_tags.size(); i++)
      {
         if (e_tags[i] != 0)
         {
            printf("we only know how to read data into tag=0!\n");
            continue;
         }
         switch (s[i])
         {
         case 'U': break;
         case 'V': e_tags[i] = tag_self_valid; break;
         case 'I': e_tags[i] = tag_self_invalid; break;
         default:
            printf("unknown char: %c\n", s[i]);
         }
      }
      free(s);
   }
}

void or_multiset::E8RoadmapSelfCC::TagCache::save_begin(void)
{
   std::string selffname = "e8/set-" + selffile_header_md5 + ".txt";
   printf("writing to cache file: |%s|\n", selffname.c_str());
   std::string path = OpenRAVE::RaveFindDatabaseFile(selffname, false); // bRead
   if (path == "") throw OpenRAVE::openrave_exception("couldn't find a place to write rave database entry!");
   boost::filesystem::create_directories(boost::filesystem::path(path).parent_path());
   fp = fopen(path.c_str(), "w");
   if (!fp) throw OpenRAVE::openrave_exception("couldn't write to file, should never happen!");
   fprintf(fp, "%s", selffile_header.c_str());
}

void or_multiset::E8RoadmapSelfCC::TagCache::save_end(void)
{
   if (!fp) return;
   fclose(fp);
   fp = 0;
}

void or_multiset::E8RoadmapSelfCC::TagCache::save_vertices(
   ompl_multiset::E8Roadmap::VIdxTagMap v_tag_map, size_t v_from, size_t v_to)
{
   if (!fp) return;
   for (size_t v_index=v_from; v_index<v_to; v_index++)
   {
      size_t & v_tag = v_tag_map[v_index];
      fprintf(fp, "vprop %lu validity %c\n", v_index, tag_letters[v_tag]);
   }
}

void or_multiset::E8RoadmapSelfCC::TagCache::save_edges(
   ompl_multiset::E8Roadmap::EIdxTagsMap e_tags_map, size_t e_from, size_t e_to)
{
   if (!fp) return;
   for (size_t e_index=e_from; e_index<e_to; e_index++)
   {
      std::vector< size_t > & e_tags = e_tags_map[e_index];
      fprintf(fp, "eprop %lu validity ", e_index);
      for (unsigned int i=0; i<e_tags.size(); i++)
         fprintf(fp, "%c", tag_letters[e_tags[i]]);
      fprintf(fp, "\n");
   }
}

or_multiset::E8RoadmapSelfCC::E8RoadmapSelfCC(OpenRAVE::EnvironmentBasePtr env):
   OpenRAVE::PlannerBase(env), env(env)
{
   __description = "E8 roadmap planner";
   RegisterCommand("GetSelfHeader",
      boost::bind(&or_multiset::E8RoadmapSelfCC::GetSelfHeader,this,_1,_2),
      "get self header");
   RegisterCommand("GetSelfHash",
      boost::bind(&or_multiset::E8RoadmapSelfCC::GetSelfHash,this,_1,_2),
      "get self hash");
   RegisterCommand("GetTimes",
      boost::bind(&or_multiset::E8RoadmapSelfCC::GetTimes,this,_1,_2),
      "get timing information from last plan");
   RegisterCommand("CacheCalculateSave",
      boost::bind(&or_multiset::E8RoadmapSelfCC::CacheCalculateSave,this,_1,_2),
      "cache calculate and save");
}

or_multiset::E8RoadmapSelfCC::~E8RoadmapSelfCC()
{
}

bool
or_multiset::E8RoadmapSelfCC::InitPlan(OpenRAVE::RobotBasePtr robot, std::istream & inparams_ser)
{
   or_multiset::E8RoadmapParametersPtr inparams(new or_multiset::E8RoadmapParameters());
   inparams_ser >> *inparams;
   inparams->Validate();
   return this->InitPlan(robot, inparams);
}

bool
or_multiset::E8RoadmapSelfCC::InitPlan(OpenRAVE::RobotBasePtr inrobot, OpenRAVE::PlannerBase::PlannerParametersConstPtr inparams_base)
{
   or_multiset::E8RoadmapParametersConstPtr inparams = boost::dynamic_pointer_cast<or_multiset::E8RoadmapParameters const>(inparams_base);
   if (!inparams)
   {
      RAVELOG_WARN("Warning, E8RoadmapSelfCC planner passed an unknown PlannerParameters type! Attempting to serialize ...\n");
      std::stringstream inparams_ser;
      inparams_ser << *inparams_base;
      return this->InitPlan(inrobot, inparams_ser);
   }
   if (!inrobot || !inparams_base)
      throw OpenRAVE::openrave_exception("robot/params objects must be passed!");
   
   // do setup
   params_ptr = inparams;
   alglog = inparams->has_alglog ? inparams->alglog : "";
   graph = inparams->has_graph ? inparams->graph : "";
   robot = inrobot;
   robot_adofs = inrobot->GetActiveDOFIndices();
   
   // set up ompl space
   ompl_space.reset(new ompl::base::RealVectorStateSpace(robot_adofs.size()));
   ompl_space->as<ompl::base::RealVectorStateSpace>()->setBounds(ompl_bounds(robot));
   ompl_space->setLongestValidSegmentFraction(ompl_resolution(robot) / ompl_space->getMaximumExtent());
   ompl_space->setup();
   
   // roadmap
   tag_cache.reset(new or_multiset::E8RoadmapSelfCC::TagCache());
   if (!inparams->has_roadmap_id)
      throw OpenRAVE::openrave_exception("no roadmap_id parameter passed!");
   try
   {
      roadmapgen.reset(ompl_multiset::make_roadmap_gen<ompl_multiset::E8Roadmap::Roadmap>(ompl_space, inparams->roadmap_id));
   }
   catch (const std::runtime_error & ex)
   {
      throw OpenRAVE::openrave_exception("failure to create roadmap!");
   }
   
   // compute ilcs
   {
      OpenRAVE::RobotBase::RobotStateSaver saver(robot,
         OpenRAVE::RobotBase::Save_LinkEnable);
      robot->Enable(true);
      std::vector<or_multiset::InterLinkCheck> ilcs_all;
      or_multiset::compute_checks(robot, ilcs_all);
      ilcs_self.clear();
      for (unsigned int i=0; i<ilcs_all.size(); i++)
      {
         if (ilcs_all[i].link1->GetParent() != robot) continue;
         if (ilcs_all[i].link2->GetParent() != robot) continue;
         ilcs_self.push_back(ilcs_all[i]);
      }
   }
   or_multiset::compute_checks(robot, ilcs_targ);
   ilcs_self_only.clear();
   ilcs_targ_only.clear();
   ilcs_both.clear();
   for (unsigned int i=0; i<ilcs_self.size(); i++)
   {
      unsigned int j;
      for (j=0; j<ilcs_targ.size(); j++)
         if (ilcs_self[i] == ilcs_targ[j])
            break;
      if (j<ilcs_targ.size())
         ilcs_both.push_back(ilcs_self[i]);
      else
         ilcs_self_only.push_back(ilcs_self[i]);
   }
   for (unsigned int i=0; i<ilcs_targ.size(); i++)
   {
      unsigned int j;
      for (j=0; j<ilcs_both.size(); j++)
         if (ilcs_targ[i] == ilcs_both[j])
            break;
      if (!(j<ilcs_both.size()))
         ilcs_targ_only.push_back(ilcs_targ[i]);
   }
   //printf("number of ilcs in ilcs_self_only: %lu\n", ilcs_self_only.size());
   //printf("number of ilcs in ilcs_targ_only: %lu\n", ilcs_targ_only.size());
   //printf("number of ilcs in ilcs_both: %lu\n", ilcs_both.size());
   
   // compute header for the self checks
   {
      // figure out space header
      tag_cache->selffile_header.clear();
      tag_cache->selffile_header += ompl_multiset::space_header(ompl_space);
      tag_cache->selffile_header += ompl_multiset::roadmap_header(roadmapgen.get());
      
      // figure out detail of robot self collision check
      // (linka_hash,linkb_hash), with linka_hash < linkb_hash
      std::set<std::string> ilc_lines;
      for (unsigned int i=0; i<ilcs_self.size(); i++)
      {
         // assume link collision data is in the link frame
         // link1 collision data
         std::stringstream ss1;
         ilcs_self[i].link1->GetCollisionData().serialize(ss1, 0);
         std::string link1_hash = OpenRAVE::utils::GetMD5HashString(ss1.str());
         // link2 collision data
         std::stringstream ss2;
         ilcs_self[i].link2->GetCollisionData().serialize(ss2, 0);
         std::string link2_hash = OpenRAVE::utils::GetMD5HashString(ss2.str());
         // do consistent ordering
         std::string ilc_link1_link2;
         std::string ilc_link2_link1;
         {
            // order: link1 link2
            OpenRAVE::Transform tx_b_wrt_a = ilcs_self[i].link2_path[0].tx;
            std::stringstream sspath;
            sspath << "linka_path";
            ilc_link_path_hash(sspath, 0, ilcs_self[i].link1_path);
            sspath << " linkb_path";
            ilc_link_path_hash(sspath, &tx_b_wrt_a, ilcs_self[i].link2_path);
            std::string sspath_str = sspath.str();
            //printf("  sspath: |%s| hash: |%s|\n",
            //   sspath_str.c_str(),OpenRAVE::utils::GetMD5HashString(sspath_str).c_str());
            //printf("    chars:"); for (unsigned int j=0; j<sspath_str.length(); j++) printf(" %d", sspath_str[j]); printf("\n");
            ilc_link1_link2 = link1_hash + " " + link2_hash + " "
               + OpenRAVE::utils::GetMD5HashString(sspath_str);
            //   + sspath_str;
         }
         {
            // order: link2 link1
            OpenRAVE::Transform tx_b_wrt_a = ilcs_self[i].link2_path[0].tx.inverse();
            std::stringstream sspath;
            sspath << "linka_path";
            ilc_link_path_hash(sspath, 0, ilcs_self[i].link2_path);
            sspath << " linkb_path";
            ilc_link_path_hash(sspath, &tx_b_wrt_a, ilcs_self[i].link1_path);
            std::string sspath_str = sspath.str();
            //printf("  sspath: |%s| hash: |%s|\n",
            //   sspath_str.c_str(),OpenRAVE::utils::GetMD5HashString(sspath_str).c_str());
            //printf("    chars:"); for (unsigned int j=0; j<sspath_str.length(); j++) printf(" %d", sspath_str[j]); printf("\n");
            ilc_link2_link1 = link2_hash + " " + link1_hash + " "
               + OpenRAVE::utils::GetMD5HashString(sspath_str);
            //   + sspath_str;
         }
         if (ilc_link1_link2 <= ilc_link2_link1)
            ilc_lines.insert(ilc_link1_link2);
         else
            ilc_lines.insert(ilc_link2_link1);
      }
      for (std::set<std::string>::iterator it=ilc_lines.begin(); it!=ilc_lines.end(); it++)
      {
         tag_cache->selffile_header += "ilc " + *it + "\n";
      }
      //printf("subsetfile_header:\n");
      //printf("%s",tag_cache->selffile_header.c_str());
      
      tag_cache->selffile_header_md5 = OpenRAVE::utils::GetMD5HashString(tag_cache->selffile_header);
      //printf("subsetfile_header_md5: |%s|\n", tag_cache->selffile_header_md5.c_str());
   }
   
   // create the family
   family.reset(new ompl_multiset::Family());
   // self_only
   {
      ompl::base::SpaceInformationPtr si(
         new ompl::base::SpaceInformation(ompl_space));
      si->setStateValidityChecker(ompl::base::StateValidityCheckerPtr(
         new IlcChecker(si,env->GetCollisionChecker(),robot,robot_adofs.size(),ilcs_self_only)));
      family->subsets.insert(std::make_pair("self_only",
         ompl_multiset::Family::Subset(si,0.1*(1+ilcs_self_only.size()))
      ));
   }
   // targ_only
   {
      ompl::base::SpaceInformationPtr si(
         new ompl::base::SpaceInformation(ompl_space));
      si->setStateValidityChecker(ompl::base::StateValidityCheckerPtr(
         new IlcChecker(si,env->GetCollisionChecker(),robot,robot_adofs.size(),ilcs_targ_only)));
      family->subsets.insert(std::make_pair("targ_only",
         ompl_multiset::Family::Subset(si,0.1*(1+ilcs_targ_only.size()))
      ));
   }
   // both
   {
      ompl::base::SpaceInformationPtr si(
         new ompl::base::SpaceInformation(ompl_space));
      si->setStateValidityChecker(ompl::base::StateValidityCheckerPtr(
         new IlcChecker(si,env->GetCollisionChecker(),robot,robot_adofs.size(),ilcs_both)));
      family->subsets.insert(std::make_pair("both",
         ompl_multiset::Family::Subset(si,0.1*(1+ilcs_both.size()))
      ));
   }
   // self
   {
      ompl::base::SpaceInformationPtr si(
         new ompl::base::SpaceInformation(ompl_space));
      si->setStateValidityChecker(ompl::base::StateValidityCheckerPtr(
         new IlcChecker(si,env->GetCollisionChecker(),robot,robot_adofs.size(),ilcs_self)));
      family->subsets.insert(std::make_pair("self",
         ompl_multiset::Family::Subset(si,0.1*(1+ilcs_self.size()))
      ));
   }
   // targ
   {
      ompl::base::SpaceInformationPtr si(
         new ompl::base::SpaceInformation(ompl_space));
      si->setStateValidityChecker(ompl::base::StateValidityCheckerPtr(
         new IlcChecker(si,env->GetCollisionChecker(),robot,robot_adofs.size(),ilcs_targ)));
      family->subsets.insert(std::make_pair("targ",
         ompl_multiset::Family::Subset(si,0.1*(1+ilcs_targ.size()))
      ));
   }
   family->intersections.insert(ompl_multiset::Family::Intersection("self","self_only","both"));
   family->intersections.insert(ompl_multiset::Family::Intersection("targ","targ_only","both"));
   
   // create the family effort model
   fem.reset(new ompl_multiset::FamilyEffortModel(*family));
   fem->set_target(family->subsets.find("targ")->second.si);
   
   // set up tag cache w.r.t. effort model
   {
      // get the fem var corresponding to the self subset
      std::size_t fem_var_self;
      for (fem_var_self=0; fem_var_self<fem->subsets.size(); fem_var_self++)
         if (fem->subsets[fem_var_self].first == "self")
            break;
      if (!(fem_var_self<fem->subsets.size()))
         throw OpenRAVE::openrave_exception("couldnt find self subset in fem!");
      // compute tag_letters (for saving)
      tag_cache->tag_letters.resize(num_vertices(fem->g));
      for (unsigned int i=0; i<num_vertices(fem->g); i++)
      {
         if (!fem->g[vertex(i,fem->g)].knowns[fem_var_self])
            tag_cache->tag_letters[i] = 'U';
         else if (fem->g[vertex(i,fem->g)].values[fem_var_self])
            tag_cache->tag_letters[i] = 'V';
         else
            tag_cache->tag_letters[i] = 'I';
      }
      // compute tag_self_valid/tag_self_invalid (for loading)
      tag_cache->tag_self_valid = 0;
      tag_cache->tag_self_invalid = 0;
      typedef boost::graph_traits<ompl_multiset::FamilyEffortModel::Graph>::out_edge_iterator OutEdgeIter;
      OutEdgeIter fem_ei, fem_ei_end;
      for (boost::tie(fem_ei,fem_ei_end)=out_edges(vertex(0,fem->g),fem->g); fem_ei!=fem_ei_end; fem_ei++)
      {
         if (fem->g[*fem_ei].var != fem_var_self)
            continue;
         if (fem->g[*fem_ei].value)
            tag_cache->tag_self_valid = target(*fem_ei,fem->g);
         else
            tag_cache->tag_self_invalid = target(*fem_ei,fem->g);
      }
      if (!tag_cache->tag_self_valid || !tag_cache->tag_self_invalid)
         throw OpenRAVE::openrave_exception("couldnt find tags for self check from fem!");
   }
   
   // set up planner
   ompl_planner.reset(new ompl_multiset::E8Roadmap(ompl_space, *fem, *tag_cache, roadmapgen));
   
   // planner params
   if (inparams->has_coeff_distance)
      ompl_planner->setCoeffDistance(inparams->coeff_distance);
   if (inparams->has_coeff_checkcost)
      ompl_planner->setCoeffCheckcost(inparams->coeff_checkcost);
   if (inparams->has_coeff_batch)
      ompl_planner->setCoeffBatch(inparams->coeff_batch);
   if (inparams->has_do_timing)
      ompl_planner->setDoTiming(inparams->do_timing);
   if (inparams->has_persist_roots)
      ompl_planner->setPersistRoots(inparams->persist_roots);
   if (inparams->has_num_batches_init)
      ompl_planner->setNumBatchesInit(inparams->num_batches_init);
   if (inparams->has_max_batches)
      ompl_planner->setMaxBatches(inparams->max_batches);
   if (inparams->has_search_type)
      ompl_planner->setSearchType(inparams->search_type);
   if (inparams->has_eval_type)
      ompl_planner->setEvalType(inparams->eval_type);
   
   // problem definition
   ompl_pdef.reset(new ompl::base::ProblemDefinition(family->subsets.find("targ")->second.si));
   ompl_set_roots(ompl_pdef, inparams);
   ompl_planner->setProblemDefinition(ompl_pdef);
   
   return true;
}

OpenRAVE::PlannerBase::PlannerParametersConstPtr
or_multiset::E8RoadmapSelfCC::GetParameters() const
{
   return params_ptr;
}

OpenRAVE::PlannerStatus
or_multiset::E8RoadmapSelfCC::PlanPath(OpenRAVE::TrajectoryBasePtr traj)
{
   for (std::map<std::string, ompl_multiset::Family::Subset>::iterator
      it=family->subsets.begin(); it!=family->subsets.end(); it++)
   {
      boost::dynamic_pointer_cast<IlcChecker>(it->second.si->getStateValidityChecker())->num_checks = 0;
   }
   
   std::ofstream fp_alglog;
   if (alglog == "-")
   {
      ompl_planner->as<ompl_multiset::E8Roadmap>()->os_alglog = &std::cout;
   }
   else if (alglog != "")
   {
      fp_alglog.open(alglog.c_str());
      ompl_planner->as<ompl_multiset::E8Roadmap>()->os_alglog = &fp_alglog;
   }
   
   ompl::base::PlannerStatus ompl_status;
   ompl_status = ompl_planner->solve(ompl::base::timedPlannerTerminationCondition(10.0));
   printf("planner returned: %s\n", ompl_status.asString().c_str());
   
   ompl_planner->as<ompl_multiset::E8Roadmap>()->os_alglog = 0;
   fp_alglog.close();
   
   if (graph == "-")
   {
      ompl_planner->as<ompl_multiset::E8Roadmap>()->dump_graph(std::cout);
   }
   else if (graph != "")
   {
      std::ofstream fp_graph;
      fp_graph.open(graph.c_str());
      ompl_planner->as<ompl_multiset::E8Roadmap>()->dump_graph(fp_graph);
      fp_graph.close();
   }
   
   switch (ompl::base::PlannerStatus::StatusType(ompl_status))
   {
   case ompl::base::PlannerStatus::UNKNOWN:
   case ompl::base::PlannerStatus::CRASH:
   case ompl::base::PlannerStatus::TYPE_COUNT:
   case ompl::base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE:
      throw OpenRAVE::openrave_exception("ompl planner threw unexpected status!");
   case ompl::base::PlannerStatus::INVALID_START:
   case ompl::base::PlannerStatus::INVALID_GOAL:
      return OpenRAVE::PS_Failed;
   case ompl::base::PlannerStatus::TIMEOUT:
      return OpenRAVE::PS_Interrupted;
   case ompl::base::PlannerStatus::APPROXIMATE_SOLUTION:
   case ompl::base::PlannerStatus::EXACT_SOLUTION:
      break;
   }
   
   // check if the planner found a solution path
   // (if the planner exited with an exact empty solution, then it's done!)
   ompl::base::PathPtr path = this->ompl_planner->getProblemDefinition()->getSolutionPath();
   if (!path)
      return OpenRAVE::PS_Failed;
   
   // convert result
   ompl::geometric::PathGeometric * gpath = dynamic_cast<ompl::geometric::PathGeometric*>(path.get());
   if (!gpath)
      throw OpenRAVE::openrave_exception("ompl path is not geometric for some reason!");
   traj->Init(robot->GetActiveConfigurationSpecification());
   for (unsigned int i=0; i<gpath->getStateCount(); i++)
   {
      ompl::base::ScopedState<ompl::base::RealVectorStateSpace> s(ompl_space, gpath->getState(i));
      traj->Insert(i, std::vector<OpenRAVE::dReal>(&s[0], &s[0]+robot->GetActiveDOF()));
   }
   
   if (ompl_status == ompl::base::PlannerStatus::APPROXIMATE_SOLUTION)
      return OpenRAVE::PS_InterruptedWithSolution;
   else
      return OpenRAVE::PS_HasSolution;
}

bool or_multiset::E8RoadmapSelfCC::GetSelfHeader(std::ostream & sout, std::istream & sin) const
{
   sout << tag_cache->selffile_header;
   return true;
}

bool or_multiset::E8RoadmapSelfCC::GetSelfHash(std::ostream & sout, std::istream & sin) const
{
   sout << tag_cache->selffile_header_md5;
   return true;
}

bool or_multiset::E8RoadmapSelfCC::CacheCalculateSave(std::ostream & sout, std::istream & sin)
{
   printf("CacheCalculateSave called ...\n");
   
   OpenRAVE::RobotBase::RobotStateSaver saver(robot,
      OpenRAVE::RobotBase::Save_LinkEnable);
   robot->Enable(true);
   
   ompl_pdef.reset(new ompl::base::ProblemDefinition(family->subsets.find("self")->second.si));
   ompl_planner->setProblemDefinition(ompl_pdef);
   fem->set_target(family->subsets.find("self")->second.si);
   
   ompl_planner->solve_all();
   
   ompl_planner->cache_save_all();
   
   return true;
}

bool or_multiset::E8RoadmapSelfCC::GetTimes(std::ostream & sout, std::istream & sin) const
{
   sout << "checktime " << 0.0;
   sout << " totaltime " << 0.0;
   for (std::map<std::string, ompl_multiset::Family::Subset>::iterator
      it=family->subsets.begin(); it!=family->subsets.end(); it++)
   {
      sout << " n_checks_" << it->first;
      sout << " " << boost::dynamic_pointer_cast<IlcChecker>(it->second.si->getStateValidityChecker())->num_checks;
   }
   return true;
}
