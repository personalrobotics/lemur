/* File: planner_e8roadmapselfcc.cpp
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

#include <openrave/openrave.h>

#include <boost/filesystem.hpp>
#include <boost/graph/adjacency_list.hpp>

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

#include <pr_bgl/edge_indexed_graph.h>
#include <pr_bgl/overlay_manager.h>
#include <pr_bgl/heap_indexed.h>

#include <ompl_multiset/util.h>
#include <ompl_multiset/EffortModel.h>
#include <ompl_multiset/Family.h>
#include <ompl_multiset/FamilyEffortModel.h>
#include <ompl_multiset/FnString.h>
#include <ompl_multiset/SpaceID.h>
#include <ompl_multiset/SamplerGenMonkeyPatch.h>
#include <ompl_multiset/Roadmap.h>
#include <ompl_multiset/RoadmapAAGrid.h>
#include <ompl_multiset/RoadmapHalton.h>
#include <ompl_multiset/RoadmapHaltonDens.h>
#include <ompl_multiset/RoadmapRGG.h>
#include <ompl_multiset/RoadmapRGGDensConst.h>
#include <ompl_multiset/RoadmapID.h>
#include <ompl_multiset/BisectPerm.h>
#include <ompl_multiset/E8Roadmap.h>

#include <openrave/utils.h>
#include <or_multiset/inter_link_checks.h>

#include "or_checker.h"
#include "planner_e8roadmapselfcc.h"


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

void ilc_link_path_hash(std::ostream & sout,
   const std::vector<or_multiset::TxAjoint> & link_path)
{
   for (unsigned int j=0; j<link_path.size(); j++)
   {
      OpenRAVE::Transform tx = link_path[j].tx;
      
      if (tx.rot.x < 0.)
      {
         tx.rot.y = -tx.rot.y; // qx
         tx.rot.z = -tx.rot.z; // qy
         tx.rot.w = -tx.rot.w; // qz
         tx.rot.x = -tx.rot.x; // qw
      }
      sout << tx.trans.x << " " << tx.trans.y << " " << tx.trans.z;
      sout << " " << tx.rot.y << " " << tx.rot.z << " " << tx.rot.w << " " << tx.rot.x;
      OpenRAVE::KinBody::JointPtr joint = link_path[j].ajoint;
      if (joint)
      {
         const OpenRAVE::KinBody::JointInfo & info = joint->GetInfo();
         sout << " " << info._type;
         sout << " " << info._vanchor.x;
         sout << " " << info._vanchor.y;
         sout << " " << info._vanchor.z;
         for (int k=0; k<joint->GetDOF(); k++)
         {
            sout << " " << info._bIsCircular[k];
            sout << " " << info._vaxes[k].x;
            sout << " " << info._vaxes[k].y;
            sout << " " << info._vaxes[k].z;
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
   IlcChecker(const ompl::base::SpaceInformationPtr & si,
      OpenRAVE::CollisionCheckerBasePtr checker,
      const OpenRAVE::RobotBasePtr robot,
      const size_t dim,
      const std::vector<or_multiset::InterLinkCheck> & ilcs):
      ompl::base::StateValidityChecker(si),
      checker(checker), robot(robot), dim(dim), ilcs(ilcs)
   {}
   bool isValid(const ompl::base::State * state) const
   {
      double * q = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
      std::vector<OpenRAVE::dReal> adofvals(q, q+dim);
      robot->SetActiveDOFValues(adofvals, OpenRAVE::KinBody::CLA_Nothing);
      for (unsigned int i=0; i<ilcs.size(); i++)
         if (checker->CheckCollision(ilcs[i].link1, ilcs[i].link2))
            return false;
      return true;
   }
};

} // anonymous namespace

or_multiset::E8RoadmapSelfCC::PlannerParameters::PlannerParameters():
   roadmap_id(""),
   coeff_distance(1.), coeff_checkcost(0.), coeff_batch(0.)
{
   _vXMLParameters.push_back("roadmap_id");
   _vXMLParameters.push_back("coeff_distance");
   _vXMLParameters.push_back("coeff_checkcost");
   _vXMLParameters.push_back("coeff_batch");
   _vXMLParameters.push_back("alglog");
   _vXMLParameters.push_back("graph");
}

bool
or_multiset::E8RoadmapSelfCC::PlannerParameters::serialize(std::ostream& sout, int options) const
{
   if (!OpenRAVE::PlannerBase::PlannerParameters::serialize(sout))
      return false;
   sout << "<roadmap_id>" << roadmap_id << "</roadmap_id>";
   sout << "<coeff_distance>" << coeff_distance << "</coeff_distance>";
   sout << "<coeff_checkcost>" << coeff_checkcost << "</coeff_checkcost>";
   sout << "<coeff_batch>" << coeff_batch << "</coeff_batch>";
   sout << "<alglog>" << alglog << "</alglog>";
   sout << "<graph>" << graph << "</graph>";
   return !!sout;
}

OpenRAVE::BaseXMLReader::ProcessElement
or_multiset::E8RoadmapSelfCC::PlannerParameters::startElement(
   const std::string & name, const OpenRAVE::AttributesList & atts)
{
   if (el_deserializing.size())
      return PE_Ignore;
   // ask base calss
   enum OpenRAVE::BaseXMLReader::ProcessElement base;
   base = OpenRAVE::PlannerBase::PlannerParameters::startElement(name,atts);
   if (base != PE_Pass) return base;
   // can we handle it?
   if (name == "roadmap_id"
      || name == "coeff_distance"
      || name == "coeff_checkcost"
      || name == "coeff_batch"
      || name == "alglog"
      || name == "graph")
   {
      el_deserializing = name;
      return PE_Support;
   }
   return PE_Pass;
}

bool
or_multiset::E8RoadmapSelfCC::PlannerParameters::endElement(const std::string & name)
{
   if (!el_deserializing.size())
      return OpenRAVE::PlannerBase::PlannerParameters::endElement(name);
   if (name == el_deserializing)
   {
      if (el_deserializing == "roadmap_id")
         roadmap_id = _ss.str();
      if (el_deserializing == "coeff_distance")
         _ss >> coeff_distance;
      if (el_deserializing == "coeff_checkcost")
         _ss >> coeff_checkcost;
      if (el_deserializing == "coeff_batch")
         _ss >> coeff_batch;
      if (el_deserializing == "alglog")
         alglog = _ss.str();
      if (el_deserializing == "graph")
         graph = _ss.str();
   }
   else
      RAVELOG_WARN("closing tag doesnt match opening tag!\n");
   el_deserializing.clear();
   return false;
}

void or_multiset::E8RoadmapSelfCC::TagCache::load_vertex(size_t v_index, size_t & v_tag)
{
   //printf("E8RoadmapSelfCC::TagCache::load_vertex called!\n");
}

void or_multiset::E8RoadmapSelfCC::TagCache::load_edge(size_t e_index, std::vector< size_t > & e_tags)
{
   //printf("E8RoadmapSelfCC::TagCache::load_edge called!\n");
}

void or_multiset::E8RoadmapSelfCC::TagCache::save_begin()
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

void or_multiset::E8RoadmapSelfCC::TagCache::save_end()
{
   fclose(fp);
}

void or_multiset::E8RoadmapSelfCC::TagCache::save_vertex(size_t v_index, size_t & v_tag)
{
   fprintf(fp, "vprop %lu validity %c\n", v_index, tag_letters[v_tag]);
}

void or_multiset::E8RoadmapSelfCC::TagCache::save_edge(size_t e_index, std::vector< size_t > & e_tags)
{
   fprintf(fp, "eprop %lu validity ", e_index);
   for (unsigned int i=0; i<e_tags.size(); i++)
      fprintf(fp, "%c", tag_letters[e_tags[i]]);
   fprintf(fp, "\n");
}

or_multiset::E8RoadmapSelfCC::E8RoadmapSelfCC(OpenRAVE::EnvironmentBasePtr env):
   OpenRAVE::PlannerBase(env), env(env)
{
   __description = "E8 roadmap planner";
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
   PlannerParametersPtr inparams(new PlannerParameters());
   inparams_ser >> *inparams;
   RAVELOG_WARN("skipping custom PlannerParameters validation due to exception!\n");
   //params->Validate();
   return this->InitPlan(robot, inparams);
}

bool
or_multiset::E8RoadmapSelfCC::InitPlan(OpenRAVE::RobotBasePtr inrobot, OpenRAVE::PlannerBase::PlannerParametersConstPtr inparams_base)
{
   PlannerParametersConstPtr inparams = boost::dynamic_pointer_cast<PlannerParameters const>(inparams_base);
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
   params = inparams;
   robot = inrobot;
   robot_adofs = inrobot->GetActiveDOFIndices();
   
   // set up ompl space
   ompl_space.reset(new ompl::base::RealVectorStateSpace(robot_adofs.size()));
   ompl_space->as<ompl::base::RealVectorStateSpace>()->setBounds(ompl_bounds(robot));
   ompl_space->setLongestValidSegmentFraction(ompl_resolution(robot) / ompl_space->getMaximumExtent());
   ompl_space->setup();
   
   // roadmap
   tag_cache.reset(new or_multiset::E8RoadmapSelfCC::TagCache());
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
      for (unsigned int i=0; i<ilcs_all.size(); i++)
      {
         if (ilcs_all[i].link1->GetParent() != robot) continue;
         if (ilcs_all[i].link2->GetParent() != robot) continue;
         ilcs_self.push_back(ilcs_all[i]);
      }
   }
   or_multiset::compute_checks(robot, ilcs_targ);
   printf("number of ilcs in ilcs_self: %lu\n", ilcs_self.size());
   printf("number of ilcs in ilcs_targ: %lu\n", ilcs_targ.size());
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
   printf("number of ilcs in ilcs_self_only: %lu\n", ilcs_self_only.size());
   printf("number of ilcs in ilcs_targ_only: %lu\n", ilcs_targ_only.size());
   printf("number of ilcs in ilcs_both: %lu\n", ilcs_both.size());
   
   // compute header for the self checks
   {
      OpenRAVE::RobotBase::RobotStateSaver(robot,
         OpenRAVE::RobotBase::Save_LinkEnable);
      robot->Enable(true);
   
      // figure out space header
      tag_cache->selffile_header.clear();
      tag_cache->selffile_header += ompl_multiset::space_header(ompl_space);
      tag_cache->selffile_header += ompl_multiset::roadmap_header(roadmapgen.get());
      
      // figure out detail of robot self collision check
      printf("computing check hashes ...\n");
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
         if (link1_hash <= link2_hash)
         {
            std::stringstream sspath;
            ilc_link_path_hash(sspath, ilcs_self[i].link1_path);
            sspath << " ";
            ilc_link_path_hash(sspath, ilcs_self[i].link2_path);
            ilc_lines.insert(link1_hash + " " + link2_hash + " "
               + OpenRAVE::utils::GetMD5HashString(sspath.str()));
         }
         else
         {
            std::stringstream sspath;
            ilc_link_path_hash(sspath, ilcs_self[i].link2_path);
            sspath << " ";
            ilc_link_path_hash(sspath, ilcs_self[i].link1_path);
            ilc_lines.insert(link2_hash + " " + link1_hash + " "
               + OpenRAVE::utils::GetMD5HashString(sspath.str()));
         }
      }
      for (std::set<std::string>::iterator it=ilc_lines.begin(); it!=ilc_lines.end(); it++)
      {
         tag_cache->selffile_header += "ilc " + *it + "\n";
      }
      printf("subsetfile_header:\n");
      printf("%s",tag_cache->selffile_header.c_str());
      
      tag_cache->selffile_header_md5 = OpenRAVE::utils::GetMD5HashString(tag_cache->selffile_header);
      printf("subsetfile_header_md5: |%s|\n", tag_cache->selffile_header_md5.c_str());
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
         ompl_multiset::Family::Subset(si,0.1*ilcs_self_only.size())
      ));
   }
   // targ_only
   {
      ompl::base::SpaceInformationPtr si(
         new ompl::base::SpaceInformation(ompl_space));
      si->setStateValidityChecker(ompl::base::StateValidityCheckerPtr(
         new IlcChecker(si,env->GetCollisionChecker(),robot,robot_adofs.size(),ilcs_targ_only)));
      family->subsets.insert(std::make_pair("targ_only",
         ompl_multiset::Family::Subset(si,0.1*ilcs_targ_only.size())
      ));
   }
   // both
   {
      ompl::base::SpaceInformationPtr si(
         new ompl::base::SpaceInformation(ompl_space));
      si->setStateValidityChecker(ompl::base::StateValidityCheckerPtr(
         new IlcChecker(si,env->GetCollisionChecker(),robot,robot_adofs.size(),ilcs_both)));
      family->subsets.insert(std::make_pair("both",
         ompl_multiset::Family::Subset(si,0.1*ilcs_both.size())
      ));
   }
   // self
   {
      ompl::base::SpaceInformationPtr si(
         new ompl::base::SpaceInformation(ompl_space));
      si->setStateValidityChecker(ompl::base::StateValidityCheckerPtr(
         new IlcChecker(si,env->GetCollisionChecker(),robot,robot_adofs.size(),ilcs_self)));
      family->subsets.insert(std::make_pair("self",
         ompl_multiset::Family::Subset(si,0.1*ilcs_self.size())
      ));
   }
   // targ
   {
      ompl::base::SpaceInformationPtr si(
         new ompl::base::SpaceInformation(ompl_space));
      si->setStateValidityChecker(ompl::base::StateValidityCheckerPtr(
         new IlcChecker(si,env->GetCollisionChecker(),robot,robot_adofs.size(),ilcs_targ)));
      family->subsets.insert(std::make_pair("targ",
         ompl_multiset::Family::Subset(si,0.1*ilcs_targ.size())
      ));
   }
   family->intersections.insert(ompl_multiset::Family::Intersection("self","self_only","both"));
   family->intersections.insert(ompl_multiset::Family::Intersection("targ","targ_only","both"));
   
   // create the family effort model
   fem.reset(new ompl_multiset::FamilyEffortModel(*family));
   fem->set_target(family->subsets.find("targ")->second.si);
   // set up tag cache w.r.t. effort model
   tag_cache->tag_letters.resize(num_vertices(fem->g));
   for (unsigned int i=0; i<num_vertices(fem->g); i++)
   {
      if (!fem->g[vertex(i,fem->g)].knowns[fem->var_target])
         tag_cache->tag_letters[i] = 'U';
      else if (fem->g[vertex(i,fem->g)].values[fem->var_target])
         tag_cache->tag_letters[i] = 'V';
      else
         tag_cache->tag_letters[i] = 'I';
   }
   
   // set up planner
   ompl_planner.reset(new ompl_multiset::E8Roadmap(
      ompl::base::SpaceInformationPtr(new ompl::base::SpaceInformation(ompl_space)),
      *fem, *tag_cache, roadmapgen, 1));
   
   // planner params
   ompl_planner->coeff_checkcost = params->coeff_checkcost;
   ompl_planner->coeff_distance = params->coeff_distance;
   ompl_planner->coeff_batch = params->coeff_batch;
   
   // problem definition
   ompl_pdef.reset(new ompl::base::ProblemDefinition(family->subsets.find("targ")->second.si));
   ompl_set_roots(ompl_pdef, params);
   ompl_planner->setProblemDefinition(ompl_pdef);
   
   return true;
}

OpenRAVE::PlannerBase::PlannerParametersConstPtr
or_multiset::E8RoadmapSelfCC::GetParameters() const
{
   return params;
}

OpenRAVE::PlannerStatus
or_multiset::E8RoadmapSelfCC::PlanPath(OpenRAVE::TrajectoryBasePtr traj)
{
   printf("planning ...\n");
   
   std::ofstream fp_alglog;
   if (params->alglog != "")
   {
      fp_alglog.open(params->alglog.c_str());
      ompl_planner->as<ompl_multiset::E8Roadmap>()->os_alglog = &fp_alglog;
   }
   
   ompl::base::PlannerStatus ompl_status;
   ompl_status = ompl_planner->solve(ompl::base::timedPlannerTerminationCondition(10.0));
   printf("planner returned: %s\n", ompl_status.asString().c_str());
   
   ompl_planner->as<ompl_multiset::E8Roadmap>()->os_alglog = 0;
   fp_alglog.close();
   
   if (params->graph != "")
   {
      std::ofstream fp_graph;
      fp_graph.open(params->graph.c_str());
      ompl_planner->as<ompl_multiset::E8Roadmap>()->dump_graph(fp_graph);
      fp_graph.close();
   }
   
   if (ompl_status != ompl::base::PlannerStatus::EXACT_SOLUTION) return OpenRAVE::PS_Failed;
   
   // convert result
   ompl::base::PathPtr path = this->ompl_planner->getProblemDefinition()->getSolutionPath();
   ompl::geometric::PathGeometric * gpath = dynamic_cast<ompl::geometric::PathGeometric*>(path.get());
   if (!gpath)
      throw OpenRAVE::openrave_exception("ompl path is not geometric for some reason!");
   traj->Init(robot->GetActiveConfigurationSpecification());
   for (unsigned int i=0; i<gpath->getStateCount(); i++)
   {
      ompl::base::ScopedState<ompl::base::RealVectorStateSpace> s(ompl_space, gpath->getState(i));
      traj->Insert(i, std::vector<OpenRAVE::dReal>(&s[0], &s[0]+robot->GetActiveDOF()));
   }
   
   return OpenRAVE::PS_HasSolution;
}

bool or_multiset::E8RoadmapSelfCC::CacheCalculateSave(std::ostream & sout, std::istream & sin)
{
   printf("CacheCalculateSave called ...\n");
   
   OpenRAVE::RobotBase::RobotStateSaver saver(robot,
      OpenRAVE::RobotBase::Save_LinkEnable);
   robot->Enable(true);
   
   ompl_pdef.reset(new ompl::base::ProblemDefinition(family->subsets.find("self")->second.si));
   ompl_planner->setProblemDefinition(ompl_pdef);
   
   printf("checking all edges ...\n");
   ompl_planner->solve_all();
   
   printf("saving self state ...\n");
   
   // prepare cache
   
   tag_cache->save_begin();
   ompl_planner->cache_save_all();
   tag_cache->save_end();
   
   return true;
}

bool or_multiset::E8RoadmapSelfCC::GetTimes(std::ostream & sout, std::istream & sin) const
{
   sout << "checktime " << 0.0;
   sout << " totaltime " << 0.0;
   //sout << " n_checks " << ompl_checker->num_checks;
   return true;
}
