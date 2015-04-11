/* File: testherb.cpp
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

#include <csignal>
#include <cstdio>
#include <boost/thread.hpp>
#include <openrave-core.h>
#include <openrave/utils.h>
#include <openrave/interface.h>
#include <openrave/planningutils.h>

#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/Planner.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>

#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include <ompl_multiset/Roadmap.h>
#include <ompl_multiset/RoadmapSampledConst.h>
#include <ompl_multiset/Cache.h>
#include <ompl_multiset/MultiSetPRM.h>

OpenRAVE::Transform ortx_from_pose(const double pose[7])
{
   return OpenRAVE::Transform(
      OpenRAVE::Vector(pose[6],pose[3],pose[4],pose[5]),
      OpenRAVE::Vector(pose[0],pose[1],pose[2]));
}

OpenRAVE::Transform ortx_from_vpose(
   double x, double y, double z, double qx, double qy, double qz, double qw)
{
   return OpenRAVE::Transform(OpenRAVE::Vector(qw,qx,qy,qz),OpenRAVE::Vector(x,y,z));
}

OpenRAVE::EnvironmentBasePtr penv;
OpenRAVE::RobotBasePtr probot;
OpenRAVE::RobotBasePtr probot_padded;
OpenRAVE::ModuleBasePtr pikfast;
OpenRAVE::KinBodyPtr kb_kitchen;
OpenRAVE::KinBodyPtr kb_table;
OpenRAVE::KinBodyPtr kb_bin;
OpenRAVE::KinBodyPtr kb_mug;
OpenRAVE::KinBodyPtr kb_frame;

const double pose_ee_palm[7] = { 0., 0., 0.1365, 0., 0., 0., 1. };
const double pose_mug_grasp1[7] = { 0.15, 0., 0.09, -0.5, -0.5, 0.5, 0.5 };
const double pose_mugB[7] = { -0.3975, 1.61, 0.735, 0., 0., 1., 0. }; // table
const double pose_mugD[7] = { -1.1, 2.3, 0.0, 0.,0.,M_SQRT1_2,-M_SQRT1_2 }; // bin
const double pose_mug_drop[7] = { -1.1, 2.3, 0.735, 0.,0.,M_SQRT1_2,-M_SQRT1_2 }; // drop location

unsigned long long g_checktime;

/* spaces:
 * R: (robot) x (robot+kitchen+table+bin)
 * T: (robot) x (mug-on-table)
 * H: (mug-in-hand) x (robot+kitchen+table+bin)
 * D: (robot) x (mug-in-bin)
 * 
 * P: (padded_robot) x (padded_robot+kitchen+table+bin)
 *
 * step 1, in RnT
 * step 2, in RnH
 * step 3, in RnD
 * 
 * also [EXPERIMENTAL] RE and RS, both independent of moveable/grasped objects
 */

#define RELS_BASELINE 1
#define RELS_OG_ONLY 2
#define RELS_SELFCC_ONLY 3
#define RELS_OG_SELFCC 4

//#define BLACK_BOX_BROAD_PHASE 1
#define LAMBDA (0.0001)
//#define LAMBDA (0.5)
//#define LAMBDA (0.9999)
#define RELS 1

#define PLANNER_MULTISET 1
#define PLANNER_RRT 2
#define PLANNER_LBKPIECE1 3
#define PLANNER_EST 4

#define PLANNER PLANNER_MULTISET

//#define CLOCK CLOCK
#define CLOCK CLOCK_REALTIME

// this is regardless of the mug location and grabbed state
bool isvalid_now_PS(void)
{
   if (penv->CheckStandaloneSelfCollision(probot_padded)) return false; // ignores grabbed bodies
   return true;
}

// this is regardless of the mug location and grabbed state
bool isvalid_now_PE(void)
{
   const std::vector<OpenRAVE::KinBody::LinkPtr> rlinks = probot_padded->GetLinks();
   for (unsigned int i=0; i<rlinks.size(); i++)
   {
      if (penv->CheckCollision(rlinks[i], kb_kitchen)) return false;
      if (penv->CheckCollision(rlinks[i], kb_table)) return false;
      if (penv->CheckCollision(rlinks[i], kb_bin)) return false;
   }
   return true;
}

// this is regardless of the mug location and grabbed state
bool isvalid_now_RS(void)
{
#ifdef BLACK_BOX_BROAD_PHASE
   // DO SMART SHORT CIRCUIT
   std::vector<OpenRAVE::dReal> adofvals;
   probot->GetActiveDOFValues(adofvals);
   probot_padded->SetActiveDOFValues(adofvals);
   if (isvalid_now_PS())
      return true;
#endif

   if (penv->CheckStandaloneSelfCollision(probot)) return false; // ignores grabbed bodies
   return true;
}

// this is regardless of the mug location and grabbed state
bool isvalid_now_RE(void)
{
#ifdef BLACK_BOX_BROAD_PHASE
   // DO SMART SHORT CIRCUIT
   std::vector<OpenRAVE::dReal> adofvals;
   probot->GetActiveDOFValues(adofvals);
   probot_padded->SetActiveDOFValues(adofvals);
   if (isvalid_now_PE())
      return true;
#endif

   const std::vector<OpenRAVE::KinBody::LinkPtr> rlinks = probot->GetLinks();
   for (unsigned int i=0; i<rlinks.size(); i++)
   {
      if (penv->CheckCollision(rlinks[i], kb_kitchen)) return false;
      if (penv->CheckCollision(rlinks[i], kb_table)) return false;
      if (penv->CheckCollision(rlinks[i], kb_bin)) return false;
   }
   return true;
}

bool isvalid_now_P(void);

// this is regardless of the mug location and grabbed state
bool isvalid_now_R(void)
{
#ifdef BLACK_BOX_BROAD_PHASE
   // DO SMART SHORT CIRCUIT
   std::vector<OpenRAVE::dReal> adofvals;
   probot->GetActiveDOFValues(adofvals);
   probot_padded->SetActiveDOFValues(adofvals);
   if (isvalid_now_P())
      return true;
#endif
   
   //if (probot->CheckSelfCollision()) return false;
   if (penv->CheckStandaloneSelfCollision(probot)) return false; // ignores grabbed bodies
   //if (penv->CheckCollision(probot, kb_kitchen)) return false;
   //if (penv->CheckCollision(probot, kb_table)) return false;
   //if (penv->CheckCollision(probot, kb_bin)) return false;
   const std::vector<OpenRAVE::KinBody::LinkPtr> rlinks = probot->GetLinks();
   for (unsigned int i=0; i<rlinks.size(); i++)
   {
      if (penv->CheckCollision(rlinks[i], kb_kitchen)) return false;
      if (penv->CheckCollision(rlinks[i], kb_table)) return false;
      if (penv->CheckCollision(rlinks[i], kb_bin)) return false;
   }
   return true;
}

// this is regardless of the mug location and grabbed state
bool isvalid_now_P(void)
{
   //if (probot->CheckSelfCollision()) return false;
   if (penv->CheckStandaloneSelfCollision(probot_padded)) return false; // ignores grabbed bodies
   //if (penv->CheckCollision(probot, kb_kitchen)) return false;
   //if (penv->CheckCollision(probot, kb_table)) return false;
   //if (penv->CheckCollision(probot, kb_bin)) return false;
   const std::vector<OpenRAVE::KinBody::LinkPtr> rlinks = probot_padded->GetLinks();
   for (unsigned int i=0; i<rlinks.size(); i++)
   {
      if (penv->CheckCollision(rlinks[i], kb_kitchen)) return false;
      if (penv->CheckCollision(rlinks[i], kb_table)) return false;
      if (penv->CheckCollision(rlinks[i], kb_bin)) return false;
   }
   return true;
}

bool isvalid_now_TD(void)
{
   if (penv->CheckCollision(probot, kb_mug)) return false;
   return true;
}


bool isvalid_now_TD_padded(void)
{
   if (penv->CheckCollision(probot_padded, kb_mug)) return false;
   return true;
}

bool isvalid_now_H(void)
{
   /* check mug against robot */
   const std::vector<OpenRAVE::KinBody::LinkPtr> mlinks = kb_mug->GetLinks();
   const std::vector<OpenRAVE::KinBody::LinkPtr> rlinks = probot->GetLinks();
   for (unsigned int j=0; j<mlinks.size(); j++)
   {
      for (unsigned int i=0; i<rlinks.size(); i++)
      {
         if (penv->CheckCollision(rlinks[i], mlinks[j])) return false;
         /* ignore collision with grabbed link(s)? we should do that with GetGrabbedInfo */
      }
      if (penv->CheckCollision(mlinks[j], kb_kitchen)) return false;
      if (penv->CheckCollision(mlinks[j], kb_table)) return false;
      if (penv->CheckCollision(mlinks[j], kb_bin)) return false;
   }
   return true;
}

bool isvalid_now_H_padded(void)
{
   /* check mug against robot */
   const std::vector<OpenRAVE::KinBody::LinkPtr> mlinks = kb_mug->GetLinks();
   const std::vector<OpenRAVE::KinBody::LinkPtr> rlinks = probot_padded->GetLinks();
   for (unsigned int j=0; j<mlinks.size(); j++)
   {
      for (unsigned int i=0; i<rlinks.size(); i++)
      {
         if (rlinks[i]->GetName() == "/right/wam7")
            continue;
         if (penv->CheckCollision(rlinks[i], mlinks[j])) return false;
         /* ignore collision with grabbed link(s)? we should do that with GetGrabbedInfo */
      }
      if (penv->CheckCollision(mlinks[j], kb_kitchen)) return false;
      if (penv->CheckCollision(mlinks[j], kb_table)) return false;
      if (penv->CheckCollision(mlinks[j], kb_bin)) return false;
   }
   return true;
}

bool isvalid_RS(const ompl::base::State * s)
{
   struct timespec tic;
   struct timespec toc;
   bool isvalid;
   clock_gettime(CLOCK, &tic);
   // regardless of mug location
   double * q = s->as<ompl::base::RealVectorStateSpace::StateType>()->values;
   std::vector<double> adofvals(q,q+7);
   probot->SetActiveDOFValues(adofvals);
   isvalid = isvalid_now_RS();
   clock_gettime(CLOCK, &toc);
   g_checktime += (toc.tv_nsec - tic.tv_nsec) + 1000000000*(toc.tv_sec - tic.tv_sec);
   return isvalid;
}

bool isvalid_RE(const ompl::base::State * s)
{
   struct timespec tic;
   struct timespec toc;
   bool isvalid;
   clock_gettime(CLOCK, &tic);
   // regardless of mug location
   double * q = s->as<ompl::base::RealVectorStateSpace::StateType>()->values;
   std::vector<double> adofvals(q,q+7);
   probot->SetActiveDOFValues(adofvals);
   isvalid = isvalid_now_RE();
   clock_gettime(CLOCK, &toc);
   g_checktime += (toc.tv_nsec - tic.tv_nsec) + 1000000000*(toc.tv_sec - tic.tv_sec);
   return isvalid;
}

// SMART VERSION THAT INTERNALLY USES P!
bool isvalid_R(const ompl::base::State * s)
{
   struct timespec tic;
   struct timespec toc;
   bool isvalid;
   clock_gettime(CLOCK, &tic);
   // regardless of mug location
   double * q = s->as<ompl::base::RealVectorStateSpace::StateType>()->values;
   std::vector<double> adofvals(q,q+7);
   probot->SetActiveDOFValues(adofvals);
   isvalid = isvalid_now_R();
   clock_gettime(CLOCK, &toc);
   g_checktime += (toc.tv_nsec - tic.tv_nsec) + 1000000000*(toc.tv_sec - tic.tv_sec);
   return isvalid;
}

bool isvalid_P(const ompl::base::State * s)
{
   struct timespec tic;
   struct timespec toc;
   bool isvalid;
   clock_gettime(CLOCK, &tic);
   // regardless of mug location
   double * q = s->as<ompl::base::RealVectorStateSpace::StateType>()->values;
   std::vector<double> adofvals(q,q+7);
   probot_padded->SetActiveDOFValues(adofvals);
   isvalid = isvalid_now_P();
   clock_gettime(CLOCK, &toc);
   g_checktime += (toc.tv_nsec - tic.tv_nsec) + 1000000000*(toc.tv_sec - tic.tv_sec);
   return isvalid;
}

// ensure mug isnt grabbed?
bool isvalid_T(const ompl::base::State * s)
{
   struct timespec tic;
   struct timespec toc;
   bool isvalid;
   clock_gettime(CLOCK, &tic);
   kb_mug->SetTransform(ortx_from_pose(pose_mugB));
   double * q = s->as<ompl::base::RealVectorStateSpace::StateType>()->values;
   std::vector<double> adofvals(q,q+7);
   probot->SetActiveDOFValues(adofvals);
   isvalid = isvalid_now_TD();
   clock_gettime(CLOCK, &toc);
   g_checktime += (toc.tv_nsec - tic.tv_nsec) + 1000000000*(toc.tv_sec - tic.tv_sec);
   return isvalid;
}

// place mug relative to hand
bool isvalid_H(const ompl::base::State * s)
{
   struct timespec tic;
   struct timespec toc;
   bool isvalid;
   clock_gettime(CLOCK, &tic);
   double * q = s->as<ompl::base::RealVectorStateSpace::StateType>()->values;
   std::vector<double> adofvals(q,q+7);
   probot->SetActiveDOFValues(adofvals);
   kb_mug->SetTransform(
      probot->GetActiveManipulator()->GetEndEffectorTransform()
      * ortx_from_pose(pose_ee_palm)
      * ortx_from_pose(pose_mug_grasp1).inverse());
   isvalid = isvalid_now_H();
   clock_gettime(CLOCK, &toc);
   g_checktime += (toc.tv_nsec - tic.tv_nsec) + 1000000000*(toc.tv_sec - tic.tv_sec);
   return isvalid;
}

bool isvalid_D(const ompl::base::State * s)
{
   struct timespec tic;
   struct timespec toc;
   bool isvalid;
   clock_gettime(CLOCK, &tic);
   kb_mug->SetTransform(ortx_from_pose(pose_mugD));
   double * q = s->as<ompl::base::RealVectorStateSpace::StateType>()->values;
   std::vector<double> adofvals(q,q+7);
   probot->SetActiveDOFValues(adofvals);
   isvalid = isvalid_now_TD();
   clock_gettime(CLOCK, &toc);
   g_checktime += (toc.tv_nsec - tic.tv_nsec) + 1000000000*(toc.tv_sec - tic.tv_sec);
   return isvalid;
}

bool isvalid_RnT(const ompl::base::State * s)
{
   struct timespec tic;
   struct timespec toc;
   bool isvalid;
   clock_gettime(CLOCK, &tic);
   kb_mug->SetTransform(ortx_from_pose(pose_mugB));
   double * q = s->as<ompl::base::RealVectorStateSpace::StateType>()->values;
   std::vector<double> adofvals(q,q+7);
   probot->SetActiveDOFValues(adofvals);
   isvalid = isvalid_now_R() && isvalid_now_TD();
   clock_gettime(CLOCK, &toc);
   g_checktime += (toc.tv_nsec - tic.tv_nsec) + 1000000000*(toc.tv_sec - tic.tv_sec);
   return isvalid;
}

// place mug relative to hand
bool isvalid_RnH(const ompl::base::State * s)
{
   struct timespec tic;
   struct timespec toc;
   bool isvalid;
   clock_gettime(CLOCK, &tic);
   double * q = s->as<ompl::base::RealVectorStateSpace::StateType>()->values;
   std::vector<double> adofvals(q,q+7);
   probot->SetActiveDOFValues(adofvals);
   kb_mug->SetTransform(
      probot->GetActiveManipulator()->GetEndEffectorTransform()
      * ortx_from_pose(pose_ee_palm)
      * ortx_from_pose(pose_mug_grasp1).inverse());
   isvalid = isvalid_now_R() && isvalid_now_H();
   clock_gettime(CLOCK, &toc);
   g_checktime += (toc.tv_nsec - tic.tv_nsec) + 1000000000*(toc.tv_sec - tic.tv_sec);
   return isvalid;
}

bool isvalid_RnD(const ompl::base::State * s)
{
   struct timespec tic;
   struct timespec toc;
   bool isvalid;
   clock_gettime(CLOCK, &tic);
   kb_mug->SetTransform(ortx_from_pose(pose_mugD));
   double * q = s->as<ompl::base::RealVectorStateSpace::StateType>()->values;
   std::vector<double> adofvals(q,q+7);
   probot->SetActiveDOFValues(adofvals);
   isvalid = isvalid_now_R() && isvalid_now_TD();
   clock_gettime(CLOCK, &toc);
   g_checktime += (toc.tv_nsec - tic.tv_nsec) + 1000000000*(toc.tv_sec - tic.tv_sec);
   return isvalid;
}

bool isvalid_PnT(const ompl::base::State * s)
{
   struct timespec tic;
   struct timespec toc;
   bool isvalid;
   clock_gettime(CLOCK, &tic);
   kb_mug->SetTransform(ortx_from_pose(pose_mugB));
   double * q = s->as<ompl::base::RealVectorStateSpace::StateType>()->values;
   std::vector<double> adofvals(q,q+7);
   probot_padded->SetActiveDOFValues(adofvals);
   isvalid = isvalid_now_P() && isvalid_now_TD_padded();
   clock_gettime(CLOCK, &toc);
   g_checktime += (toc.tv_nsec - tic.tv_nsec) + 1000000000*(toc.tv_sec - tic.tv_sec);
   return isvalid;
}

// place mug relative to hand
bool isvalid_PnH(const ompl::base::State * s)
{
   struct timespec tic;
   struct timespec toc;
   bool isvalid;
   clock_gettime(CLOCK, &tic);
   double * q = s->as<ompl::base::RealVectorStateSpace::StateType>()->values;
   std::vector<double> adofvals(q,q+7);
   probot_padded->SetActiveDOFValues(adofvals);
   kb_mug->SetTransform(
      probot_padded->GetActiveManipulator()->GetEndEffectorTransform()
      * ortx_from_pose(pose_ee_palm)
      * ortx_from_pose(pose_mug_grasp1).inverse());
   isvalid = isvalid_now_P() && isvalid_now_H_padded();
   clock_gettime(CLOCK, &toc);
   g_checktime += (toc.tv_nsec - tic.tv_nsec) + 1000000000*(toc.tv_sec - tic.tv_sec);
   return isvalid;
}

bool isvalid_PnD(const ompl::base::State * s)
{
   struct timespec tic;
   struct timespec toc;
   bool isvalid;
   clock_gettime(CLOCK, &tic);
   kb_mug->SetTransform(ortx_from_pose(pose_mugD));
   double * q = s->as<ompl::base::RealVectorStateSpace::StateType>()->values;
   std::vector<double> adofvals(q,q+7);
   probot_padded->SetActiveDOFValues(adofvals);
   isvalid = isvalid_now_P() && isvalid_now_TD_padded();
   clock_gettime(CLOCK, &toc);
   g_checktime += (toc.tv_nsec - tic.tv_nsec) + 1000000000*(toc.tv_sec - tic.tv_sec);
   return isvalid;
}




sig_atomic_t stop = 0;
void viewermain(OpenRAVE::EnvironmentBasePtr penv)
{
  try
  {
    OpenRAVE::ViewerBasePtr viewer;
    viewer = OpenRAVE::RaveCreateViewer(penv, "qtcoin");
    penv->AddViewer(viewer);
    viewer->main(true);
  }
  catch (OpenRAVE::openrave_exception ex)
  {
    printf("Could not setup OpenRAVE viewer: %s\n", ex.what());
  }
  stop = 1;
}
void sigint_handler(int param)
{
   stop = 1;
}

int main(int argc, char * argv[])
{
   bool success;
   OpenRAVE::Transform tx;
   std::vector<ompl::base::ProblemDefinitionPtr> pdefs;
   std::vector<unsigned long long> checktimes;
   
   /* constants */
   OpenRAVE::Transform ortx_ee_palm(OpenRAVE::Vector(1,0,0,0),OpenRAVE::Vector(0,0,0.1365));
   
   printf("starting testherb ...\n");
   
   if (argc != 2)
   {
      fprintf(stderr,"Usage: %s <seed>!\n", argv[0]);
      exit(1);
   }
   
   printf("setting seed to %d ...\n", atoi(argv[1]));
   ompl::RNG::setSeed(atoi(argv[1]));
   
   /* init openrave */
   OpenRAVE::RaveInitialize(true, OpenRAVE::Level_Info); /* plugins, level */
   penv = OpenRAVE::RaveCreateEnvironment();
   //boost::thread viewerthread = boost::thread(viewermain, penv);
   
   /* set up */

   /* add some furniture */
   kb_kitchen = penv->ReadKinBodyXMLFile("environments/pr_kitchen.kinbody.xml");
   penv->Add(kb_kitchen);
   
   kb_table = penv->ReadKinBodyXMLFile("objects/furniture/table_zup.kinbody.xml");
   penv->Add(kb_table);
   kb_table->SetTransform(ortx_from_vpose(-0.3975,1.61,0., 0.,0.,-M_SQRT1_2,M_SQRT1_2));
   
   kb_bin = penv->ReadKinBodyXMLFile("objects/household/recyclingbin-zlevel.kinbody.xml");
   penv->Add(kb_bin);
   kb_bin->SetTransform(ortx_from_vpose(-1.1,2.3,0.0, 0.,0.,-M_SQRT1_2,M_SQRT1_2));
   
   kb_frame = penv->ReadKinBodyXMLFile("objects/misc/coordframe.kinbody.xml");
   penv->Add(kb_frame);
   kb_frame->Enable(false);
   
   kb_mug = penv->ReadKinBodyXMLFile("objects/household/mug2.kinbody.xml");
   penv->Add(kb_mug);
   
   /* add robot */
   probot = penv->ReadRobotXMLFile("robots/herb2_padded_nosensors.robot.xml");
   if (!probot)
   {
      printf("Tried to open robot xml \"robots/herb2_padded_nosensors.robot.xml\", but loading failed!");
      return 2;
   }
   penv->Add(probot);
   probot->SetTransform(ortx_from_vpose(-0.3975,2.38,0., 0.,0.,-M_SQRT1_2,M_SQRT1_2));
   {
      double array[] = {
         5.759, -1.972, -0.20, 1.9, 0., 0., 0., 1.3,1.3,1.3,0., /* right */
         0.630, -1.900,  0.15, 1.9, 0., 0., 0., 2.3,2.3,2.3,0.  /* left */
      };
      std::vector<double> dofvals(array, array+sizeof(array)/sizeof(array[0]));
      dofvals.resize(probot->GetDOF(), 0.0);
      probot->SetDOFValues(dofvals);
   }
   probot->SetActiveManipulator("right_wam");
   probot->SetActiveDOFs(probot->GetActiveManipulator()->GetArmIndices());
   
#if 0
   /* add padded robot, positioned/configured appropriately */
   probot_padded = penv->ReadRobotXMLFile("robots/herb2_blocky.robot.xml");
   if (!probot_padded)
   {
      printf("Tried to open robot xml \"robots/herb2_blocky.robot.xml\", but loading failed!");
      return 2;
   }
   penv->Add(probot_padded);
   probot_padded->SetTransform(probot->GetTransform());
   {
      std::vector<double> dofvals;
      probot->GetDOFValues(dofvals);
      probot_padded->SetDOFValues(dofvals);
      
   }
   probot_padded->SetActiveManipulator(probot->GetActiveManipulator()->GetName());
   probot_padded->SetActiveDOFs(probot->GetActiveDOFIndices());
   probot_padded->Enable(false);
   probot_padded->SetVisible(false);
#endif
   
   /* load ik */
   pikfast = RaveCreateModule(penv,"ikfast");
   penv->Add(pikfast,true,"");
   {
      std::stringstream ssin;
      std::stringstream ssout;
      ssin << "LoadIKFastSolver " << probot->GetName() << " Transform6D";
      success = pikfast->SendCommand(ssout,ssin);
      if (!success)
      {
         printf("could not load ikfast solver!\n");
         abort();
      }
   }
   
   //kb_frame->SetTransform(probot->GetActiveManipulator()->GetEndEffectorTransform() * ortx_from_pose(pose_ee_palm));
   //printf("y:%f\n", probot->GetActiveManipulator()->GetEndEffectorTransform().trans.y);
   
   // get ik solutions for mug in place T
   kb_mug->SetTransform(ortx_from_pose(pose_mugB));
   tx = kb_mug->GetTransform()
      * ortx_from_pose(pose_mug_grasp1)
      * ortx_from_pose(pose_ee_palm).inverse();
   std::vector< std::vector< OpenRAVE::dReal > > mugiksB;
   success = probot->GetActiveManipulator()->FindIKSolutions(tx, mugiksB, OpenRAVE::IKFO_CheckEnvCollisions);
   if (!success)
   {
      printf("no ik solutions for mugiksB found!\n");
      abort();
   }
   if (0) for (int i=0; i<(int)mugiksB.size(); i++)
   {
      printf("mugik %d/%lu\n", i+1, mugiksB.size());
      probot->SetActiveDOFValues(mugiksB[i]);
      sleep(10);
   }
   
   // get ik solutions for mug at D
   kb_mug->SetTransform(ortx_from_pose(pose_mug_drop));
   tx = kb_mug->GetTransform()
      * ortx_from_pose(pose_mug_grasp1)
      * ortx_from_pose(pose_ee_palm).inverse();
   std::vector< std::vector< OpenRAVE::dReal > > mugiksdrop;
   success = probot->GetActiveManipulator()->FindIKSolutions(tx, mugiksdrop, OpenRAVE::IKFO_CheckEnvCollisions);
   if (!success)
   {
      printf("no ik solutions for mugiksdrop found!\n");
      //abort();
   }
   if (0) for (int i=0; i<(int)mugiksdrop.size(); i++)
   {
      printf("mugik %d/%lu\n", i+1, mugiksdrop.size());
      probot->SetActiveDOFValues(mugiksdrop[i]);
      sleep(1);
   }
   
   // re-enable padded robot (we shouldnt check against it tho)
   if (probot_padded) probot_padded->Enable(true);
   
   
   
   /* space: right arm dofs */
   ompl::base::StateSpacePtr space(new ompl::base::RealVectorStateSpace(probot->GetActiveDOF()));
   /* state space set bounds */
   {
      ompl::base::RealVectorBounds bounds(probot->GetActiveDOF());
      std::vector<OpenRAVE::dReal> lowers;
		std::vector<OpenRAVE::dReal> uppers;
      probot->GetActiveDOFLimits(lowers, uppers);
      for (int i=0; i<probot->GetActiveDOF(); i++)
      {
         bounds.setLow(i, lowers[i]);
         bounds.setHigh(i, uppers[i]);
         printf("bounds for dof %d: %.20f,%.20f\n", i, lowers[i], uppers[i]);
      }
      space->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);
   }
   /* set space resolution */
   space->setLongestValidSegmentFraction(0.05 / space->getMaximumExtent());
   
   
   
#if 0 // get collision checking times
   printf("getting collision checking times ...\n");
   
   ompl::RNG rng;
   std::vector<OpenRAVE::dReal> lowers;
   std::vector<OpenRAVE::dReal> uppers;
   probot->GetActiveDOFLimits(lowers, uppers);
   struct timespec tic;
   struct timespec toc;
   std::vector<long int> times(1000);
   
   for (int i=0; i<times.size(); i++)
   {
      bool valid;

      /* get random state */
      ompl::base::ScopedState<ompl::base::RealVectorStateSpace> s(space);
      for (int j=0; j<7; j++)
         s->values[j] = rng.uniformReal(lowers[j], uppers[j]);
      
      clock_gettime(CLOCK, &tic);
      valid = isvalid_RE(s.get());
      clock_gettime(CLOCK, &toc);
      times[i] = (toc.tv_nsec - tic.tv_nsec) + 1000000000*(toc.tv_sec - tic.tv_sec);
      
      //{
      //   OpenRAVE::EnvironmentMutex::scoped_lock lock(penv->GetMutex());
      //}
      
      //printf("valid: %s\n", valid?"valid":"invalid");
      //sleep(5);
   }
   
   printf("times:\n");
   for (int i=0; i<times.size(); i++)
      printf("%ld\n",times[i]);

   return 0;
#endif
   
   
   
   
   
   /* create si for space */
   ompl::base::SpaceInformationPtr si_RS(new ompl::base::SpaceInformation(space));
   si_RS->setStateValidityChecker(&isvalid_RS);
   ompl::base::SpaceInformationPtr si_RE(new ompl::base::SpaceInformation(space));
   si_RE->setStateValidityChecker(&isvalid_RE);
   ompl::base::SpaceInformationPtr si_R(new ompl::base::SpaceInformation(space));
   si_R->setStateValidityChecker(&isvalid_R);
   ompl::base::SpaceInformationPtr si_P(new ompl::base::SpaceInformation(space));
   si_P->setStateValidityChecker(&isvalid_P);
   ompl::base::SpaceInformationPtr si_T(new ompl::base::SpaceInformation(space));
   si_T->setStateValidityChecker(&isvalid_T);
   ompl::base::SpaceInformationPtr si_H(new ompl::base::SpaceInformation(space));
   si_H->setStateValidityChecker(&isvalid_H);
   ompl::base::SpaceInformationPtr si_D(new ompl::base::SpaceInformation(space));
   si_D->setStateValidityChecker(&isvalid_D);
   ompl::base::SpaceInformationPtr si_RnT(new ompl::base::SpaceInformation(space));
   si_RnT->setStateValidityChecker(&isvalid_RnT);
   ompl::base::SpaceInformationPtr si_RnH(new ompl::base::SpaceInformation(space));
   si_RnH->setStateValidityChecker(&isvalid_RnH);
   ompl::base::SpaceInformationPtr si_RnD(new ompl::base::SpaceInformation(space));
   si_RnD->setStateValidityChecker(&isvalid_RnD);
   ompl::base::SpaceInformationPtr si_PnT(new ompl::base::SpaceInformation(space));
   si_PnT->setStateValidityChecker(&isvalid_PnT);
   ompl::base::SpaceInformationPtr si_PnH(new ompl::base::SpaceInformation(space));
   si_PnH->setStateValidityChecker(&isvalid_PnH);
   ompl::base::SpaceInformationPtr si_PnD(new ompl::base::SpaceInformation(space));
   si_PnD->setStateValidityChecker(&isvalid_PnD);
   
   // used for ONLY self-collision-checked PRM
   ompl::base::SpaceInformationPtr si_R1E(new ompl::base::SpaceInformation(space));
   si_R1E->setStateValidityChecker(&isvalid_RE);
   ompl::base::SpaceInformationPtr si_R1(new ompl::base::SpaceInformation(space));
   si_R1->setStateValidityChecker(&isvalid_R);
   
   ompl::base::SpaceInformationPtr si_R2E(new ompl::base::SpaceInformation(space));
   si_R2E->setStateValidityChecker(&isvalid_RE);
   ompl::base::SpaceInformationPtr si_R2(new ompl::base::SpaceInformation(space));
   si_R2->setStateValidityChecker(&isvalid_R);
   
   ompl::base::SpaceInformationPtr si_R3E(new ompl::base::SpaceInformation(space));
   si_R3E->setStateValidityChecker(&isvalid_RE);
   ompl::base::SpaceInformationPtr si_R3(new ompl::base::SpaceInformation(space));
   si_R3->setStateValidityChecker(&isvalid_R);
   
   
   /* problem 1: plan from start to any mugikB */
   
   ompl::base::ProblemDefinitionPtr pdef_1(new ompl::base::ProblemDefinition(si_RnT));
   {
      /* single start state */
      ompl::base::ScopedState<ompl::base::RealVectorStateSpace> s_start(space);
      s_start->values[0] =  5.759;
      s_start->values[1] = -1.972;
      s_start->values[2] = -0.22;
      s_start->values[3] =  1.9;
      s_start->values[4] =  0.;
      s_start->values[5] =  0.;
      s_start->values[6] =  0.;
      pdef_1->clearStartStates();
      pdef_1->addStartState(s_start);
      
      /* multiple goal states */
      ompl::base::GoalStates * gs = new ompl::base::GoalStates(si_RnT);
      gs->clear();
      for (int i=0; i<(int)mugiksB.size(); i++)
      {
         ompl::base::ScopedState<ompl::base::RealVectorStateSpace> s_goal(space);
         for (int j=0; j<7; j++)
            s_goal->values[j] = mugiksB[i][j];
         gs->addState(s_goal);
      }
      pdef_1->clearGoal();
      pdef_1->setGoal(ompl::base::GoalPtr(gs));
   }
   pdefs.push_back(pdef_1);
   
   /* problem 2: plan from any mugikB to any mugikdrop */
   ompl::base::ProblemDefinitionPtr pdef_2(new ompl::base::ProblemDefinition(si_RnH));
   {
      pdef_2->clearStartStates();
      for (int i=0; i<(int)mugiksB.size(); i++)
      {
         ompl::base::ScopedState<ompl::base::RealVectorStateSpace> s_start(space);
         for (int j=0; j<7; j++)
            s_start->values[j] = mugiksB[i][j];
         pdef_2->addStartState(s_start);
      }
      
      ompl::base::GoalStates * gs = new ompl::base::GoalStates(si_RnH);
      gs->clear();
      for (int i=0; i<(int)mugiksdrop.size(); i++)
      {
         ompl::base::ScopedState<ompl::base::RealVectorStateSpace> s_goal(space);
         for (int j=0; j<7; j++)
            s_goal->values[j] = mugiksdrop[i][j];
         gs->addState(s_goal);
      }
      pdef_2->clearGoal();
      pdef_2->setGoal(ompl::base::GoalPtr(gs));
   }
   pdefs.push_back(pdef_2);
   
   /* problem 3: plan from any mugikdrop back to start */
   ompl::base::ProblemDefinitionPtr pdef_3(new ompl::base::ProblemDefinition(si_RnD));
   {
      pdef_3->clearStartStates();
      for (int i=0; i<(int)mugiksdrop.size(); i++)
      {
         ompl::base::ScopedState<ompl::base::RealVectorStateSpace> s_start(space);
         for (int j=0; j<7; j++)
            s_start->values[j] = mugiksdrop[i][j];
         pdef_3->addStartState(s_start);
      }
      
      ompl::base::GoalStates * gs = new ompl::base::GoalStates(si_RnD);
      gs->clear();
      {
         ompl::base::ScopedState<ompl::base::RealVectorStateSpace> s_goal(space);
         s_goal->values[0] =  5.759;
         s_goal->values[1] = -1.972;
         s_goal->values[2] = -0.22;
         s_goal->values[3] =  1.9;
         s_goal->values[4] =  0.;
         s_goal->values[5] =  0.;
         s_goal->values[6] =  0.;
         gs->addState(s_goal);
      }
      pdef_3->clearGoal();
      pdef_3->setGoal(ompl::base::GoalPtr(gs));
   }
   pdefs.push_back(pdef_3);
   
   
   
#if PLANNER == PLANNER_MULTISET

   /* create planner */
   ompl_multiset::RoadmapPtr roadmap(
      new ompl_multiset::RoadmapSampledConst(space, 419884521, 1000, 2.0));
   ompl_multiset::CachePtr cache;
   // cache.reset(ompl_multiset::cache_create("mycache"));
   ompl_multiset::MultiSetPRM * p
      = ompl_multiset::MultiSetPRM::create(space, roadmap);
   p->set_interroot_radius(2.0);
   p->set_lambda(LAMBDA);
   
#if 0
   // costs are in nanoseconds apparently
   const double cost_RS = 28915645.0; // totally made up, maybe 3:1?
   const double cost_RE =  9638549.0; // totally made up, maybe 3:1?
   const double cost_R =  38554193.0;
   const double cost_P =   1632685.0;
   const double cost_T =    198423.5;
   const double cost_H =   1310076.5;
   const double cost_D =    197120.5;
#endif
   
#if 0
   const double cost_RS = 0.028915645; // totally made up, maybe 3:1?
   const double cost_RE = 0.009638549; // totally made up, maybe 3:1?
   const double cost_R =  0.038554193;
   const double cost_P =  0.001632685;
   const double cost_T =  0.000198423;
   const double cost_H =  0.001310076;
   const double cost_D =  0.000197120;
#endif
   
   // the actual things
   //p->add_cfree(si_RnT, "RnT", cost_R+cost_T);
   //p->add_cfree(si_RnH, "RnH", cost_R+cost_H);
   //p->add_cfree(si_RnD, "RnD", cost_R+cost_D);

   p->add_cfree(si_RnT, "RnT", (60.0e-6)*(630+19+1));
   p->add_cfree(si_RnH, "RnH", (60.0e-6)*(630+35+1));
   p->add_cfree(si_RnD, "RnD", (60.0e-6)*(630+19+1));

#if RELS == RELS_OG_ONLY

   //p->add_cfree(si_R, "R", cost_R);
   //p->add_cfree(si_T, "T", cost_T);
   //p->add_cfree(si_H, "H", cost_H);
   //p->add_cfree(si_D, "D", cost_D);
   
   p->add_cfree(si_R, "R", (60.0e-6)*(630+1));
   p->add_cfree(si_T, "T", (60.0e-6)*(19+1));
   p->add_cfree(si_H, "H", (60.0e-6)*(35+1));
   p->add_cfree(si_D, "D", (60.0e-6)*(19+1));

   p->add_intersection(si_R, si_T, si_RnT);
   p->add_intersection(si_R, si_H, si_RnH);
   p->add_intersection(si_R, si_D, si_RnD);

#endif

#if RELS == RELS_SELFCC_ONLY

   p->add_cfree(si_T, "T", cost_T);
   p->add_cfree(si_H, "H", cost_H);
   p->add_cfree(si_D, "D", cost_D);
   p->add_cfree(si_RS, "RS", cost_RS);
   p->add_cfree(si_R1E, "R1E", cost_RE);
   p->add_cfree(si_R1,  "R1",  cost_R);
   p->add_cfree(si_R2E, "R2E", cost_RE);
   p->add_cfree(si_R2,  "R2",  cost_R);
   p->add_cfree(si_R3E, "R3E", cost_RE);
   p->add_cfree(si_R3,  "R3",  cost_R);
   
   p->add_intersection(si_RS, si_R1E, si_R1);
   p->add_intersection(si_RS, si_R2E, si_R2);
   p->add_intersection(si_RS, si_R3E, si_R3);
   p->add_intersection(si_R1, si_T, si_RnT);
   p->add_intersection(si_R2, si_H, si_RnH);
   p->add_intersection(si_R3, si_D, si_RnD);

#endif

#if RELS == RELS_OG_SELFCC

   p->add_cfree(si_RS, "RS", cost_RS);
   p->add_cfree(si_RE, "RE", cost_RE);
   p->add_cfree(si_R, "R", cost_R);
   p->add_cfree(si_T, "T", cost_T);
   p->add_cfree(si_H, "H", cost_H);
   p->add_cfree(si_D, "D", cost_D);
   
   p->add_intersection(si_R, si_T, si_RnT);
   p->add_intersection(si_R, si_H, si_RnH);
   p->add_intersection(si_R, si_D, si_RnD);
   p->add_intersection(si_RS, si_RE, si_R);

#endif

#if 0 // old and busted
   p->add_cfree(si_RS, "RS", cost_RS);
   p->add_cfree(si_RE, "RE", cost_RE);
   
   // for o/g relations
   p->add_cfree(si_R, "R", cost_R);
   p->add_cfree(si_T, "T", cost_T);
   p->add_cfree(si_H, "H", cost_H);
   p->add_cfree(si_D, "D", cost_D);
   
   //p->add_cfree(si_P, "P", cost_P);
   // padded inter-step relations
   //p->add_cfree(si_PnT, "PnT", cost_P+cost_T);
   //p->add_cfree(si_PnH, "PnH", cost_P+cost_H);
   //p->add_cfree(si_PnD, "PnD", cost_P+cost_D);
   
   // for keeping steps separate, but with self-cc prm
   //p->add_cfree(si_R1E, "R1E", cost_RE);
   //p->add_cfree(si_R1,  "R1",  cost_R);
   //p->add_cfree(si_R2E, "R2E", cost_RE);
   //p->add_cfree(si_R2,  "R2",  cost_R);
   //p->add_cfree(si_R3E, "R3E", cost_RE);
   //p->add_cfree(si_R3,  "R3",  cost_R);

   p->add_intersection(si_RS, si_RE, si_R);

   //p->add_inclusion(si_R, si_P);
   
   // inter-step relations
   p->add_intersection(si_R, si_T, si_RnT);
   p->add_intersection(si_R, si_H, si_RnH);
   p->add_intersection(si_R, si_D, si_RnD);
   
   // just padded relations (no inter-step)
   //p->add_inclusion(si_RnT, si_PnT);
   //p->add_inclusion(si_RnH, si_PnH);
   //p->add_inclusion(si_RnD, si_PnD);
   
   // for keeping steps separate, but with self-cc prm
   //p->add_intersection(si_RS, si_R1E, si_R1);
   //p->add_intersection(si_RS, si_R2E, si_R2);
   //p->add_intersection(si_RS, si_R3E, si_R3);
   //p->add_intersection(si_R1, si_T, si_RnT);
   //p->add_intersection(si_R2, si_H, si_RnH);
   //p->add_intersection(si_R3, si_D, si_RnD);
#endif
   
   //p->force_batch();
   //if (cache)
   //   p->cache_load(cache);
   
#if (RELS == RELS_SELFCC_ONLY) || (RELS == RELS_OG_SELFCC)
   // check all edges for self collision! (-:
   //ompl::base::ProblemDefinitionPtr pdef_rs_bogus(new ompl::base::ProblemDefinition(si_RS));
   //p->setProblemDefinition(pdef_rs_bogus);
   //p->force_eval_everything();
   
   p->use_num_subgraphs(1);
   p->eval_everything(si_RS);
#endif

#elif (PLANNER == PLANNER_RRT) || (PLANNER == PLANNER_LBKPIECE1) || (PLANNER == PLANNER_EST)

   ompl::base::Planner * p;


#else // PLANNER

#error "planner not known!"

#endif // PLANNER
   


#define PLANNER_MULTISET 1
#define PLANNER_RRT 2
#define PLANNER_LBKPIECE1 3
   
   
   for (unsigned int pi=0; pi<pdefs.size(); pi++)
   {
      printf("solving %u/%lu ...\n",pi+1,pdefs.size());
      g_checktime = 0;
      
      struct timespec tic;
      struct timespec toc;
      
#if PLANNER == PLANNER_RRT
      p = new ompl::geometric::RRTConnect(pdefs[pi]->getSpaceInformation());
#elif PLANNER == PLANNER_LBKPIECE1
      p = new ompl::geometric::LBKPIECE1(pdefs[pi]->getSpaceInformation());
#elif PLANNER == PLANNER_EST
      p = new ompl::geometric::EST(pdefs[pi]->getSpaceInformation());
#endif
      
      p->setProblemDefinition(pdefs[pi]);
      
      clock_gettime(CLOCK, &tic);
      ompl::base::PlannerStatus status = p->solve(ompl::base::timedPlannerTerminationCondition(600.0));
      clock_gettime(CLOCK, &toc);
      unsigned long long plantime = (toc.tv_nsec - tic.tv_nsec) + 1000000000*(toc.tv_sec - tic.tv_sec);
      
      printf("planner returned: %s\n", status.asString().c_str());
      if (status != ompl::base::PlannerStatus::EXACT_SOLUTION)
      {
         fprintf(stderr,"no solution to problem 1 found after 600 seconds!\n");
         exit(1);
      }
      
#if (PLANNER == PLANNER_RRT) || (PLANNER == PLANNER_LBKPIECE1) || (PLANNER == PLANNER_EST)
      delete p;
#endif
      
      printf("g_checktime: %llu\n", g_checktime);
      printf("   plantime: %llu\n", plantime);
      checktimes.push_back(plantime);
   }
   
   if (cache)
      p->cache_save(cache);
   
   double sum;
   
   printf("      actual planning time (s):");
   sum = 0.0;
   for (unsigned int pi=0; pi<checktimes.size(); pi++)
   {
      printf(" %5.2f&s &", 1.0e-9*checktimes[pi]);
      sum += 1.0e-9*checktimes[pi];
   }
   printf(" %5.2f&s\n", sum);

   printf("returned solution length (rad):");
   sum = 0.0;
   for (unsigned int pi=0; pi<pdefs.size(); pi++)
   {
      ompl::base::PathPtr path = pdefs[pi]->getSolutionPath();
      ompl::geometric::PathGeometric * gpath = dynamic_cast<ompl::geometric::PathGeometric*>(path.get());
      printf(" %5.2f&rad &", gpath->length());
      sum += gpath->length();
   }
   printf(" %5.2f&rad\n", sum);

   if (penv->GetViewer())
   {
      printf("viewing trajs, press [Ctrl]+[C] to quit ...\n");
      signal(SIGINT, sigint_handler);
      while (!stop)
      for (unsigned int pi=0; pi<pdefs.size(); pi++)
      {
         ompl::base::PathPtr path = pdefs[pi]->getSolutionPath();
         ompl::geometric::PathGeometric * gpath = dynamic_cast<ompl::geometric::PathGeometric*>(path.get());
         OpenRAVE::TrajectoryBasePtr ptraj = OpenRAVE::RaveCreateTrajectory(penv);
         ptraj->Init(probot->GetActiveConfigurationSpecification());
         for (unsigned int ui=0; ui<gpath->getStateCount(); ui++)
         {
            ompl::base::ScopedState<ompl::base::RealVectorStateSpace> s(space);
            s = gpath->getState(ui);
            std::vector<OpenRAVE::dReal> vec(&s[0], &s[0]+probot->GetActiveDOF());
            ptraj->Insert(ui, vec);
         }
         OpenRAVE::planningutils::RetimeActiveDOFTrajectory(ptraj,probot,false,1.0,1.0,"","");
         
         /* place mug */
         switch (pi)
         {
         case 0:
            kb_mug->SetTransform(ortx_from_pose(pose_mugB));
            break;
         case 1:
            kb_mug->SetTransform(
               probot->GetActiveManipulator()->GetEndEffectorTransform()
               * ortx_from_pose(pose_ee_palm)
               * ortx_from_pose(pose_mug_grasp1).inverse());
            probot->Grab(kb_mug);
            break;
         case 2:
            kb_mug->SetTransform(ortx_from_pose(pose_mugD));
            break;
         }
         
         probot->GetController()->SetPath(ptraj);
         while (!stop)
         {
            if (probot->GetController()->IsDone())
               break;
            sleep(1);
         }
         
         probot->Release(kb_mug);
         
         sleep(1);
      }
   }
   
   printf("destroying ...\n");
   penv->Destroy();
   OpenRAVE::RaveDestroy();
}
