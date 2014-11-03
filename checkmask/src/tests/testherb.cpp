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
#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include <checkmask/graph.h>

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

unsigned long long checktime;

/* spaces:
 * A: (robot) x (robot+kitchen+table+bin)
 * B: (robot) x (mug-on-table)
 * C: (mug-in-hand) x (robot+kitchen+table+bin)
 * D: (robot) x (mug-in-bin)
 *
 * step 1, in AnB
 * step 2, in AnC
 * step 3, in AnD
 */

// this is regardless of the mug location and grabbed state
bool isvalid_now_A(void)
{
   //if (probot->CheckSelfCollision()) return false;
   if (penv->CheckSelfCollision(probot)) return false; // ignores grabbed bodies
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

bool isvalid_now_BD(void)
{
   if (penv->CheckCollision(probot, kb_mug)) return false;
   return true;
}

bool isvalid_now_C(void)
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

bool isvalid_A(const ompl::base::State * s)
{
   struct timespec tic;
   struct timespec toc;
   bool isvalid;
   clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &tic);
   // regardless of mug location
   double * q = s->as<ompl::base::RealVectorStateSpace::StateType>()->values;
   std::vector<double> adofvals(q,q+7);
   probot->SetActiveDOFValues(adofvals);
   isvalid = isvalid_now_A();
   clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &toc);
   checktime += (toc.tv_nsec - tic.tv_nsec) + 1000000000*(toc.tv_sec - tic.tv_sec);
   return isvalid;
}

// ensure mug isnt grabbed?
bool isvalid_B(const ompl::base::State * s)
{
   struct timespec tic;
   struct timespec toc;
   bool isvalid;
   clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &tic);
   kb_mug->SetTransform(ortx_from_pose(pose_mugB));
   double * q = s->as<ompl::base::RealVectorStateSpace::StateType>()->values;
   std::vector<double> adofvals(q,q+7);
   probot->SetActiveDOFValues(adofvals);
   isvalid = isvalid_now_BD();
   clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &toc);
   checktime += (toc.tv_nsec - tic.tv_nsec) + 1000000000*(toc.tv_sec - tic.tv_sec);
   return isvalid;
}

// place mug relative to hand
bool isvalid_C(const ompl::base::State * s)
{
   struct timespec tic;
   struct timespec toc;
   bool isvalid;
   clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &tic);
   double * q = s->as<ompl::base::RealVectorStateSpace::StateType>()->values;
   std::vector<double> adofvals(q,q+7);
   probot->SetActiveDOFValues(adofvals);
   kb_mug->SetTransform(
      probot->GetActiveManipulator()->GetEndEffectorTransform()
      * ortx_from_pose(pose_ee_palm)
      * ortx_from_pose(pose_mug_grasp1).inverse());
   isvalid = isvalid_now_C();
   clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &toc);
   checktime += (toc.tv_nsec - tic.tv_nsec) + 1000000000*(toc.tv_sec - tic.tv_sec);
   return isvalid;
}

bool isvalid_D(const ompl::base::State * s)
{
   struct timespec tic;
   struct timespec toc;
   bool isvalid;
   clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &tic);
   kb_mug->SetTransform(ortx_from_pose(pose_mugD));
   double * q = s->as<ompl::base::RealVectorStateSpace::StateType>()->values;
   std::vector<double> adofvals(q,q+7);
   probot->SetActiveDOFValues(adofvals);
   isvalid = isvalid_now_BD();
   clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &toc);
   checktime += (toc.tv_nsec - tic.tv_nsec) + 1000000000*(toc.tv_sec - tic.tv_sec);
   return isvalid;
}

bool isvalid_AnB(const ompl::base::State * s)
{
   struct timespec tic;
   struct timespec toc;
   bool isvalid;
   clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &tic);
   kb_mug->SetTransform(ortx_from_pose(pose_mugB));
   double * q = s->as<ompl::base::RealVectorStateSpace::StateType>()->values;
   std::vector<double> adofvals(q,q+7);
   probot->SetActiveDOFValues(adofvals);
   isvalid = isvalid_now_A() && isvalid_now_BD();
   clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &toc);
   checktime += (toc.tv_nsec - tic.tv_nsec) + 1000000000*(toc.tv_sec - tic.tv_sec);
   return isvalid;
}

// place mug relative to hand
bool isvalid_AnC(const ompl::base::State * s)
{
   struct timespec tic;
   struct timespec toc;
   bool isvalid;
   clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &tic);
   double * q = s->as<ompl::base::RealVectorStateSpace::StateType>()->values;
   std::vector<double> adofvals(q,q+7);
   probot->SetActiveDOFValues(adofvals);
   kb_mug->SetTransform(
      probot->GetActiveManipulator()->GetEndEffectorTransform()
      * ortx_from_pose(pose_ee_palm)
      * ortx_from_pose(pose_mug_grasp1).inverse());
   isvalid = isvalid_now_A() && isvalid_now_C();
   clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &toc);
   checktime += (toc.tv_nsec - tic.tv_nsec) + 1000000000*(toc.tv_sec - tic.tv_sec);
   return isvalid;
}

bool isvalid_AnD(const ompl::base::State * s)
{
   struct timespec tic;
   struct timespec toc;
   bool isvalid;
   clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &tic);
   kb_mug->SetTransform(ortx_from_pose(pose_mugD));
   double * q = s->as<ompl::base::RealVectorStateSpace::StateType>()->values;
   std::vector<double> adofvals(q,q+7);
   probot->SetActiveDOFValues(adofvals);
   isvalid = isvalid_now_A() && isvalid_now_BD();
   clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &toc);
   checktime += (toc.tv_nsec - tic.tv_nsec) + 1000000000*(toc.tv_sec - tic.tv_sec);
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
   
   printf("setting seed ...\n");
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
      printf("Tried to open robot xml \"robots/barrettwam.robot.xml\", but loading failed!");
      return 2;
   }
   penv->Add(probot);
   probot->SetTransform(ortx_from_vpose(-0.3975,2.38,0., 0.,0.,-M_SQRT1_2,M_SQRT1_2));
   {
      double array[] = {
         5.759, -1.972, -0.2, 1.9, 0., 0., 0., 0., 0., 0., 0.,
         0.524, -1.972,  0.2, 1.9, 0., 0., 0., 2.3,2.3,2.3,0.
      };
      std::vector<double> dofvals(array, array+sizeof(array)/sizeof(array[0]));
      dofvals.resize(probot->GetDOF(), 0.0);
      probot->SetDOFValues(dofvals);
   }
   probot->SetActiveManipulator("right_wam");
   probot->SetActiveDOFs(probot->GetActiveManipulator()->GetArmIndices());
   
   
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
   
   // get ik solutions for mug in place B
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
   
   // get ik solutions for mug at drop
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
   
   
   
   
   /* space: right arm dofs */
   ompl::base::StateSpacePtr space(new ompl::base::RealVectorStateSpace(probot->GetActiveDOF()));
   /* state space set bounds */
   {
      ompl::base::RealVectorBounds bounds(probot->GetActiveDOF());
      std::vector<OpenRAVE::dReal> lowers;
		std::vector<OpenRAVE::dReal> uppers;
      probot->GetActiveDOFLimits(lowers, uppers);
      for (int i=0; i<probot->GetActiveDOF(); i++)
         { bounds.setLow(i, lowers[i]); bounds.setHigh(i, uppers[i]); }
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
      
      clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &tic);
      valid = isvalid_D(s.get());
      clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &toc);
      times[i] = (toc.tv_nsec - tic.tv_nsec) + 1000000000*(toc.tv_sec - tic.tv_sec);
      
      //{
      //   OpenRAVE::EnvironmentMutex::scoped_lock lock(penv->GetMutex());
      //}
      
      //printf("valid: %s\n", valid?"valid":"invalid");
      //sleep(1);
   }
   
   printf("times:\n");
   for (int i=0; i<times.size(); i++)
      printf("%ld\n",times[i]);

   return 0;
#endif
   
   
   
   
   
   /* create si for space */
   ompl::base::SpaceInformationPtr si_A(new ompl::base::SpaceInformation(space));
   si_A->setStateValidityChecker(&isvalid_A);
   ompl::base::SpaceInformationPtr si_B(new ompl::base::SpaceInformation(space));
   si_B->setStateValidityChecker(&isvalid_B);
   ompl::base::SpaceInformationPtr si_C(new ompl::base::SpaceInformation(space));
   si_C->setStateValidityChecker(&isvalid_C);
   ompl::base::SpaceInformationPtr si_D(new ompl::base::SpaceInformation(space));
   si_D->setStateValidityChecker(&isvalid_D);
   ompl::base::SpaceInformationPtr si_AnB(new ompl::base::SpaceInformation(space));
   si_AnB->setStateValidityChecker(&isvalid_AnB);
   ompl::base::SpaceInformationPtr si_AnC(new ompl::base::SpaceInformation(space));
   si_AnC->setStateValidityChecker(&isvalid_AnC);
   ompl::base::SpaceInformationPtr si_AnD(new ompl::base::SpaceInformation(space));
   si_AnD->setStateValidityChecker(&isvalid_AnD);
   
   /* problem 1: plan from start to any mugikB */
   
   ompl::base::ProblemDefinitionPtr pdef_1(new ompl::base::ProblemDefinition(si_AnB));
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
      ompl::base::GoalStates * gs = new ompl::base::GoalStates(si_AnB);
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
   ompl::base::ProblemDefinitionPtr pdef_2(new ompl::base::ProblemDefinition(si_AnC));
   {
      pdef_2->clearStartStates();
      for (int i=0; i<(int)mugiksB.size(); i++)
      {
         ompl::base::ScopedState<ompl::base::RealVectorStateSpace> s_start(space);
         for (int j=0; j<7; j++)
            s_start->values[j] = mugiksB[i][j];
         pdef_2->addStartState(s_start);
      }
      
      ompl::base::GoalStates * gs = new ompl::base::GoalStates(si_AnC);
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
   ompl::base::ProblemDefinitionPtr pdef_3(new ompl::base::ProblemDefinition(si_AnD));
   {
      pdef_3->clearStartStates();
      for (int i=0; i<(int)mugiksdrop.size(); i++)
      {
         ompl::base::ScopedState<ompl::base::RealVectorStateSpace> s_start(space);
         for (int j=0; j<7; j++)
            s_start->values[j] = mugiksdrop[i][j];
         pdef_3->addStartState(s_start);
      }
      
      ompl::base::GoalStates * gs = new ompl::base::GoalStates(si_AnD);
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
   
   
   
   
   
#if 1

   /* create planner (with dummy si based on our space) */
   checkmask::GraphPlanner * p = checkmask::GraphPlanner::create(space);
   p->set_radius(2.0);
   p->set_resolution(0.05);
   p->set_batchsize(1000);
   
   const double cost_A = 38554193.0;
   const double cost_B =   198423.5;
   const double cost_C =  1310076.5;
   const double cost_D =   197120.5;
   
   p->add_cfree(si_A, "A", cost_A);
   p->add_cfree(si_B, "B", cost_B);
   p->add_cfree(si_C, "C", cost_C);
   p->add_cfree(si_D, "D", cost_D);
   p->add_cfree(si_AnB, "AnB", cost_A+cost_B);
   p->add_cfree(si_AnC, "AnC", cost_A+cost_C);
   p->add_cfree(si_AnD, "AnD", cost_A+cost_D);
   
   //p->add_intersection(si_A, si_B, si_AnB);
   //p->add_intersection(si_A, si_C, si_AnC);
   //p->add_intersection(si_A, si_D, si_AnD);
   
#else // rrt

   ompl::base::Planner * p = new ompl::geometric::RRTConnect(pdef_1);
   
#endif
   
   
   
   
   for (unsigned int pi=0; pi<pdefs.size(); pi++)
   {
      printf("solving %d/%d ...\n",pi+1,pdefs.size());
      checktime = 0;
      p->setProblemDefinition(pdefs[pi]);
      ompl::base::PlannerStatus status = p->solve(ompl::base::timedPlannerTerminationCondition(600.0));
      printf("planner returned: %s\n", status.asString().c_str());
      if (status != ompl::base::PlannerStatus::EXACT_SOLUTION)
      {
         fprintf(stderr,"no solution to problem 1 found after 600 seconds!\n");
         exit(1);
      }
      printf("checktime: %llu\n", checktime);
      checktimes.push_back(checktime);
   }


   printf("checktimes:");
   for (unsigned int pi=0; pi<checktimes.size(); pi++)
      printf(" %llu", checktimes[pi]);
   printf("\n");

#if 0
   
   printf("viewing trajs, press [Ctrl]+[C] to quit ...\n");
   signal(SIGINT, sigint_handler);
   while (!stop)
   for (unsigned int pi=0; pi<pdefs.size(); pi++)
   {
      ompl::base::PathPtr path = pdefs[pi]->getSolutionPath();
      ompl::geometric::PathGeometric * gpath = dynamic_cast<ompl::geometric::PathGeometric*>(path.get());
      OpenRAVE::TrajectoryBasePtr ptraj = OpenRAVE::RaveCreateTrajectory(penv);
      ptraj->Init(probot->GetActiveConfigurationSpecification());
      for (int i=0; i<gpath->getStateCount(); i++)
      {
         ompl::base::ScopedState<ompl::base::RealVectorStateSpace> s(space);
         s = gpath->getState(i);
         std::vector<OpenRAVE::dReal> vec(&s[0], &s[0]+probot->GetActiveDOF());
         ptraj->Insert(i, vec);
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
#endif
   
   printf("destroying ...\n");
   penv->Destroy();
   OpenRAVE::RaveDestroy();
}
