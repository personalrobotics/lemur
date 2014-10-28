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

#include <checkmask/graph.h>

OpenRAVE::Transform ortx_from_pose(double pose[7])
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

bool isvalid_x(const ompl::base::State * s)
{
   double * q = s->as<ompl::base::RealVectorStateSpace::StateType>()->values;
   std::vector<double> adofvals(q,q+7);
   probot->SetActiveDOFValues(adofvals);
   if (penv->CheckSelfCollision(probot)) return false;
   if (penv->CheckCollision(probot, kb_kitchen)) return false;
   if (penv->CheckCollision(probot, kb_table)) return false;
   if (penv->CheckCollision(probot, kb_bin)) return false;
   if (penv->CheckCollision(probot, kb_mug)) return false;
   return true;
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

int main()
{
   bool success;
   
   /* constants */
   double pose_ee_palm[7] = { 0., 0., 0.1365, 0., 0., 0., 1. };
   double pose_mug_grasp1[7] = { 0.15, 0., 0.09, -0.5, -0.5, 0.5, 0.5 };
   //OpenRAVE::Transform ortx_ee_palm(OpenRAVE::Vector(1,0,0,0),OpenRAVE::Vector(0,0,0.1365));
   
   printf("starting testherb ...\n");
   
   /* init openrave */
   OpenRAVE::RaveInitialize(true, OpenRAVE::Level_Info); /* plugins, level */
   penv = OpenRAVE::RaveCreateEnvironment();
   boost::thread viewerthread = boost::thread(viewermain, penv);
   
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
   kb_mug->SetTransform(ortx_from_vpose(-0.3975,1.61,0.735, 0.,0.,1.,0.));
   
   
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
   
   kb_frame->SetTransform(probot->GetActiveManipulator()->GetEndEffectorTransform() * ortx_from_pose(pose_ee_palm));
   //printf("y:%f\n", probot->GetActiveManipulator()->GetEndEffectorTransform().trans.y);
   
   // grab the mug
   OpenRAVE::Transform tx
      = kb_mug->GetTransform()
      * ortx_from_pose(pose_mug_grasp1)
      * ortx_from_pose(pose_ee_palm).inverse();
   
   std::vector< std::vector< OpenRAVE::dReal > > mugiks;
   success = probot->GetActiveManipulator()->FindIKSolutions(tx, mugiks, OpenRAVE::IKFO_CheckEnvCollisions);
   if (!success)
   {
      printf("no ik solutions found!\n");
      abort();
   }
   if (0) for (int i=0; i<(int)mugiks.size(); i++)
   {
      printf("mugik %d/%lu\n", i+1, mugiks.size());
      probot->SetActiveDOFValues(mugiks[i]);
      sleep(10);
   }
   
   
   
   
   /* plan from start to any mugik */
   
   /* space: right arm dofs */
   ompl::base::StateSpacePtr space(new ompl::base::RealVectorStateSpace(probot->GetActiveDOF()));
   /* state space set bounds */
   {
      ompl::base::RealVectorBounds bounds(probot->GetActiveDOF());
      std::vector<OpenRAVE::dReal> lowers;
		std::vector<OpenRAVE::dReal> uppers;
      probot->GetDOFLimits(lowers, uppers);
      for (int i=0; i<probot->GetActiveDOF(); i++)
         { bounds.setLow(i, lowers[i]); bounds.setHigh(i, uppers[i]); }
      space->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);
   }
   
   /* create planner (with dummy si based on our space) */
   checkmask::GraphPlanner * p = checkmask::GraphPlanner::create(space);
   
   /* create basic si */
   ompl::base::SpaceInformationPtr si_x(new ompl::base::SpaceInformation(space));
   si_x->setStateValidityChecker(&isvalid_x);
   p->add_cfree(si_x, "x", 10.0);
   
   ompl::base::ProblemDefinitionPtr pdef_x(new ompl::base::ProblemDefinition(si_x));
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
      pdef_x->clearStartStates();
      pdef_x->addStartState(s_start);
      
      /* multiple goal states */
      ompl::base::GoalStates * gs = new ompl::base::GoalStates(si_x);
      gs->clear();
      for (int i=0; i<(int)mugiks.size(); i++)
      {
         ompl::base::ScopedState<ompl::base::RealVectorStateSpace> s_goal(space);
         s_goal->values[0] = mugiks[i][0];
         s_goal->values[1] = mugiks[i][1];
         s_goal->values[2] = mugiks[i][2];
         s_goal->values[3] = mugiks[i][3];
         s_goal->values[4] = mugiks[i][4];
         s_goal->values[5] = mugiks[i][5];
         s_goal->values[6] = mugiks[i][6];
         gs->addState(s_goal);
      }
      pdef_x->clearGoal();
      pdef_x->setGoal(ompl::base::GoalPtr(gs));
   }
   
   printf("solving ...\n");
   p->setProblemDefinition(pdef_x);
   ompl::base::PlannerStatus status = p->solve(ompl::base::timedPlannerTerminationCondition(600.0));
   printf("planner returned: %s\n", status.asString().c_str());
   if (status != ompl::base::PlannerStatus::EXACT_SOLUTION)
   {
      printf("no solution found after 600 seconds!\n");
      abort();
   }
   
   ompl::base::PathPtr path = pdef_x->getSolutionPath();
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
   
   printf("viewing traj, press [Ctrl]+[C] to quit ...\n");
   signal(SIGINT, sigint_handler);
   while (!stop)
   {
      probot->GetController()->SetPath(ptraj);
      while (!stop)
      {
         if (probot->GetController()->IsDone())
            break;
         sleep(1);
      }
   }
   
   printf("destroying ...\n");
   penv->Destroy();
   OpenRAVE::RaveDestroy();
}
