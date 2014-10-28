#include <openrave/openrave.h>

#include "planner_checkmask.h"

checkmask::OmplCheckMask::OmplCheckMask(OpenRAVE::EnvironmentBasePtr penv):
   OpenRAVE::PlannerBase(penv)
{
   printf("constructed!\n");
}

checkmask::OmplCheckMask::~OmplCheckMask()
{
   printf("destructed!\n");
}

bool checkmask::OmplCheckMask::InitPlan(OpenRAVE::RobotBasePtr pbase, std::istream & isParameters)
{
   throw OpenRAVE::openrave_exception("initplan not yet implemented!");
#if 0
   RAVELOG_WARN(str(boost::format("using default planner parameters structure to de-serialize parameters data inside %s, information might be lost!! Please define a InitPlan(robot,stream) function!\n")%GetXMLId()));
   boost::shared_ptr<PlannerParameters> localparams(new PlannerParameters());
   isParameters >> *localparams;
   localparams->Validate();
   return InitPlan(pbase,localparams);
#endif
}

bool checkmask::OmplCheckMask::InitPlan(OpenRAVE::RobotBasePtr robot, OpenRAVE::PlannerBase::PlannerParametersConstPtr params)
{
   throw OpenRAVE::openrave_exception(
      "OmplCheckMask planner does not support default PlannerParameters structure!");
}

OpenRAVE::PlannerStatus checkmask::OmplCheckMask::PlanPath(OpenRAVE::TrajectoryBasePtr ptraj)
{
   return OpenRAVE::PS_Failed;
}

OpenRAVE::PlannerBase::PlannerParametersConstPtr checkmask::OmplCheckMask::GetParameters() const
{
   return OpenRAVE::PlannerBase::PlannerParametersConstPtr();
}
