/* File: planner_e8.h
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

namespace or_multiset
{

// this only works for real vector state spaces
class OrChecker: public ompl::base::StateValidityChecker
{
public:
   const OpenRAVE::EnvironmentBasePtr env;
   const OpenRAVE::RobotBasePtr robot;
   const size_t dim;
   mutable size_t num_checks;
   OrChecker(
      const ompl::base::SpaceInformationPtr & si,
      const OpenRAVE::EnvironmentBasePtr env,
      const OpenRAVE::RobotBasePtr robot,
      const size_t dim):
         ompl::base::StateValidityChecker(si),
         env(env), robot(robot), dim(dim), num_checks(0)
   {
   }
   bool isValid(const ompl::base::State * state) const
   {
      num_checks++;
      double * q = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
      std::vector<OpenRAVE::dReal> adofvals(q, q+dim);
      robot->SetActiveDOFValues(adofvals, OpenRAVE::KinBody::CLA_Nothing);
      bool collided = env->CheckCollision(robot) || robot->CheckSelfCollision();
      return !collided;
   }
};

class E8Roadmap : public OpenRAVE::PlannerBase
{
public:
   
   // <startstate> and <goalstate> can be specified multiple times
   class PlannerParameters : public OpenRAVE::PlannerBase::PlannerParameters
   {
   public:
      std::string roadmap_id;
      double coeff_distance;
      double coeff_checkcost;
      double coeff_subgraph;
      PlannerParameters();
   private:
      std::string el_deserializing;
      bool serialize(std::ostream& sout, int options) const;
      OpenRAVE::BaseXMLReader::ProcessElement startElement(
         const std::string & name, const OpenRAVE::AttributesList & atts);
      bool endElement(const std::string & name);
   };
   typedef boost::shared_ptr<PlannerParameters> PlannerParametersPtr;
   typedef boost::shared_ptr<PlannerParameters const> PlannerParametersConstPtr;

   const OpenRAVE::EnvironmentBasePtr env;
   PlannerParametersConstPtr params;
   OpenRAVE::RobotBasePtr robot;
   std::vector<int> robot_adofs;
   ompl::base::StateSpacePtr ompl_space;
   ompl::base::SpaceInformationPtr ompl_si;
   boost::shared_ptr<OrChecker> ompl_checker;
   boost::shared_ptr<ompl_multiset::SimpleEffortModel> sem;
   ompl_multiset::E8Roadmap::RoadmapGenPtr roadmapgen;
   boost::shared_ptr<ompl_multiset::E8Roadmap> ompl_planner;
   ompl::base::ProblemDefinitionPtr ompl_pdef;
   
   E8Roadmap(OpenRAVE::EnvironmentBasePtr env);
   ~E8Roadmap();
   
   bool InitPlan(OpenRAVE::RobotBasePtr robot, std::istream & isParameters);
   bool InitPlan(OpenRAVE::RobotBasePtr robot, OpenRAVE::PlannerBase::PlannerParametersConstPtr params);
   OpenRAVE::PlannerBase::PlannerParametersConstPtr GetParameters() const;
   
   OpenRAVE::PlannerStatus PlanPath(OpenRAVE::TrajectoryBasePtr);
   
   bool GetTimes(std::ostream & sout, std::istream & sin) const;
   
};

} // namespace or_multiset
