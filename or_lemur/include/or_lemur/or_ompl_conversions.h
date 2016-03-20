/*! \file or_ompl_conversions.h
 * \author Chris Dellin <cdellin@gmail.com>
 * \copyright 2015 Carnegie Mellon University
 * \copyright License: BSD
 */

namespace or_lemur
{

ompl::base::RealVectorBounds ompl_bounds(OpenRAVE::RobotBasePtr robot);

double ompl_resolution(OpenRAVE::RobotBasePtr robot);

bool ompl_set_roots(ompl::base::ProblemDefinitionPtr ompl_pdef,
   OpenRAVE::PlannerBase::PlannerParametersConstPtr params);

// this only works for real vector state spaces
class OrChecker: public ompl::base::StateValidityChecker
{
public:
   const OpenRAVE::EnvironmentBasePtr env;
   const OpenRAVE::RobotBasePtr robot;
   const size_t dim;
   mutable size_t num_checks;
   mutable boost::chrono::high_resolution_clock::duration dur_checks;
   OrChecker(
      const ompl::base::SpaceInformationPtr & si,
      const OpenRAVE::EnvironmentBasePtr env,
      const OpenRAVE::RobotBasePtr robot,
      const size_t dim):
         ompl::base::StateValidityChecker(si),
         env(env), robot(robot), dim(dim),
         num_checks(0),
         dur_checks()
   {
   }
   bool isValid(const ompl::base::State * state) const
   {
      boost::chrono::high_resolution_clock::time_point time_begin
         = boost::chrono::high_resolution_clock::now();
      num_checks++;
      double * q = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
      std::vector<OpenRAVE::dReal> adofvals(q, q+dim);
      robot->SetActiveDOFValues(adofvals, OpenRAVE::KinBody::CLA_Nothing);
      bool collided = env->CheckCollision(robot) || robot->CheckSelfCollision();
      dur_checks += boost::chrono::high_resolution_clock::now() - time_begin;
      return !collided;
   }
};

class OrIndicatorChecker: public ompl::base::StateValidityChecker
{
public:
   boost::function<bool (std::vector<OpenRAVE::dReal> &)> indicator;
   const size_t dim;
   mutable size_t num_checks;
   mutable boost::chrono::high_resolution_clock::duration dur_checks;
   OrIndicatorChecker(
      const ompl::base::SpaceInformationPtr & si,
      boost::function<bool (std::vector<OpenRAVE::dReal> &)> indicator):
      ompl::base::StateValidityChecker(si),
      indicator(indicator), dim(si->getStateDimension()),
      num_checks(0), dur_checks()
   {
   }
   bool isValid(const ompl::base::State * state) const
   {
      boost::chrono::high_resolution_clock::time_point time_begin
         = boost::chrono::high_resolution_clock::now();
      num_checks++;
      double * q = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
      std::vector<OpenRAVE::dReal> adofvals(q, q+dim);
      bool is_valid = indicator(adofvals);
      dur_checks += boost::chrono::high_resolution_clock::now() - time_begin;
      return is_valid;
   }
};

} // namespace or_lemur
