/* File: or_checker.h
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

} // namespace or_multiset
