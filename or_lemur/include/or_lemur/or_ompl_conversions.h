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
// with baking support
class OrChecker: public ompl::base::StateValidityChecker
{
public:
   const OpenRAVE::EnvironmentBasePtr env;
   const OpenRAVE::RobotBasePtr robot;
   const size_t dim;
   mutable size_t num_checks;
   mutable boost::chrono::high_resolution_clock::duration dur_checks;
   // optional baked stuff
   const bool do_baked;
   boost::function<void ()> bake_begin;
   boost::function<OpenRAVE::KinBodyPtr ()> bake_end;
   boost::function<bool (OpenRAVE::KinBodyConstPtr, OpenRAVE::CollisionReportPtr)> baked_checker;
   OpenRAVE::KinBodyPtr baked_kinbody;
   OrChecker(
      const ompl::base::SpaceInformationPtr & si,
      const OpenRAVE::EnvironmentBasePtr env,
      const OpenRAVE::RobotBasePtr robot,
      const size_t dim,
      const bool do_baked):
         ompl::base::StateValidityChecker(si),
         env(env), robot(robot), dim(dim),
         num_checks(0),
         dur_checks(),
         do_baked(do_baked)
   {
      if (si->getStateSpace()->getType() != ompl::base::STATE_SPACE_REAL_VECTOR)
      {
         throw std::runtime_error("state space is not real vector!");
      }
      
      if (do_baked)
      {
         OpenRAVE::CollisionCheckerBasePtr cc = env->GetCollisionChecker();
         
         std::stringstream sinput;
         std::stringstream soutput;
         sinput << "GetBakingFunctions";
         try
         {
            if (!cc->SendCommand(soutput, sinput))
               throw std::runtime_error("collision checker doesn't support baked checks!");
         }
         catch (const OpenRAVE::openrave_exception & exc)
         {
            throw std::runtime_error("collision checker doesn't support baked checks!");
         }
         
         boost::function< void ()> * fn_bake_begin;
         boost::function< OpenRAVE::KinBodyPtr ()> * fn_bake_end;
         boost::function< bool (OpenRAVE::KinBodyConstPtr, OpenRAVE::CollisionReportPtr)> * fn_check_baked_collision;
         soutput >> (void *&)fn_bake_begin;
         soutput >> (void *&)fn_bake_end;
         soutput >> (void *&)fn_check_baked_collision;
         bake_begin = *fn_bake_begin;
         bake_end = *fn_bake_end;
         baked_checker = *fn_check_baked_collision;
      }
      
   }
   void start()
   {
      if (do_baked)
      {
         bake_begin();
         env->CheckCollision(robot);
         robot->CheckSelfCollision();
         baked_kinbody = bake_end();
      }
   }
   void stop()
   {
      baked_kinbody.reset();
   }
   bool isValid(const ompl::base::State * state) const
   {
      boost::chrono::high_resolution_clock::time_point time_begin
         = boost::chrono::high_resolution_clock::now();
      num_checks++;
      double * q = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
      std::vector<OpenRAVE::dReal> adofvals(q, q+dim);
      robot->SetActiveDOFValues(adofvals, OpenRAVE::KinBody::CLA_Nothing);
      bool collided;
      if (do_baked)
         collided = baked_checker(baked_kinbody, OpenRAVE::CollisionReportPtr());
      else
         collided = env->CheckCollision(robot) || robot->CheckSelfCollision();
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
