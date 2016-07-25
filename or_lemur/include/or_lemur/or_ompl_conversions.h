/*! \file or_ompl_conversions.h
 * \author Chris Dellin <cdellin@gmail.com>
 * \copyright 2015 Carnegie Mellon University
 * \copyright License: BSD
 */

namespace or_lemur
{

double ompl_resolution(
   const OpenRAVE::RobotBasePtr & robot,
   const std::vector<int> & dofs);

bool ompl_set_roots(ompl::base::ProblemDefinitionPtr ompl_pdef,
   OpenRAVE::PlannerBase::PlannerParametersConstPtr params);

/*! \brief Create an OMPL space/spaceinfo (and accompanying validity
 *         checker)
 * 
 * The validity checker is guaranteed to be a derived class of
 * or_lemur::OrChecker.
 * 
 * If do_checker is set, space_info will be setup().
 */
bool create_space(
   const OpenRAVE::RobotBasePtr & robot,
   const std::vector<int> & dofs,
   bool do_checker,
   bool do_baked,
   ompl::base::SpaceInformationPtr & out_space_info);

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
   
   OpenRAVE::CollisionCheckerBasePtr baked_cc;
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
   }
   void start()
   {
      if (!do_baked)
         return;
      // handle baked case
      OpenRAVE::CollisionCheckerBasePtr cc = env->GetCollisionChecker();
      if (!cc)
         throw std::runtime_error("no collision checker set!");
      // communicate with baked checker
      bool success;
      std::stringstream sinput("BakeGetType BakeBegin BakeEnd"), soutput;
      try
      {
         success = cc->SendCommand(soutput, sinput); // BakeGetType
      }
      catch (const OpenRAVE::openrave_exception & exc)
      {
         throw std::runtime_error("collision checker doesn't support baked checks!");
      }
      // start baking
      if (!success) std::runtime_error("BakeGetType failed!");
      std::string kinbody_type = soutput.str();
      success = cc->SendCommand(soutput, sinput); // BakeBegin
      if (!success) std::runtime_error("BakeBegin failed!");
      OpenRAVE::KinBodyPtr kb = OpenRAVE::RaveCreateKinBody(env,kinbody_type);
      if (!baked_kinbody) std::runtime_error("RaveCreateKinBody failed!");
      env->CheckCollision(robot);
      robot->CheckSelfCollision();
      success = cc->SendCommand(soutput, sinput); // BakeEnd
      if (!success) std::runtime_error("BakeEnd failed!");
      // success
      baked_cc = cc;
      baked_kinbody = kb;
   }
   void stop()
   {
      baked_cc.reset();
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
         collided = baked_cc->CheckStandaloneSelfCollision(baked_kinbody);
      else
         collided = env->CheckCollision(robot) || robot->CheckSelfCollision();
      dur_checks += boost::chrono::high_resolution_clock::now() - time_begin;
      return !collided;
   }
};

#ifdef OR_LEMUR_HAS_BOOSTSMARTPTRS
typedef boost::shared_ptr<OrChecker> OrCheckerPtr;
#else
typedef std::shared_ptr<OrChecker> OrCheckerPtr;
#endif

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

typedef boost::shared_ptr<OrIndicatorChecker> OrIndicatorCheckerPtr;

} // namespace or_lemur
