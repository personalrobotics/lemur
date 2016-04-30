/*! \file link_indicators.h
 * \author Chris Dellin <cdellin@gmail.com>
 * \copyright 2015 Carnegie Mellon University
 * \copyright License: BSD
 */

namespace or_lemur
{

class AbortingIndicator
{
public:
   AbortingIndicator() {}
   bool operator()(const std::vector<OpenRAVE::dReal> & values) const
   {
      abort();
   }
};

class BakedCheckIndicator
{
public:
   const OpenRAVE::RobotBasePtr _robot;
   const OpenRAVE::CollisionCheckerBasePtr _baked_checker;
   const OpenRAVE::KinBodyConstPtr _baked_check;
   BakedCheckIndicator(const OpenRAVE::RobotBasePtr & robot,
      const OpenRAVE::CollisionCheckerBasePtr baked_checker,
      const OpenRAVE::KinBodyConstPtr baked_check):
      _robot(robot), _baked_checker(baked_checker), _baked_check(baked_check)
   {
   }
   bool operator()(const std::vector<OpenRAVE::dReal> & values) const
   {
      _robot->SetActiveDOFValues(values);
      bool collides = _baked_checker->CheckStandaloneSelfCollision(_baked_check);
      if (collides)
         return false;
      return true;
   }
};

class AllPairsIndicator
{
public:
   const OpenRAVE::CollisionCheckerBasePtr _cc;
   const OpenRAVE::RobotBasePtr _robot;
   const std::set< std::pair<OpenRAVE::KinBody::LinkConstPtr, OpenRAVE::KinBody::LinkConstPtr> > _pairs;
   
   AllPairsIndicator(const OpenRAVE::RobotBasePtr & robot,
         const std::set< std::pair<OpenRAVE::KinBody::LinkConstPtr, OpenRAVE::KinBody::LinkConstPtr> > & pairs):
      _cc(robot->GetEnv()->GetCollisionChecker()), _robot(robot), _pairs(pairs)
   {
   }
   
   bool operator()(const std::vector<OpenRAVE::dReal> & values) const
   {
      _robot->SetActiveDOFValues(values);
      for (std::set< std::pair<OpenRAVE::KinBody::LinkConstPtr, OpenRAVE::KinBody::LinkConstPtr> >::iterator
         it=_pairs.begin(); it!=_pairs.end(); it++)
      {
         if (_cc->CheckCollision(it->first, it->second))
            return false;
      }
      
      return true;
   }
};


} // namespace or_lemur
