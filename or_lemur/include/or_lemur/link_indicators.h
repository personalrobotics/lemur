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
   const boost::function< bool (boost::shared_ptr<void>, OpenRAVE::CollisionReportPtr)> _baked_checker;
   const boost::shared_ptr<void> _baked_check;
   BakedCheckIndicator(const OpenRAVE::RobotBasePtr & robot,
      const boost::function< bool (boost::shared_ptr<void>, OpenRAVE::CollisionReportPtr)> baked_checker,
      const boost::shared_ptr<void> baked_check):
      _robot(robot), _baked_checker(baked_checker), _baked_check(baked_check)
   {
   }
   bool operator()(const std::vector<OpenRAVE::dReal> & values) const
   {
      _robot->SetActiveDOFValues(values);
      bool collides = _baked_checker(_baked_check, OpenRAVE::CollisionReportPtr());
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
