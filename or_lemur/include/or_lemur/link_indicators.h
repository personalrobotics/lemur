/*! \file link_indicators.h
 * \author Chris Dellin <cdellin@gmail.com>
 * \copyright 2015 Carnegie Mellon University
 * \copyright License: BSD
 */

namespace or_lemur
{

class AllPairsIndicator
{
public:
   const OpenRAVE::CollisionCheckerBasePtr _cc;
   const OpenRAVE::RobotBasePtr _robot;
   const std::set< std::pair<OpenRAVE::KinBody::LinkPtr, OpenRAVE::KinBody::LinkPtr> > _pairs;
   
   AllPairsIndicator(const OpenRAVE::RobotBasePtr & robot,
         const std::set< std::pair<OpenRAVE::KinBody::LinkPtr, OpenRAVE::KinBody::LinkPtr> > & pairs):
      _cc(robot->GetEnv()->GetCollisionChecker()), _robot(robot), _pairs(pairs)
   {
   }
   
   bool operator()(const std::vector<OpenRAVE::dReal> & values) const
   {
      _robot->SetActiveDOFValues(values);
      for (std::set< std::pair<OpenRAVE::KinBody::LinkPtr, OpenRAVE::KinBody::LinkPtr> >::iterator
         it=_pairs.begin(); it!=_pairs.end(); it++)
      {
         if (_cc->CheckCollision(it->first, it->second))
            return false;
      }
      
      return true;
   }
};



/*
struct int_div { 
  float operator()(int x, int y) const { return ((float)x)/y; }; 
};
 */




typedef boost::function<bool (const std::vector<OpenRAVE::dReal> & values)> OrIndicator;

OrIndicator make_all_pairs_indicator(
   OpenRAVE::RobotBasePtr robot,
   std::pair<OpenRAVE::KinBody::LinkPtr, OpenRAVE::KinBody::LinkPtr> pairs);

} // namespace or_lemur
