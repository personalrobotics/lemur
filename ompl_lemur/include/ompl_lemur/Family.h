/* File: Family.h
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

namespace ompl_lemur
{

// a family is a collection of subsets
// each backed by a spaceinformationptr
// along with set relations thereof
//
// this is passed as input to the family planner
//
// callers manipulate this directly;
// ideally this is the only thing holding si ptrs between calls
struct Family
{
   struct Subset
   {
      ompl::base::SpaceInformationPtr si;
      double check_cost;
      Subset(ompl::base::SpaceInformationPtr si, double check_cost):
         si(si), check_cost(check_cost)
      {
      }
   };
   std::map<std::string, Subset> subsets;
   
   struct Inclusion
   {
      std::string subset;
      std::string superset;
      Inclusion(std::string subset, std::string superset):
         subset(subset), superset(superset)
      {}
      friend bool operator<(const Inclusion & l, const Inclusion & r)
      {
         if (l.subset < r.subset) return true;
         if (r.subset < l.subset) return false;
         return l.superset < r.superset;
      }
   };
   std::set<Inclusion> inclusions;
   
   struct Intersection
   {
      std::string subset;
      std::set<std::string> supersets;
      Intersection(std::string subset, std::string superset_a, std::string superset_b):
         subset(subset)
      {
         supersets.insert(superset_a);
         supersets.insert(superset_b);
      }
      Intersection(std::string subset, std::set<std::string> & supersets):
         subset(subset), supersets(supersets)
      {
      }
      friend bool operator<(const Intersection & l, const Intersection & r)
      {
         if (l.subset < r.subset) return true;
         if (r.subset < l.subset) return false;
         //if (l.superset_a < r.superset_a) return true;
         //if (r.superset_a < l.superset_a) return false;
         return l.supersets < r.supersets;
      }
   };
   std::set<Intersection> intersections;
};

} // namespace family
