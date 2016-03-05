/*! \file Family.h
 * \author Chris Dellin <cdellin@gmail.com>
 * \copyright 2015 Carnegie Mellon University
 * \copyright License: BSD
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
   std::set<std::string> sets;
   
   // andecedents, consequent
   typedef std::pair< std::set<std::string>, std::string > Relation;
   std::set<Relation> relations;
   
   void add_inclusion(std::string subset, std::string superset)
   {
      std::set<std::string> subsets;
      subsets.insert(subset);
      relations.insert(std::make_pair(subsets, superset));
   }
   
   void add_intersection(std::string subset, std::set<std::string> & supersets)
   {
      relations.insert(std::make_pair(supersets, subset));
      std::set<std::string> subsets;
      subsets.insert(subset);
      for (std::set<std::string>::iterator sit=supersets.begin(); sit!=supersets.end(); sit++)
         relations.insert(std::make_pair(subsets, *sit));
   }
};

} // namespace ompl_lemur
