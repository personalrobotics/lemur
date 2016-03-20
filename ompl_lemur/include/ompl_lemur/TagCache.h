/*! \file TagCache.h
 * \author Chris Dellin <cdellin@gmail.com>
 * \copyright 2015 Carnegie Mellon University
 * \copyright License: BSD
 */

namespace ompl_lemur
{

// this may throw on load or save error!
template <class VTagMap, class ETagMap>
class TagCache
{
public:
   // this is checked by setProblemDefinition() (or solve()?)
   // if this is set, then the planner assumes that the entire graph
   // should be re-loaded
   virtual bool hasChanged() = 0;
   
   virtual void loadBegin() = 0;
   virtual void loadBatch(size_t batch,
      VTagMap v_tag_map, size_t v_from, size_t v_to,
      ETagMap e_tags_map, size_t e_from, size_t e_to) = 0;
   virtual void loadEnd() = 0;
   
   virtual void saveBegin() = 0;
   virtual void saveBatch(size_t batch,
      VTagMap v_tag_map, size_t v_from, size_t v_to,
      ETagMap e_tags_map, size_t e_from, size_t e_to) = 0;
   virtual void saveEnd() = 0;
};

} // namespace ompl_lemur
