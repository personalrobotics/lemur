/*! \file TagCache.h
 * \author Chris Dellin <cdellin@gmail.com>
 * \copyright 2015 Carnegie Mellon University
 * \copyright License: BSD
 */

namespace ompl_lemur
{

// this may throw on load or save error!
template <class VTagMap, class ETagsMap>
class TagCache
{
public:
   // this is checked by setProblemDefinition() (or solve()?)
   // if this is set, then the planner assumes that the entire graph
   // should be re-loaded
   virtual bool hasChanged() = 0;
   
   virtual void loadBegin(void) = 0;
   virtual void loadVertices(VTagMap v_tag_map, size_t v_from, size_t v_to) = 0;
   virtual void loadEdges(ETagsMap e_tags_map, size_t e_from, size_t e_to) = 0;
   virtual void loadEnd(void) = 0;
   
   virtual void saveBegin(void) = 0;
   virtual void saveVertices(VTagMap v_tag_map, size_t v_from, size_t v_to) = 0;
   virtual void saveEdges(ETagsMap e_tags_map, size_t e_from, size_t e_to) = 0;
   virtual void saveEnd(void) = 0;
};

} // namespace ompl_lemur
