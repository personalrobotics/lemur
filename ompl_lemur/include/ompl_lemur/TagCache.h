/*! \file TagCache.h
 * \author Chris Dellin <cdellin@gmail.com>
 * \copyright 2015 Carnegie Mellon University
 * \copyright License: BSD
 */

namespace ompl_lemur
{

template <class VTagMap, class ETagsMap>
class TagCache
{
public:
   virtual void load_begin(void) = 0;
   virtual void load_vertices(VTagMap v_tag_map, size_t v_from, size_t v_to) = 0;
   virtual void load_edges(ETagsMap e_tags_map, size_t e_from, size_t e_to) = 0;
   virtual void load_end(void) = 0;
   
   virtual void save_begin(void) = 0;
   virtual void save_vertices(VTagMap v_tag_map, size_t v_from, size_t v_to) = 0;
   virtual void save_edges(ETagsMap e_tags_map, size_t e_from, size_t e_to) = 0;
   virtual void save_end(void) = 0;
};

template <class VTagMap, class ETagsMap>
class DummyTagCache : public TagCache<VTagMap,ETagsMap>
{
public:
   void load_begin(void) {}
   void load_vertices(VTagMap v_tag_map, size_t v_from, size_t v_to) {}
   void load_edges(ETagsMap e_tags_map, size_t e_from, size_t e_to) {}
   void load_end(void) {}
   
   void save_begin(void) {}
   void save_vertices(VTagMap v_tag_map, size_t v_from, size_t v_to) {}
   void save_edges(ETagsMap e_tags_map, size_t e_from, size_t e_to) {}
   void save_end(void) {}
};

} // namespace ompl_lemur
