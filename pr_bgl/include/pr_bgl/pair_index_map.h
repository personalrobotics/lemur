/* File: pair_index_map.h
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

/*
 * This class is a readable boost property map
 * which maps from a pair of indexable entities to a master index
 * like c row major order 2d matrix indices
 */

namespace pr_bgl
{
   template <class Each, class EachIndexMap>
   class PairIndexMap
   {
   public:
      typedef std::pair<Each,Each> key_type;
      typedef size_t value_type;
      typedef size_t reference;
      typedef boost::readable_property_map_tag category;
      const EachIndexMap & eachmap;
      const size_t ncols;
      PairIndexMap(const EachIndexMap & in_eachmap, const size_t in_ncols) :
         eachmap(in_eachmap), ncols(in_ncols) {}
   };
} // namespace pr_bgl

namespace boost
{
   template <class Each, class EachIndexMap>
   inline size_t
   get(const pr_bgl::PairIndexMap<Each,EachIndexMap>& map, const std::pair<Each,Each> & k)
   {
      size_t i = boost::get(map.eachmap, k.first);
      size_t j = boost::get(map.eachmap, k.second);
      return i*map.ncols + j;
   }
} // namespace boost
