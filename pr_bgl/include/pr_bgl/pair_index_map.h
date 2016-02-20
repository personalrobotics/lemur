/*! \file pair_index_map.h
 * \author Chris Dellin <cdellin@gmail.com>
 * \copyright 2015 Carnegie Mellon University
 * \copyright License: BSD
 * 
 * \brief Property map for converting 2d matrix indices to a single
 *        int index (pr_bgl::pair_index_map).
 */

namespace pr_bgl
{

/*! \brief Convert a master index to a row-major-order matrix index pair.
 * 
 * This class is a readable boost property map
 * which maps from a pair of indexable entities to a master index
 * like c row major order 2d matrix indices
 */
template <class Each, class EachIndexMap>
class pair_index_map
{
public:
   typedef std::pair<Each,Each> key_type;
   typedef size_t value_type;
   typedef size_t reference;
   typedef boost::readable_property_map_tag category;
   const EachIndexMap & eachmap;
   const size_t ncols;
   pair_index_map(const EachIndexMap & in_eachmap, const size_t in_ncols) :
      eachmap(in_eachmap), ncols(in_ncols) {}
};

template <class Each, class EachIndexMap>
inline size_t
get(const pair_index_map<Each,EachIndexMap>& map, const std::pair<Each,Each> & k)
{
   size_t i = get(map.eachmap, k.first);
   size_t j = get(map.eachmap, k.second);
   return i*map.ncols + j;
}

} // namespace pr_bgl
