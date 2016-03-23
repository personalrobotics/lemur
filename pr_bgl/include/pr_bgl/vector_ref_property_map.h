/*! \file vector_ref_property_map.h
 * \author Chris Dellin <cdellin@gmail.com>
 * \copyright 2015 Carnegie Mellon University
 * \copyright License: BSD
 * 
 * \brief Contains pr_bgl::edge_indexed_graph.
 */

namespace pr_bgl
{
   
/*! \brief Readable property map adaptor for a vector. 
 * 
 * This is similar to boost::iterator_property_map, except that it holds
 * a reference to the vector, instead of the begin iterator. This allows
 * the map to continue to work even if the referenced vector is
 * modified.
 */
template<typename T>
class vector_ref_property_map
{
public:
   typedef boost::readable_property_map_tag category;
   typedef size_t key_type;
   typedef T value_type;
   typedef T reference;
   std::vector<T> & vec;
   vector_ref_property_map(std::vector<T> & vec): vec(vec) {}
};

template<typename T>
inline const T get(const vector_ref_property_map<T> & map, const size_t & idx)
{
   return map.vec[idx];
}

} // namespace pr_bgl
