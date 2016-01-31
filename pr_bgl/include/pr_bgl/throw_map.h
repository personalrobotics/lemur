/* File: throw_map.h
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

namespace pr_bgl
{

template <typename key_type_, typename value_type_>
class throw_map
{
public:
   typedef boost::read_write_property_map_tag category;
   typedef key_type_ key_type;
   typedef value_type_ value_type;
   typedef value_type_ reference;
};

template <typename key_type_, typename value_type_>
inline const value_type_ get(const throw_map<key_type_,value_type_> & map, const key_type_ key)
{
   throw std::runtime_error("get(throw_map) called");
}

template <typename key_type_, typename value_type_>
inline void put(const throw_map<key_type_,value_type_> & map, const key_type_ key, const value_type_ val)
{
   throw std::runtime_error("put(throw_map) called");
}

} // namespace pr_bgl
