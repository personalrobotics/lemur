/*! \file flag_set_map.h
 * \author Chris Dellin <cdellin@gmail.com>
 * \copyright 2015 Carnegie Mellon University
 * \copyright License: BSD
 * 
 * \brief Contains pr_bgl::flag_set_map.
 */

namespace pr_bgl
{

/*! \brief Wrapper for a readable property map which sets a flag when
 *         accessed.
 * 
 * The flag_set_map class wraps an existing map by putting true to an
 * ancillary map (with the same key) whenever the primary wrap is
 * accessed (read or written).
 */
template <class PropMap, class FlagMap>
class flag_set_map
{
public:
   typedef typename boost::property_traits<PropMap>::category category;
   typedef typename boost::property_traits<PropMap>::key_type key_type;
   typedef typename boost::property_traits<PropMap>::value_type value_type;
   typedef typename boost::property_traits<PropMap>::reference reference;
   PropMap prop_map;
   FlagMap flag_map;
   flag_set_map(PropMap prop_map, FlagMap flag_map):
      prop_map(prop_map), flag_map(flag_map)
   {}
};

template <class PropMap, class FlagMap>
flag_set_map<PropMap,FlagMap>
make_flag_set_map(PropMap prop_map, FlagMap flag_map)
{
   return flag_set_map<PropMap,FlagMap>(prop_map, flag_map);
}

template <class PropMap, class FlagMap>
inline const typename flag_set_map<PropMap,FlagMap>::value_type
get(const flag_set_map<PropMap,FlagMap> & fs_map,
   const typename flag_set_map<PropMap,FlagMap>::key_type & key)
{
   put(fs_map.flag_map, key, true);
   return get(fs_map.prop_map, key);
}

template <class PropMap, class FlagMap>
inline void
put(const flag_set_map<PropMap,FlagMap> & fs_map,
   const typename flag_set_map<PropMap,FlagMap>::key_type & key,
   const typename flag_set_map<PropMap,FlagMap>::value_type & value)
{
   put(fs_map.flag_map, key, true);
   put(fs_map.prop_map, key, value);
}

} // namespace pr_bgl
