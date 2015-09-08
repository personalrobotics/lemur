/* File: flag_set_map.h
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

namespace pr_bgl
{

// wrapper for a readable property map which sets a flag when accessed
template <class PropMap, class FlagMap>
class FlagSetMap
{
public:
   typedef typename boost::property_traits<PropMap>::category category;
   typedef typename boost::property_traits<PropMap>::key_type key_type;
   typedef typename boost::property_traits<PropMap>::value_type value_type;
   typedef typename boost::property_traits<PropMap>::reference reference;
   PropMap prop_map;
   FlagMap flag_map;
   FlagSetMap(PropMap prop_map, FlagMap flag_map):
      prop_map(prop_map), flag_map(flag_map)
   {}
};

template <class PropMap, class FlagMap>
FlagSetMap<PropMap,FlagMap>
make_flag_set_map(PropMap prop_map, FlagMap flag_map)
{
   return FlagSetMap<PropMap,FlagMap>(prop_map, flag_map);
}

template <class PropMap, class FlagMap>
inline const typename FlagSetMap<PropMap,FlagMap>::value_type
get(const FlagSetMap<PropMap,FlagMap> & fs_map,
   const typename FlagSetMap<PropMap,FlagMap>::key_type & key)
{
   put(fs_map.flag_map, key, true);
   return get(fs_map.prop_map, key);
}

template <class PropMap, class FlagMap>
inline void
put(const FlagSetMap<PropMap,FlagMap> & fs_map,
   const typename FlagSetMap<PropMap,FlagMap>::key_type & key,
   const typename FlagSetMap<PropMap,FlagMap>::value_type & value)
{
   put(fs_map.flag_map, key, true);
   put(fs_map.prop_map, key, value);
}

} // namespace pr_bgl
