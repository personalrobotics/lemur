/*! \file lazysp_wmap_identity_map.h
 * \author Chris Dellin <cdellin@gmail.com>
 * \copyright 2015 Carnegie Mellon University
 * \copyright License: BSD
 * 
 * \brief Contains pr_bgl::lazysp_wmap_identity_map.
 */

namespace pr_bgl
{

/*! \brief lazysp_wmap_identity_map
 */
template <class PropMap>
class lazysp_wmap_identity_map
{
public:
   typedef typename boost::property_traits<PropMap>::category category;
   typedef typename boost::property_traits<PropMap>::key_type key_type;
   typedef std::pair<
      typename boost::property_traits<PropMap>::value_type,
      std::vector< key_type >
      > value_type;
   typedef value_type reference;
   PropMap prop_map;
   lazysp_wmap_identity_map(PropMap prop_map):
      prop_map(prop_map)
   {}
};

template <class PropMap>
lazysp_wmap_identity_map<PropMap>
make_lazysp_wmap_identity_map(PropMap prop_map)
{
   return lazysp_wmap_identity_map<PropMap>(prop_map);
}

template <class PropMap>
inline const typename lazysp_wmap_identity_map<PropMap>::value_type
get(const lazysp_wmap_identity_map<PropMap> & map,
   const typename lazysp_wmap_identity_map<PropMap>::key_type & key)
{
   std::vector< typename lazysp_wmap_identity_map<PropMap>::key_type > vec;
   vec.push_back(key);
   return std::make_pair(get(map.prop_map,key), vec);
}

} // namespace pr_bgl
