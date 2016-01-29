/* File: rvstate_map_string_adaptor.h
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

namespace ompl_lemur
{

// treat ompl state values as space-separated strings
// when putting, allocs a new state (does not free existing!)
template <class StateMap>
class rvstate_map_string_adaptor
{
public:
   typedef boost::read_write_property_map_tag category;
   typedef typename boost::property_traits<StateMap>::key_type key_type;
   typedef std::string value_type;
   typedef std::string reference;
   const StateMap state_map;
   ompl::base::RealVectorStateSpace * rvspace;
   const unsigned int dim;
   rvstate_map_string_adaptor(StateMap state_map, ompl::base::RealVectorStateSpace * rvspace):
      state_map(state_map), rvspace(rvspace), dim(rvspace->getDimension())
   {
   }
};

template <class StateMap>
inline std::string
get(const rvstate_map_string_adaptor<StateMap> & adaptor,
   const typename rvstate_map_string_adaptor<StateMap>::key_type & k)
{
   ompl::base::RealVectorStateSpace::StateType * rvstate
      = (ompl::base::RealVectorStateSpace::StateType *)get(adaptor.state_map, k);
   if (!rvstate)
      return std::string();
   std::string s;
   for (unsigned int ui=0; ui<adaptor.dim; ui++)
   {
      if (ui) s += " ";
      std::string component_repr;
      pr_bgl::stringify_from_x(component_repr, rvstate->values[ui]);
      s += component_repr;
   }
   return s;
}

template <class StateMap>
inline void
put(const rvstate_map_string_adaptor<StateMap> & adaptor,
   const typename rvstate_map_string_adaptor<StateMap>::key_type & k,
   const std::string s)
{
   ompl::base::RealVectorStateSpace::StateType * rvstate;
   if (s.length() == 0)
   {
      rvstate = 0;
   }
   else
   {
      rvstate = (ompl::base::RealVectorStateSpace::StateType *)adaptor.rvspace->allocState();
      std::stringstream ss(s);
      for (unsigned int ui=0; ui<adaptor.dim; ui++)
         ss >> rvstate->values[ui];
   }
   put(adaptor.state_map, k, rvstate);
}

template <class StateMap>
rvstate_map_string_adaptor<StateMap>
make_rvstate_map_string_adaptor(StateMap state_map, ompl::base::RealVectorStateSpace * rvspace)
{
   return rvstate_map_string_adaptor<StateMap>(state_map, rvspace);
}

} // namespace ompl_lemur
