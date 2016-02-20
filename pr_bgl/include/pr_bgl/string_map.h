/*! \file string_map.h
 * \author Chris Dellin <cdellin@gmail.com>
 * \copyright 2015 Carnegie Mellon University
 * \copyright License: BSD
 * 
 * \brief Wraps a property as a string map (pr_bgl::string_map).
 */

namespace pr_bgl
{

inline void stringify_from_x(std::string & repr, const double & val)
{
   char buf[2048];
   // invariant: min DOESNT WORK, max DOES WORK
   // validate invariants
   int min = 0;
   sprintf(buf, "%.*f", min, val);
   if (val == strtod(buf,0))
   {
      repr = std::string(buf);
      return;
   }
   // what is it at 1?
   sprintf(buf, "%.*f", 1, val);
   int max = sizeof(buf)-strlen(buf);
   sprintf(buf, "%.*f", max, val);
   if (val != strtod(buf,0))
   {
      printf("stringify_from_x invariant failed!\n");
      abort();
   }
   // binary search
   for (;;)
   {
      int diff = max - min;
      if (diff == 1)
         break;
      int test = min + diff/2;
      sprintf(buf, "%.*f", test, val);
      if (val == strtod(buf,0))
         max = test;
      else
         min = test;
   }
   sprintf(buf, "%.*f", max, val);
   repr = std::string(buf);
   return;
}
inline void stringify_to_x(const std::string & repr, double & val)
{
   val = atof(repr.c_str());
}


inline void stringify_from_x(std::string & repr, const int & val)
{
   char buf[2048];
   sprintf(buf, "%d", val);
   repr = std::string(buf);
}
inline void stringify_to_x(const std::string & repr, int & val)
{
   val = atoi(repr.c_str());
}


inline void stringify_from_x(std::string & repr, const bool & val)
{
   if (val)
      repr = "true";
   else
      repr = "false";
}
inline void stringify_to_x(const std::string & repr, bool & val)
{
   if (repr == "true")
      val = true;
   else if (repr == "true")
      val = false;
   else
      throw std::runtime_error("parse error!");
}


/*! \brief Wraps a property as a string map.
 *
 * The string_map class implements a read-write map which wraps an
 * existing property by allowing converting its values to and from
 * the std::string instances. It requires that the free functions
 * stringify_from_x() and stringify_to_x() for any custom values.
 *
 * \todo this may duplicate functionality from boost::lexical_cast.
 */
template <class PropMap>
class string_map
{
public:
   typedef typename boost::property_traits<PropMap>::category category;
   typedef typename boost::property_traits<PropMap>::key_type key_type;
   typedef std::string value_type;
   typedef std::string reference;
   PropMap prop_map;
   string_map(PropMap prop_map) : prop_map(prop_map) {}
};

template <class PropMap>
string_map<PropMap> make_string_map(PropMap prop_map)
{
   return string_map<PropMap>(prop_map);
}

template <class PropMap>
inline std::string
get(const string_map<PropMap> & map, const typename string_map<PropMap>::key_type & k)
{
   std::string repr;
   stringify_from_x(repr, get(map.prop_map,k));
   return repr;
}

template <class PropMap>
inline void
put(const string_map<PropMap> & map, const typename string_map<PropMap>::key_type & k, const std::string repr)
{
   typename boost::property_traits<PropMap>::value_type val;
   stringify_to_x(repr, val);
   put(map.prop_map, k, val);
}

} // namespace pr_bgl
