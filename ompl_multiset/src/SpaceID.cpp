/* File: SpaceID.cpp
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: None
 */

#include <cstdarg>
#include <stdexcept>
#include <string>
#include <boost/shared_ptr.hpp>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl_multiset/SpaceID.h>

namespace
{
std::string sf(const char * fmt, ...)
{
  va_list ap;
  va_start(ap, fmt);
  int size = vsnprintf(0, 0, fmt, ap);
  va_end(ap);
  char * buf = new char[size+1];
  va_start(ap, fmt);
  vsnprintf(buf, size+1, fmt, ap);
  va_end(ap);
  std::string ret = std::string(buf);
  delete[] buf;
  return ret;
}
std::string double_to_text(double in)
{
   char buf[2048];
   int min = 0;
   int max = 2047;
   // invariant: min DOESNT WORK, max DOES WORK
   // validate invariants
   sprintf(buf, "%.*f", min, in);
   if (in == strtod(buf,0))
      return std::string(buf);
   sprintf(buf, "%.*f", max, in);
   if (in != strtod(buf,0))
      return std::string(buf);
   // binary search
   for (;;)
   {
      int diff = max - min;
      if (diff == 1)
         break;
      int test = min + diff/2;
      sprintf(buf, "%.*f", test, in);
      if (in == strtod(buf,0))
         max = test;
      else
         min = test;
   }
   sprintf(buf, "%.*f", max, in);
   return std::string(buf);
}
} // anonymous namespace

std::string ompl_multiset::space_id(const ompl::base::StateSpacePtr space)
{
   boost::shared_ptr<ompl::base::RealVectorStateSpace> real
      = boost::dynamic_pointer_cast<ompl::base::RealVectorStateSpace>(space);
   if (real)
   {
      std::string id = sf("class=RealVectorStateSpace dim=%u",
         real->getDimension());
      ompl::base::RealVectorBounds bounds = real->getBounds();
      for (unsigned int di=0; di<real->getDimension(); di++)
      {
         id += sf(" bounds%u=(%s,%s)", di,
            double_to_text(bounds.low[di]).c_str(),
            double_to_text(bounds.high[di]).c_str());
      }
      return id;
   }
   throw std::runtime_error("space_id: unknown space type!");
}
