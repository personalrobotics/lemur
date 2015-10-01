/* File: SpaceID.cpp
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

#include <cstdarg>
#include <stdexcept>
#include <string>
#include <boost/shared_ptr.hpp>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl_multiset/SpaceID.h>
#include <ompl_multiset/util.h>

std::string ompl_multiset::space_id(const ompl::base::StateSpacePtr space)
{
   boost::shared_ptr<ompl::base::RealVectorStateSpace> real
      = boost::dynamic_pointer_cast<ompl::base::RealVectorStateSpace>(space);
   if (real)
   {
      std::string id = ompl_multiset::util::sf("class=RealVectorStateSpace dim=%u",
         real->getDimension());
      ompl::base::RealVectorBounds bounds = real->getBounds();
      for (unsigned int di=0; di<real->getDimension(); di++)
      {
         id += ompl_multiset::util::sf(" bounds%u=(%s,%s)", di,
            ompl_multiset::util::double_to_text(bounds.low[di]).c_str(),
            ompl_multiset::util::double_to_text(bounds.high[di]).c_str());
      }
      return id;
   }
   throw std::runtime_error("space_id: unknown space type!");
}
