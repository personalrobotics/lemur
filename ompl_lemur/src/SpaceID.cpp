/*! \file SpaceID.cpp
 * \author Chris Dellin <cdellin@gmail.com>
 * \copyright 2015 Carnegie Mellon University
 * \copyright License: BSD
 */

#include <cstdarg>
#include <stdexcept>
#include <string>
#include <boost/shared_ptr.hpp>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl_lemur/SpaceID.h>
#include <ompl_lemur/util.h>

std::string ompl_lemur::space_id(const ompl::base::StateSpacePtr space)
{
   boost::shared_ptr<ompl::base::RealVectorStateSpace> real
      = boost::dynamic_pointer_cast<ompl::base::RealVectorStateSpace>(space);
   if (real)
   {
      std::string id = ompl_lemur::util::sf("class=RealVectorStateSpace dim=%u",
         real->getDimension());
      ompl::base::RealVectorBounds bounds = real->getBounds();
      for (unsigned int di=0; di<real->getDimension(); di++)
      {
         id += ompl_lemur::util::sf(" bounds%u=(%s,%s)", di,
            ompl_lemur::util::double_to_text(bounds.low[di]).c_str(),
            ompl_lemur::util::double_to_text(bounds.high[di]).c_str());
      }
      id += ompl_lemur::util::sf(" longest_valid_segment_length=%s",
         ompl_lemur::util::double_to_text(real->getLongestValidSegmentLength()).c_str());
      return id;
   }
   throw std::runtime_error("space_id: unknown space type!");
}
