/* File: SpaceID.h
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

namespace ompl_lemur
{
   
std::string space_id(const ompl::base::StateSpacePtr space);

std::string space_header(const ompl::base::StateSpacePtr space);
   
} // namespace ompl_lemur
