/* File: RoadmapGenID.h
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

namespace ompl_multiset
{

template <class RoadmapGen>
RoadmapGen * make_roadmap_gen(const ompl::base::StateSpacePtr & space, std::string roadmap_id)
{
   FnString roadmap_parsed(roadmap_id);
   printf("creating roadmap of type %s ...\n", roadmap_parsed.name.c_str());
   if (roadmap_parsed.name == "aagrid")
      return new RoadmapGenAAGrid<RoadmapGen>(space, roadmap_parsed.argstring);
   else if (roadmap_parsed.name == "rgg")
      return new RoadmapGenRGG<RoadmapGen>(space, roadmap_parsed.argstring);
   else if (roadmap_parsed.name == "halton")
      return new RoadmapGenHalton<RoadmapGen>(space, roadmap_parsed.argstring);
   else if (roadmap_parsed.name == "haltondens")
      return new RoadmapGenHaltonDens<RoadmapGen>(space, roadmap_parsed.argstring);
   else
      throw std::runtime_error("unknown roadmap type!");
}

} // namespace ompl_multiset
