/* File: RoadmapID.h
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

namespace ompl_multiset
{

template <class Roadmap>
Roadmap * make_roadmap_gen(const ompl::base::StateSpacePtr & space, std::string roadmap_id)
{
   FnString roadmap_parsed(roadmap_id);
   printf("creating roadmap of type %s ...\n", roadmap_parsed.name.c_str());
   if (roadmap_parsed.name == "aagrid")
      return new RoadmapAAGrid<Roadmap>(space, roadmap_parsed.argstring);
   else if (roadmap_parsed.name == "rgg")
      return new RoadmapRGG<Roadmap>(space, roadmap_parsed.argstring);
   else if (roadmap_parsed.name == "rggdensconst")
      return new RoadmapRGGDensConst<Roadmap>(space, roadmap_parsed.argstring);
   else if (roadmap_parsed.name == "halton")
      return new RoadmapHalton<Roadmap>(space, roadmap_parsed.argstring);
   else if (roadmap_parsed.name == "haltondens")
      return new RoadmapHaltonDens<Roadmap>(space, roadmap_parsed.argstring);
   else
      throw std::runtime_error("unknown roadmap type!");
}

} // namespace ompl_multiset
