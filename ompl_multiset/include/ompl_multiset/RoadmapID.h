/* File: RoadmapID.h
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

namespace ompl_multiset
{

/* roadmap_id like:
 * - "AAGrid(res=0.16)"
 * - "Halton(n=30 radius=0.3)"
 * - "HaltonDens(n_perbatch=30 radius_firstbatch=0.3)"
 * - "RGG(n=30 radius=0.3 seed=1)"
 * - "RGGDensConst(n_perbatch=30 radius=0.2 seed=1)"
 */

template <class Roadmap>
Roadmap * make_roadmap_gen(const ompl::base::StateSpacePtr & space, std::string roadmap_id)
{
   FnString parsed(roadmap_id);
   
   if (parsed.name == "AAGrid")
   {
      if (parsed.args.size() != 1
         || parsed.args[0].first != "res")
         throw std::runtime_error("roadmap args incorrect!");
      double res = atof(parsed.args[0].second.c_str());
      return new RoadmapAAGrid<Roadmap>(space, res);
   }
   
   if (parsed.name == "Halton")
   {
      if (parsed.args.size() != 2
         || parsed.args[0].first != "n"
         || parsed.args[1].first != "radius")
         throw std::runtime_error("roadmap args incorrect!");
      unsigned int n = atoi(parsed.args[0].second.c_str());
      double radius = atof(parsed.args[1].second.c_str());
      return new RoadmapHalton<Roadmap>(space, n, radius);
   }
   
   if (parsed.name == "HaltonDens")
   {
      if (parsed.args.size() != 2
         || parsed.args[0].first != "n_perbatch"
         || parsed.args[1].first != "radius_firstbatch")
         throw std::runtime_error("roadmap args incorrect!");
      unsigned int n_perbatch = atoi(parsed.args[0].second.c_str());
      double radius_firstbatch = atof(parsed.args[1].second.c_str());
      return new RoadmapHaltonDens<Roadmap>(space, n_perbatch, radius_firstbatch);
   }

   if (parsed.name == "RGG")
   {
      if (parsed.args.size() != 3
         || parsed.args[0].first != "n"
         || parsed.args[1].first != "radius"
         || parsed.args[2].first != "seed")
         throw std::runtime_error("roadmap args incorrect!");
      unsigned int n = atoi(parsed.args[0].second.c_str());
      double radius = atof(parsed.args[1].second.c_str());
      unsigned int seed = atoi(parsed.args[2].second.c_str());
      return new RoadmapRGG<Roadmap>(space, n, radius, seed);
   }
   
   if (parsed.name == "RGGDensConst")
   {
      if (parsed.args.size() != 3
         || parsed.args[0].first != "n_perbatch"
         || parsed.args[1].first != "radius"
         || parsed.args[2].first != "seed")
         throw std::runtime_error("roadmap args incorrect!");
      unsigned int n_perbatch = atoi(parsed.args[0].second.c_str());
      double radius = atof(parsed.args[1].second.c_str());
      unsigned int seed = atoi(parsed.args[2].second.c_str());
      return new RoadmapRGGDensConst<Roadmap>(space, n_perbatch, radius, seed);
   }

   throw std::runtime_error("unknown roadmap type!");
}

} // namespace ompl_multiset
