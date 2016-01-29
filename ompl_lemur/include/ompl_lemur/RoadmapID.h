/* File: RoadmapID.h
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

namespace ompl_lemur
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
   
   if (parsed.name == "HaltonOffDens")
   {
      if (parsed.args.size() != 3
         || parsed.args[0].first != "n_perbatch"
         || parsed.args[1].first != "radius_firstbatch"
         || parsed.args[2].first != "seed")
         throw std::runtime_error("roadmap args incorrect!");
      unsigned int n_perbatch = atoi(parsed.args[0].second.c_str());
      double radius_firstbatch = atof(parsed.args[1].second.c_str());
      unsigned int seed = atoi(parsed.args[2].second.c_str());
      return new RoadmapHaltonOffDens<Roadmap>(space, n_perbatch, radius_firstbatch, seed);
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
   
   if (parsed.name == "RGGDens")
   {
      if (parsed.args.size() != 3
         || parsed.args[0].first != "n_perbatch"
         || parsed.args[1].first != "radius_firstbatch"
         || parsed.args[2].first != "seed")
         throw std::runtime_error("roadmap args incorrect!");
      unsigned int n_perbatch = atoi(parsed.args[0].second.c_str());
      double radius_firstbatch = atof(parsed.args[1].second.c_str());
      unsigned int seed = atoi(parsed.args[2].second.c_str());
      return new RoadmapRGGDens<Roadmap>(space, n_perbatch, radius_firstbatch, seed);
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
   
   if (parsed.name == "FromFile")
   {
      if (parsed.args.size() != 2
         || parsed.args[0].first != "filename"
         || parsed.args[1].first != "root_radius")
         throw std::runtime_error("roadmap args incorrect!");
      std::string filename = parsed.args[0].second.c_str();
      double root_radius = atof(parsed.args[1].second.c_str());
      return new RoadmapFromFile<Roadmap>(space, filename, root_radius);
   }
   
   throw std::runtime_error("unknown roadmap type!");
}

template <class Roadmap>
std::string roadmap_header(const Roadmap * roadmap)
{
   if (const RoadmapAAGrid<Roadmap> * x
      = dynamic_cast<const RoadmapAAGrid<Roadmap>*>(roadmap))
   {
      std::string header;
      header = "roadmap_type: AAGrid\n";
      header += ompl_lemur::util::sf("res: %s\n",
         ompl_lemur::util::double_to_text(x->res).c_str());
      return header;
   }
   if (const RoadmapHalton<Roadmap> * x
      = dynamic_cast<const RoadmapHalton<Roadmap>*>(roadmap))
   {
      std::string header;
      header = "roadmap_type: Halton\n";
      header += ompl_lemur::util::sf("n: %u\n", x->n);
      header += ompl_lemur::util::sf("radius: %s\n",
         ompl_lemur::util::double_to_text(x->radius).c_str());
      return header;
   }
   if (const RoadmapHaltonDens<Roadmap> * x
      = dynamic_cast<const RoadmapHaltonDens<Roadmap>*>(roadmap))
   {
      std::string header;
      header = "roadmap_type: HaltonDens\n";
      header += ompl_lemur::util::sf("n_perbatch: %u\n", x->n_perbatch);
      header += ompl_lemur::util::sf("radius_firstbatch: %s\n",
         ompl_lemur::util::double_to_text(x->radius_firstbatch).c_str());
      return header;
   }
   if (const RoadmapHaltonOffDens<Roadmap> * x
      = dynamic_cast<const RoadmapHaltonOffDens<Roadmap>*>(roadmap))
   {
      std::string header;
      header = "roadmap_type: HaltonOffDens\n";
      header += ompl_lemur::util::sf("n_perbatch: %u\n", x->n_perbatch);
      header += ompl_lemur::util::sf("radius_firstbatch: %s\n",
         ompl_lemur::util::double_to_text(x->radius_firstbatch).c_str());
      header += ompl_lemur::util::sf("seed: %u\n", x->seed);
      return header;
   }
   if (const RoadmapRGG<Roadmap> * x
      = dynamic_cast<const RoadmapRGG<Roadmap>*>(roadmap))
   {
      std::string header;
      header = "roadmap_type: RGG\n";
      header += ompl_lemur::util::sf("n: %u\n", x->n);
      header += ompl_lemur::util::sf("radius: %s\n",
         ompl_lemur::util::double_to_text(x->radius).c_str());
      header += ompl_lemur::util::sf("seed: %u\n", x->seed);
      return header;
   }
   if (const RoadmapRGGDensConst<Roadmap> * x
      = dynamic_cast<const RoadmapRGGDensConst<Roadmap>*>(roadmap))
   {
      std::string header;
      header = "roadmap_type: RGGDensConst\n";
      header += ompl_lemur::util::sf("n_perbatch: %u\n", x->n_perbatch);
      header += ompl_lemur::util::sf("radius: %s\n",
         ompl_lemur::util::double_to_text(x->radius).c_str());
      header += ompl_lemur::util::sf("seed: %u\n", x->seed);
      return header;
   }
   throw std::runtime_error("unknown roadmap type!");
}

} // namespace ompl_lemur
