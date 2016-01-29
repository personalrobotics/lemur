/* File: Cache.h
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */
 
namespace ompl_lemur
{

// cache object which handles roadmaps and spaceinfo results on them
// stateless object, except cache location etc on construction
class Cache
{
public:
   virtual ~Cache() {}
   
   // these load/save a roadmap w.r.t. a cache
   // using the roadmap's unique id AND the underlying space's id
   virtual void roadmap_load(ompl_lemur::MultiSetRoadmap * roadmap) = 0;
   virtual void roadmap_save(ompl_lemur::MultiSetRoadmap * roadmap) = 0;
   
   // these load/save results for different edges
   // maybe i should do deltas or something at some point?
   
   // does not check for consistency
   virtual void si_load(
      ompl_lemur::MultiSetRoadmap * roadmap, std::string set_id,
      std::vector< std::pair<unsigned int, bool> > & vertex_results,
      std::vector< std::pair<unsigned int, bool> > & edge_results) = 0;
   
   virtual void si_save(
      ompl_lemur::MultiSetRoadmap * roadmap, std::string set_id,
      std::vector< std::pair<unsigned int, bool> > & vertex_results,
      std::vector< std::pair<unsigned int, bool> > & edge_results) = 0;
};

typedef boost::shared_ptr<Cache> CachePtr;

Cache * cache_create(std::string cache_dir);
   
} // namespace ompl_lemur
