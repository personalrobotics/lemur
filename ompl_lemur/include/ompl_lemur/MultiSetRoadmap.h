/* File: Roadmap.h
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

namespace ompl_lemur
{

/* representation of a potentially infinite explicit graph
 * (infinite as in infinitely dense)
 * vertices 0, 1, 2, ...
 * for now, assumed UNDIRECTED! */
class MultiSetRoadmap
{
public:
   struct SubGraph
   {
      unsigned int nv;
      unsigned int ne;
      double root_radius;
      SubGraph(unsigned int nv, unsigned int ne, double root_radius):
         nv(nv), ne(ne), root_radius(root_radius) {}
      SubGraph(): nv(0), ne(0), root_radius(0.) {}
   };

   const ompl::base::StateSpacePtr space;
   // these hold computed information
   std::vector< ompl::base::State * > vertices;
   std::vector< std::pair<unsigned int, unsigned int> > edges;
   std::vector< SubGraph > subgraphs; // nv, ne
   
   MultiSetRoadmap(ompl::base::StateSpacePtr space): space(space) {};
   virtual ~MultiSetRoadmap() {};
   
   virtual std::string get_id() = 0;
   
   // the subgraph number n must be below this number
   // special value 0 means infinite density
   virtual void subgraphs_limit(unsigned int * num_subgraphs) = 0;
   virtual void subgraphs_generate(unsigned int num_subgraphs) = 0;
   
   // gauranteed that all state is in the constructor params
   // plus vertices, edges, and subgraphs vectors above
   // in addition to the generator state retrieved below
   virtual void generator_save(std::string & data) = 0;
   virtual void generator_load(std::string & data) = 0;
};

typedef boost::shared_ptr<MultiSetRoadmap> MultiSetRoadmapPtr;

} // namespace ompl_lemur
