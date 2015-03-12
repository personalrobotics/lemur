
namespace ompl_multiset
{

/* representation of a potentially infinite explicit graph
 * (infinite as in infinitely dense)
 * vertices 0, 1, 2, ...
 * for now, assumed UNDIRECTED! */
class Roadmap
{
public:
   const ompl::base::StateSpacePtr space;
   // these hold computed information
   std::vector< ompl::base::State * > vertices;
   std::vector< std::pair<unsigned int, unsigned int> > edges;
   std::vector< std::pair<unsigned int, unsigned int> > subgraphs; // nv, ne
   
   Roadmap(ompl::base::StateSpacePtr space): space(space) {};
   virtual ~Roadmap() {};
   
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

} // namespace ompl_multiset
