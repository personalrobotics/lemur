#include <cstdarg>
#include <stdexcept>
#include <string>
#include <vector>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/functional/hash.hpp>
#include <ompl/base/StateSpace.h>
#include <ompl_multiset/Roadmap.h>
#include <ompl_multiset/SpaceID.h>
#include <ompl_multiset/Cache.h>
#include <ompl_multiset/util.h>

using namespace ompl_multiset::util; // for sf, startswith, sha1, double_to_text

namespace {

class C : public ompl_multiset::Cache
{
public:
   C(std::string cache_dir);
   ~C();
   
   void roadmap_load(ompl_multiset::Roadmap * roadmap);
   void roadmap_save(ompl_multiset::Roadmap * roadmap);
   
   // these load/save results for different edges
   // maybe i should do deltas or something at some point?
   
   void si_load(
      ompl_multiset::Roadmap * roadmap, std::string set_id,
      std::vector< std::pair<unsigned int, bool> > & vertex_results,
      std::vector< std::pair<unsigned int, bool> > & edge_results);
   
   void si_save(
      ompl_multiset::Roadmap * roadmap, std::string set_id,
      std::vector< std::pair<unsigned int, bool> > & vertex_results,
      std::vector< std::pair<unsigned int, bool> > & edge_results);
   
   // custom methods
   std::string get_space_id(const ompl::base::StateSpacePtr space);
   
   boost::filesystem::path cache_dir;
};

} // anonymous namespace

ompl_multiset::Cache * ompl_multiset::cache_create(std::string cache_dir)
{
   return new C(cache_dir);
}

C::C(std::string cache_dir):
   cache_dir(cache_dir)
{
   boost::filesystem::create_directory(this->cache_dir);
}

C::~C()
{
}

void C::roadmap_load(ompl_multiset::Roadmap * roadmap)
{
   std::string line;
   
   // get the space id
   std::string space_id = ompl_multiset::space_id(roadmap->space);
   std::string roadmap_id = roadmap->get_id();
   std::string hash = sha1("space " + space_id + " roadmap " + roadmap_id);
   
   // open file
   boost::filesystem::path newpath = this->cache_dir / ("roadmap-" + hash + ".txt");
   std::ifstream sin(newpath.string().c_str());
   if (!sin.good())
   {
      printf("loading roadmap from cache failed, file not found?\n");
      return;
   }
   
   // space id line first
   getline(sin, line);
   if (!startswith(line,"space "))
      throw std::runtime_error("bad cache file, no space id!");
   if (line.substr(6) != space_id)
      throw std::runtime_error("bad cache file, space id mismatch!");
   
   // roadmap id line first
   getline(sin, line);
   if (!startswith(line,"roadmap "))
      throw std::runtime_error("bad cache file, no roadmap id!");
   if (line.substr(8) != roadmap_id)
      throw std::runtime_error("bad cache file, roadmap id mismatch!");
   
   unsigned int ng = 0;
   unsigned int nv = 0;
   unsigned int ne = 0;
   std::string generator_data;
   bool extended = false;
   
   // for parsing vertices
   unsigned int serlen = roadmap->space->getSerializationLength();
   std::vector<unsigned char> ser(serlen);
   
   while (getline(sin, line))
   {
      if (startswith(line,"subgraph "))
      {
         unsigned int line_gi;
         unsigned int line_nv;
         unsigned int line_ne;
         double line_root_radius;
         int n_chars;
         sscanf(line.c_str(), "subgraph %u nv %u ne %u root_radius %lf%n",
            &line_gi, &line_nv, &line_ne, &line_root_radius, &n_chars);
         if (n_chars != line.size())
            throw std::runtime_error("bad cache file, error parsing subgraph!");
         if (line_gi != ng)
            throw std::runtime_error("bad cache file, subgraphs out of order!");
         if (roadmap->subgraphs.size() < (line_gi+1))
         {
            roadmap->subgraphs.resize(line_gi+1);
            roadmap->subgraphs[line_gi] = ompl_multiset::Roadmap::SubGraph(
               line_nv, line_ne, line_root_radius);
            extended = true;
         }
         ng++;
      }
      else if (startswith(line,"vertex "))
      {
         unsigned int line_vi;
         int n_chars;
         sscanf(line.c_str(), "vertex %u %n", &line_vi, &n_chars);
         if (n_chars + 2*serlen != line.size())
            throw std::runtime_error("bad cache file, error parsing vertex!");
         if (line_vi != nv)
            throw std::runtime_error("bad cache file, subgraphs out of order!");
         if (roadmap->vertices.size() < (line_vi+1))
         {
            roadmap->vertices.resize(line_vi+1);
            roadmap->vertices[line_vi] = roadmap->space->allocState();
            for (unsigned int i=0; i<serlen; i++)
               sscanf(line.c_str()+n_chars+2*i, "%02x", &ser[i]);
            roadmap->space->deserialize(roadmap->vertices[line_vi], &ser[0]);
            extended = true;
         }
         nv++;
      }
      else if (startswith(line,"edge "))
      {
         unsigned int line_ei;
         unsigned int line_va;
         unsigned int line_vb;
         int n_chars;
         sscanf(line.c_str(), "edge %u va %u vb %u%n",
            &line_ei, &line_va, &line_vb, &n_chars);
         if (n_chars != line.size())
            throw std::runtime_error("bad cache file, error parsing edge!");
         if (line_ei != ne)
            throw std::runtime_error("bad cache file, edges out of order!");
         if (roadmap->edges.size() < (line_ei+1))
         {
            roadmap->edges.resize(line_ei+1);
            roadmap->edges[line_ei] = std::make_pair(line_va, line_vb);
            extended = true;
         }
         ne++;
      }
      else if (startswith(line,"generator_data "))
      {
         if (generator_data.size())
            throw std::runtime_error("bad cache file, duplicate generator_data!");
         generator_data = line.substr(15);
      }
      else
         throw std::runtime_error(sf("bad cache file, unknown line: %s", line.c_str()));
   }
   
   // load generator data if necessary
   if (extended)
   {
      if (!generator_data.size())
         throw std::runtime_error("bad cache file, no generator_data!");
      roadmap->generator_load(generator_data);
   }
}

void C::roadmap_save(ompl_multiset::Roadmap * roadmap)
{
   // get the space id
   std::string space_id = ompl_multiset::space_id(roadmap->space);
   std::string roadmap_id = roadmap->get_id();
   std::string hash = sha1("space " + space_id + " roadmap " + roadmap_id);
   
   // open file
   boost::filesystem::path newpath = this->cache_dir / ("roadmap-" + hash + ".txt");
   FILE * fp = fopen(newpath.string().c_str(), "w");
   
   fprintf(fp, "space %s\n", space_id.c_str());
   fprintf(fp, "roadmap %s\n", roadmap_id.c_str());
   
   // write subgraphs
   for (unsigned int gi=0; gi<roadmap->subgraphs.size(); gi++)
   {
      fprintf(fp, "subgraph %u nv %u ne %u root_radius %s\n", gi,
         roadmap->subgraphs[gi].nv,
         roadmap->subgraphs[gi].ne,
         double_to_text(roadmap->subgraphs[gi].root_radius).c_str());
   }
   
   // write vertices
   unsigned int serlen = roadmap->space->getSerializationLength();
   std::vector<unsigned char> ser(serlen);
   for (unsigned int vi=0; vi<roadmap->vertices.size(); vi++)
   {
      roadmap->space->serialize(&ser[0], roadmap->vertices[vi]);
      fprintf(fp, "vertex %u ", vi);
      for (unsigned int i=0; i<serlen; i++)
         fprintf(fp, "%02x", ser[i]);
      fprintf(fp, "\n");
   }
   
   // write edges
   for (unsigned int ei=0; ei<roadmap->edges.size(); ei++)
   {
      fprintf(fp, "edge %u va %u vb %u\n", ei,
         roadmap->edges[ei].first,
         roadmap->edges[ei].second);
   }
   
   // write saved generator state
   std::string generator_data;
   roadmap->generator_save(generator_data);
   fprintf(fp, "generator_data %s\n", generator_data.c_str());
   
   fclose(fp);
}

// these load/save results for different edges
// maybe i should do deltas or something at some point?

void C::si_load(
   ompl_multiset::Roadmap * roadmap, std::string set_id,
   std::vector< std::pair<unsigned int, bool> > & vertex_results,
   std::vector< std::pair<unsigned int, bool> > & edge_results)
{
}

void C::si_save(
   ompl_multiset::Roadmap * roadmap, std::string set_id,
   std::vector< std::pair<unsigned int, bool> > & vertex_results,
   std::vector< std::pair<unsigned int, bool> > & edge_results)
{
}

