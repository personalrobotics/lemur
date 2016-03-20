/*! \file RoadmapCached.h
 * \author Chris Dellin <cdellin@gmail.com>
 * \copyright 2015 Carnegie Mellon University
 * \copyright License: BSD
 */

namespace or_lemur
{

// cached roadmap wrapper

// the file format is binary
// first size_t: number of batches in file
// for each batch:
//   first size_t: number of vertices in subgraph (cumulative)
//   second size_t: number of edges in subgraph (cumulative)
//   next vertices, then edges
//   for each vertex:
//      size_t is_shadow (0 or 1)
//      dim * sizeof(double): state
//   for each edge:
//      size_t vidx_a
//      size_t vidx_b
//      double distance
// last, size_t: number of bytes in generator state
// followed by those bytes
//   

template <class RoadmapArgs>
class RoadmapCached : public ompl_lemur::Roadmap<RoadmapArgs>
{
   typedef boost::graph_traits<typename RoadmapArgs::Graph> GraphTypes;
   typedef typename GraphTypes::vertex_descriptor Vertex;
   typedef typename GraphTypes::edge_descriptor Edge;
   
   // on construction
   const boost::shared_ptr< ompl_lemur::Roadmap<RoadmapArgs> > _roadmap_wrapped;
   unsigned int _dim;
   
   // determined on initialization
   std::string _cache_filename;
   
   std::ifstream _infile;
   size_t _infile_num_batches;
   
   // these are the numbers that we've added to the actual graph
   std::vector<size_t> vertices_in_subgraph;
   std::vector<size_t> edges_in_subgraph;

public:
   RoadmapCached(RoadmapArgs & args,
         boost::shared_ptr< ompl_lemur::Roadmap<RoadmapArgs> > roadmap_wrapped):
      ompl_lemur::Roadmap<RoadmapArgs>(args,
         "Cached" + roadmap_wrapped->name,
         roadmap_wrapped->max_batches),
      _roadmap_wrapped(roadmap_wrapped),
      _dim(roadmap_wrapped->space->getDimension())
   {
      // check that we're in a real vector state space
      // for now, we only know how to serialize/deserialize these states!
      if (roadmap_wrapped->space->getType() != ompl::base::STATE_SPACE_REAL_VECTOR)
         throw std::runtime_error("RoadmapCached only supports real vector state spaces!");
      
      // we don't have settable parameters;
      // cache_filename will be set on initialization based on wrapped roadmap's params
      this->params.include(_roadmap_wrapped->params);
   }
   ~RoadmapCached() {}
   
   void initialize()
   {
      // compute the cache filename for this wrapped roadmap
      std::string id = "space_id " + ompl_lemur::space_id(this->space)
         + " roadmap_id " + ompl_lemur::roadmap_id(_roadmap_wrapped.get());
      _cache_filename = "e8/roadmap-" + OpenRAVE::utils::GetMD5HashString(id) + ".bin";
            
      _infile_num_batches = 0; // assume read failure
      do
      {
         size_t mysizet;
         // search for file in openrave databases path
         std::string path = OpenRAVE::RaveFindDatabaseFile(_cache_filename, true); // bRead
         if (!path.size())
            break;
         _infile.open(path.c_str(), std::ofstream::binary);
         if (!_infile.is_open())
            break;
         _infile.read((char *)&mysizet, sizeof(mysizet));
         if (!_infile.good())
            break;
         _infile_num_batches = mysizet;
         printf("found a file with %lu batches!\n", _infile_num_batches);
      }
      while (0);
      
      // initialize wrapped roadmap (so root_radius etc work)
      _roadmap_wrapped->initialize();
      
      this->initialized = true;
   }
   
   void deserialize(const std::string & ser_data)
   {
      throw std::runtime_error("RoadmapCached deserialize from ser_data not supported!");
   }
   
   double root_radius(std::size_t i_batch)
   {
      return _roadmap_wrapped->root_radius(i_batch);
   }
   
   void generate()
   {
      // ok, should we generate from the cache file?
      if (this->num_batches_generated < _infile_num_batches)
      {
         // read batch size
         size_t num_vertices_subgraph;
         size_t num_edges_subgraph;
         _infile.read((char *)&num_vertices_subgraph, sizeof(size_t));
         _infile.read((char *)&num_edges_subgraph, sizeof(size_t));
         printf("found %lu vertices and %lu edges in cached subgraph ...\n",
            num_vertices_subgraph, num_edges_subgraph);
         
         // load vertices
         for (size_t v_index=num_vertices(this->g); v_index<num_vertices_subgraph; v_index++)
         {
            size_t is_shadow;
            ompl::base::State * v_state = this->space->allocState();
            double * v_values = v_state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
            
            _infile.read((char *)&is_shadow, sizeof(size_t));
            _infile.read((char *)v_values, _dim*sizeof(double));
            
            if (!_infile.good())
               throw std::runtime_error("error reading vertex from cache file!");
            
            //printf("adding vertex %lu at", v_index);
            //for (unsigned int j=0; j<_dim; j++) printf(" %f", v_values[j]);
            //printf("\n");
            
            Vertex v_new = add_vertex(this->g);
            put(this->vertex_batch_map, v_new, this->num_batches_generated);
            put(this->is_shadow_map, v_new, is_shadow ? true : false);
            put(this->state_map, v_new, v_state);
            this->nn->add(v_new);
         }
         
         // load edges
         for (size_t e_index=num_edges(this->g); e_index<num_edges_subgraph; e_index++)
         {
            size_t vidx_a;
            size_t vidx_b;
            double dist;
            
            _infile.read((char *)&vidx_a, sizeof(size_t));
            _infile.read((char *)&vidx_b, sizeof(size_t));
            _infile.read((char *)&dist, sizeof(double));
            
            if (!_infile.good())
               throw std::runtime_error("error reading edge from cache file!");
            
            //printf("adding edge b/w %lu -- %lu\n", vidx_a, vidx_b);
            
            Vertex v_a = vertex(vidx_a, this->g);
            Vertex v_b = vertex(vidx_b, this->g);
            Edge e = add_edge(v_a, v_b, this->g).first;
            put(this->distance_map, e, dist);
            put(this->edge_batch_map, e, this->num_batches_generated);
         }
         
         if (!_infile.good())
            throw std::runtime_error("error reading from cache file!");
         
         // we've generated a batch
         this->num_batches_generated++;
      }
      else
      {
         // we've generated num_batches_generated so far
         
         // is this the first one, after we generated something?
         if (_infile_num_batches // there were batches in the file
            && this->num_batches_generated // that we've generated
            && !(_roadmap_wrapped->num_batches_generated)) // but wrapped hasn't yet!
         {
            // ok, there were batches we generated from the cache,
            // and now we're using the wrapped roadmap for the rest
            
            // we should deserialize the generator state first
            size_t ser_data_len;
            _infile.read((char *)&ser_data_len, sizeof(size_t));
            std::string ser_data;
            ser_data.resize(ser_data_len);
            _infile.read(&ser_data[0], ser_data_len);
            _roadmap_wrapped->deserialize(ser_data);
            
            _roadmap_wrapped->num_batches_generated = this->num_batches_generated;
            
            _infile.close();
         }
         
         _roadmap_wrapped->generate();
         
         this->num_batches_generated = _roadmap_wrapped->num_batches_generated;
      }
      
      // remember to keep ours and the wrapped
      // num_batches_generated synchronized!
      
      vertices_in_subgraph.push_back(num_vertices(this->g));
      edges_in_subgraph.push_back(num_edges(this->g));
   }
   
   void save_file() const
   {
      size_t mysizet;
      
      // TODO: skip of we've generated into the the graph
      // fewer batches than the file has!
      if (_roadmap_wrapped->num_batches_generated <= _infile_num_batches)
         return;
      
      printf("saving file ...\n");
      
      std::string path = OpenRAVE::RaveFindDatabaseFile(_cache_filename, false); // bRead
      if (!path.size())
         throw OpenRAVE::openrave_exception("couldn't find a place to write rave database entry!");
      boost::filesystem::create_directories(boost::filesystem::path(path).parent_path());
      
      std::ofstream outfile(path.c_str(), std::ofstream::binary);
      
      // write number of batches
      
      mysizet = _roadmap_wrapped->num_batches_generated;
      outfile.write((const char*)&mysizet, sizeof(mysizet));
      
      // for each generated batch, write the vertices and edges!
      size_t v_index = 0;
      size_t e_index = 0;
      for (size_t i_batch=0; i_batch<_roadmap_wrapped->num_batches_generated; i_batch++)
      {
         mysizet = vertices_in_subgraph[i_batch];
         outfile.write((const char*)&mysizet, sizeof(mysizet));
         mysizet = edges_in_subgraph[i_batch];
         outfile.write((const char*)&mysizet, sizeof(mysizet));
         
         // write out the vertices for this batch
         for (; v_index<vertices_in_subgraph[i_batch]; v_index++)
         {
            Vertex v = vertex(v_index,this->g);
            
            mysizet = get(this->is_shadow_map, v) ? 1 : 0;
            outfile.write((const char*)&mysizet, sizeof(mysizet));
            
            ompl::base::State * v_state = get(this->state_map, v);
            double * v_values = v_state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
            outfile.write((const char*)v_values, _dim*sizeof(double));
         }
         
         // write out the edges for this batch
         for (; e_index<edges_in_subgraph[i_batch]; e_index++)
         {
            size_t vidx_a;
            size_t vidx_b;
            double dist;
            
            // get the edge by its index somehow!
            Edge e = get(this->edge_vector_map, e_index);
            Vertex v_a = source(e, this->g);
            Vertex v_b = target(e, this->g);
            
            vidx_a = v_a;
            vidx_b = v_b;
            dist = get(this->distance_map, e);
            
            outfile.write((const char*)&vidx_a, sizeof(size_t));
            outfile.write((const char*)&vidx_b, sizeof(size_t));
            outfile.write((const char*)&dist, sizeof(double));
         }
      }
      
      // next we must save the generator state!!!
      std::string data;
      _roadmap_wrapped->serialize(data);
      mysizet = data.size();
      outfile.write((const char*)&mysizet, sizeof(mysizet));
      outfile.write(data.data(), data.size());
   }
   
   void serialize(std::string & ser_data)
   {
      throw std::runtime_error("RoadmapCached serialize to ser_data not supported!");
   }
};


template <class RoadmapArgs>
class RoadmapCachedFactory
{
public:
   boost::function<ompl_lemur::Roadmap<RoadmapArgs> * (RoadmapArgs args)> _factory;
   
   RoadmapCachedFactory(
      boost::function<ompl_lemur::Roadmap<RoadmapArgs> * (RoadmapArgs args)> factory):
      _factory(factory)
   {
   }
   
   ompl_lemur::Roadmap<RoadmapArgs> * operator()(RoadmapArgs args) const
   {
      boost::shared_ptr< ompl_lemur::Roadmap<RoadmapArgs> > roadmap_wrapped(_factory(args));
      return new RoadmapCached<RoadmapArgs>(args, roadmap_wrapped);
   };
};

} // namespace or_lemur
