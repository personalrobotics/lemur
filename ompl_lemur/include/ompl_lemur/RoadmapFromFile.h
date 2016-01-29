/* File: RoadmapFromFile.h
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

namespace ompl_lemur
{

template <class RoadmapArgs>
class RoadmapFromFile : public Roadmap<RoadmapArgs>
{
   typedef boost::graph_traits<typename RoadmapArgs::Graph> GraphTypes;
   typedef typename GraphTypes::vertex_descriptor Vertex;
   typedef typename GraphTypes::vertex_iterator VertexIter;
   typedef typename GraphTypes::edge_descriptor Edge;
   typedef typename GraphTypes::edge_iterator EdgeIter;

   // params
   std::string _filename;
   double _root_radius;
   
public:
   RoadmapFromFile(RoadmapArgs & args):
      Roadmap<RoadmapArgs>(args, "FromFile", 1),
      _filename(""),
      _root_radius(0.0)
   {
      // check that we're in a real vector state space
      if (this->space->getType() != ompl::base::STATE_SPACE_REAL_VECTOR)
         throw std::runtime_error("RoadmapFromFile only supports rel vector state spaces!");
      
      this->template declareParam<std::string>("filename", this,
         &RoadmapFromFile::setFilename,
         &RoadmapFromFile::getFilename);
      this->template declareParam<double>("root_radius", this,
         &RoadmapFromFile::setRootRadius,
         &RoadmapFromFile::getRootRadius);
   }

   void setFilename(std::string filename)
   {
      if (filename == _filename)
         return;
      if (this->initialized)
         throw std::runtime_error("cannot set filename, already initialized!");
      _filename = filename;
   }
   
   std::string getFilename() const
   {
      return _filename;
   }

   void setRootRadius(double root_radius)
   {
      if (root_radius == _root_radius)
         return;
      if (this->initialized)
         throw std::runtime_error("cannot set root_radius, already initialized!");
      _root_radius = root_radius;
   }
   
   double getRootRadius() const
   {
      return _root_radius;
   }

   void initialize()
   {
      if (_filename == "")
         throw std::runtime_error("cannot initialize, filename not set!");
      if (_root_radius == 0.0)
         throw std::runtime_error("cannot initialize, root_radius not set!");
      this->initialized = true;
   }
   
   void deserialize(const std::string & ser_data)
   {
      throw std::runtime_error("RoadmapFromFile deserialize from ser_data not supported!");
   }
   
   // should be stateless
   double root_radius(std::size_t i_batch)
   {
      return _root_radius;
   }
   
   // sets all of these maps
   // generates one additional batch
   void generate()
   {
      if (this->max_batches < this->num_batches_generated + 1)
         throw std::runtime_error("this roadmap gen doesnt support that many batches!");
      
      std::ifstream fp;
      fp.open(_filename.c_str());
      
      boost::dynamic_properties props;
      const ompl::base::StateSpacePtr & myspace = this->space;
      props.property("state",
         ompl_lemur::make_rvstate_map_string_adaptor(
            this->state_map,
            myspace->as<ompl::base::RealVectorStateSpace>()));
      boost::read_graphml(fp, this->g, props);
      
      VertexIter vi, vi_end;
      for (boost::tie(vi,vi_end)=vertices(this->g); vi!=vi_end; ++vi)
      {
         put(this->vertex_batch_map, *vi, 0);
         put(this->is_shadow_map, *vi, false);
         this->nn->add(*vi);
      }
      
      EdgeIter ei, ei_end;
      for (boost::tie(ei,ei_end)=edges(this->g); ei!=ei_end; ++ei)
      {
         put(this->edge_batch_map, *ei, 0);
         ompl::base::State * state1 = get(this->state_map, source(*ei,this->g));
         ompl::base::State * state2 = get(this->state_map, target(*ei,this->g));
         put(this->distance_map, *ei, this->space->distance(state1, state2));
      }
      
      this->num_batches_generated++;
   }
   
   void serialize(std::string & ser_data)
   {
      throw std::runtime_error("RoadmapFromFile serialize to ser_data not supported!");
   }
};

} // namespace ompl_lemur
