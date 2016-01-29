/* File: Roadmap.h
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

namespace ompl_lemur
{


template <class Graph_, class VState_, class EDistance_, class VBatch_, class EBatch_, class VShadow_, class EVector_, class NN_>
struct RoadmapArgs
{
   typedef Graph_ Graph;
   typedef VState_ VState;
   typedef EDistance_ EDistance;
   typedef VBatch_ VBatch;
   typedef EBatch_ EBatch;
   typedef VShadow_ VShadow;
   typedef EVector_ EVector;
   typedef NN_ NN;
   ompl::base::StateSpacePtr space;
   Graph & g;
   VState state_map;
   EDistance distance_map;
   VBatch vertex_batch_map;
   EBatch edge_batch_map;
   VShadow is_shadow_map;
   EVector edge_vector_map;
   NN * nn;
   RoadmapArgs(
      ompl::base::StateSpacePtr space,
      Graph & g,
      VState state_map,
      EDistance distance_map,
      VBatch vertex_batch_map,
      EBatch edge_batch_map,
      VShadow is_shadow_map,
      EVector edge_vector_map,
      NN * nn):
      space(space), g(g),
      state_map(state_map), distance_map(distance_map),
      vertex_batch_map(vertex_batch_map), edge_batch_map(edge_batch_map),
      is_shadow_map(is_shadow_map),
      edge_vector_map(edge_vector_map), nn(nn)
   {
   }
};

// continuous space
// generates a possibly infinite roadmap given an ompl space
// note, generate() should tolerate the graph having unused but
// added vertices (e.g. from old/unapplied roots)
//
// API: after construction,
// the caller may set parameters,
// and then must call initialize() at most once at the beginning,
// followed by at most one call to deserialize(),
// followed by successive calls to generate()
// serialize() will save the roadmap generator's state so it can be re-constructed in the future!
//
// new-stype roadmap interface
template <class RoadmapArgs>
class Roadmap
{
public:
   const std::string name;
   const ompl::base::StateSpacePtr space;
   const size_t max_batches; // 0 means inf
   bool initialized;
   size_t num_batches_generated; // should be incremented by implementation's generate()
   
   typename RoadmapArgs::Graph & g;
   typename RoadmapArgs::VState state_map;
   typename RoadmapArgs::EDistance distance_map;
   typename RoadmapArgs::VBatch vertex_batch_map;
   typename RoadmapArgs::EBatch edge_batch_map;
   typename RoadmapArgs::VShadow is_shadow_map;
   typename RoadmapArgs::EVector edge_vector_map;
   typename RoadmapArgs::NN * nn; // ompl nn-like object, will call add() and nearestR()
   
   ompl::base::ParamSet params;
   
   Roadmap(RoadmapArgs & args, std::string name, size_t max_batches):
      name(name),
      space(args.space),
      max_batches(max_batches),
      initialized(false),
      num_batches_generated(0),
      g(args.g),
      state_map(args.state_map),
      distance_map(args.distance_map),
      vertex_batch_map(args.vertex_batch_map),
      edge_batch_map(args.edge_batch_map),
      is_shadow_map(args.is_shadow_map),
      edge_vector_map(args.edge_vector_map),
      nn(args.nn)
   {
   }
   virtual ~Roadmap() {}
   
   template<class T>
   T* as()
   {
      BOOST_CONCEPT_ASSERT((boost::Convertible<T*, Roadmap*>));
      return static_cast<T*>(this);
   }

   template<class T>
   const T* as() const
   {
      BOOST_CONCEPT_ASSERT((boost::Convertible<T*, Roadmap*>));
      return static_cast<const T*>(this);
   }
   
   template<typename T, typename RoadmapType, typename SetterType, typename GetterType>
   void declareParam(const std::string &name, const RoadmapType &roadmap, const SetterType& setter, const GetterType& getter, const std::string &rangeSuggestion = "")
   {
      params.declareParam<T>(name, boost::bind(setter, roadmap, _1), boost::bind(getter, roadmap));
      if (!rangeSuggestion.empty())
         params[name].setRangeSuggestion(rangeSuggestion);
   }

   template<typename T, typename RoadmapType, typename SetterType>
   void declareParam(const std::string &name, const RoadmapType &roadmap, const SetterType& setter, const std::string &rangeSuggestion = "")
   {
      params.declareParam<T>(name, boost::bind(setter, roadmap, _1));
      if (!rangeSuggestion.empty())
         params[name].setRangeSuggestion(rangeSuggestion);
   }
   
   virtual void initialize() = 0;
   
   // should be stateless (but after initialize)
   virtual double root_radius(std::size_t i_batch) = 0;
   
   virtual void deserialize(const std::string & ser_data) = 0;
   
   // sets all of these maps
   // generates one additional batch
   virtual void generate() = 0;
   
   virtual void serialize(std::string & ser_data) = 0;
};

template <class RoadmapArgs, template<class> class RoadmapTemplate>
struct RoadmapFactory
{ 
   ompl_lemur::Roadmap<RoadmapArgs> * operator()(RoadmapArgs args) const
   {
      return new RoadmapTemplate<RoadmapArgs>(args);
   };
};

template <class RoadmapArgs>
std::string roadmap_id(ompl_lemur::Roadmap<RoadmapArgs> * roadmap)
{
   std::string roadmap_id;
   
   roadmap_id += "type=" + roadmap->name;
      
   std::map<std::string, std::string> roadmap_params;
   roadmap->params.getParams(roadmap_params);
   
   for (std::map<std::string, std::string>::iterator
      it=roadmap_params.begin(); it!=roadmap_params.end(); it++)
   {
      roadmap_id += " " + it->first + "=" + it->second;
   }
   
   return roadmap_id;
}


#if 0
template <class Graph, class VState>
class NNOmplBatched
{
public:
   typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
   Graph & g;
   VState state_map;
   const ompl::base::StateSpacePtr space;
   ompl::NearestNeighbors<Vertex> * ompl_nn;
   NNOmplBatched(Graph & g, VState state_map,
      const ompl::base::StateSpacePtr space,
      ompl::NearestNeighbors<Vertex> * ompl_nn):
      g(g), state_map(state_map), space(space), ompl_nn(ompl_nn)
   {
   }
   inline void nearestR(Vertex v_new, double radius, std::vector< std::pair<Vertex,double> > & vrads_near)
   {
      //printf("nearestR NNOmplBatched ...\n");
      vrads_near.clear();
      // add in from ompl_nn object
      std::vector<Vertex> vs_near;
      ompl_nn->nearestR(v_new, radius, vs_near);
      for (unsigned int ui=0; ui<vs_near.size(); ui++)
      {
         if (vs_near[ui] == v_new)
            continue;
         double dist = space->distance(
            get(state_map, v_new)->state,
            get(state_map, vs_near[ui])->state);
         vrads_near.push_back(std::make_pair(vs_near[ui],dist));
      }
      //printf("  found %lu from ompl_nn.\n", vrads_near.size());
      // add in additionals since last sync
      for (unsigned int ui=ompl_nn->size(); ui<num_vertices(g); ui++)
      {
         Vertex v_other = vertex(ui, g);
         if (v_other == v_new)
            continue;
         double dist = this->space->distance(
            get(state_map, v_new)->state,
            get(state_map, v_other)->state);
         if (radius < dist)
            continue;
         vrads_near.push_back(std::make_pair(v_other,dist));
      }
      //printf("  found %lu total.\n", vrads_near.size());
   }
   void sync()
   {
      printf("syncing NNOmplBatched ...\n");
      for (unsigned int ui=ompl_nn->size(); ui<num_vertices(g); ui++)
         ompl_nn->add(vertex(ui,g));
   }
};
#endif

} // namespace ompl_lemur
