
/*
 * This class is a readable boost property map
 * which maps from reversed edges to original edges
 */

namespace pr_bgl
{
   template <class G>
   class RevEdgeMap
   {
   public:
      typedef typename boost::graph_traits<G>::edge_descriptor GEdge;
      typedef typename boost::reverse_graph<G> R;
      typedef typename boost::graph_traits<R>::edge_descriptor REdge;
      typedef REdge key_type;
      typedef GEdge value_type;
      typedef GEdge reference;
      typedef boost::readable_property_map_tag category;
      const R & rgp;
      RevEdgeMap(const R & in) : rgp(in) {}
   };
} // namespace pr_bgl

namespace boost
{
   template <class G>
   inline typename pr_bgl::RevEdgeMap<G>::GEdge
   get(const pr_bgl::RevEdgeMap<G>& map, const typename pr_bgl::RevEdgeMap<G>::REdge& k)
   {
      return boost::get(boost::edge_underlying_t(), map.rgp, k);
   }
} // namespace boost
