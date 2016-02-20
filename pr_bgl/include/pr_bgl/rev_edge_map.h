/*! \file rev_edge_map.h
 * \author Chris Dellin <cdellin@gmail.com>
 * \copyright 2015 Carnegie Mellon University
 * \copyright License: BSD
 * 
 * \brief Property map which maps from reversed edges to original edges
 *        (pr_bgl::rev_edge_map).
 */

namespace pr_bgl
{

/*! \brief Readable boost property map which maps from reversed edges
 *         to original edges.
 * 
 * The rev_edge_map class is a readable boost property map which maps
 * from reversed edges to original edges in a reversed graph.
 */
template <class G>
class rev_edge_map
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
   rev_edge_map(const R & in) : rgp(in) {}
};

template <class G>
inline typename pr_bgl::rev_edge_map<G>::GEdge
get(const rev_edge_map<G>& map, const typename pr_bgl::rev_edge_map<G>::REdge& k)
{
   return boost::get(boost::edge_underlying_t(), map.rgp, k);
}

} // namespace pr_bgl
