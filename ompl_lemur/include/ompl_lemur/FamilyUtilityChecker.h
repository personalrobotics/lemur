/*! \file FamilyUtilityChecker.h
 * \author Chris Dellin <cdellin@gmail.com>
 * \copyright 2015 Carnegie Mellon University
 * \copyright License: BSD
 */

namespace ompl_lemur
{

/*! \brief Use a Family as a UtilityChecker.
 * 
 * This class implements an optimistic search over a family of sets.
 * Internally, this class computes a graph of known states over all
 * N sets (naively 3^N states, modulo any relations).
 * For a given target set and costed checkers, it computes the
 * optimal set of checks (and their results) that would imply
 * membership in the target set.
 * 
 * This used to be called `FamilyEffortModel`.
 */
class FamilyUtilityChecker : public UtilityChecker
{
public:
   
   FamilyUtilityChecker(ompl::base::SpaceInformation * si, const Family & family);
   FamilyUtilityChecker(const ompl::base::SpaceInformationPtr & si, const Family & family);
   ~FamilyUtilityChecker();
   
   // called by constructor
   void initialize();

public: // set by problem definition

   typedef std::pair<double, ompl::base::StateValidityCheckerPtr> SetChecker;
   
   void start_checking(std::string set_target, const std::map<std::string, SetChecker> & set_checkers);
   void stop_checking();

public: // used by planner

   bool hasChanged();
   
   bool isKnown(size_t tag) const;
   
   bool isKnownInvalid(size_t tag) const;
   
   double getPartialEvalCost(size_t tag, const ompl::base::State * state) const;
   
   bool isValidPartialEval(size_t & tag, const ompl::base::State * state) const;
   
//private:
   typedef boost::adjacency_list<boost::vecS,boost::vecS,boost::bidirectionalS>::edge_descriptor PreEdge;
   struct VProps
   {
      std::vector<bool> knowns;
      std::vector<bool> values;
      // below is recalculated every time target si changes!
      double cost_to_go;
      size_t checks_to_go; // if 0, edge_next is garbage!
      PreEdge edge_next;
   };
   struct EProps
   {
      std::size_t var;
      bool value;
      double check_cost;
      PreEdge edge_other;
   };
   typedef boost::adjacency_list<
      boost::vecS, // Edgelist ds, for per-vertex out-edges
      boost::vecS, // VertexList ds, for vertex set
      boost::bidirectionalS, // type of graph
      VProps, // internal (bundled) vertex properties
      EProps // internal (bundled) edge properties
      > Graph;
   typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
   typedef boost::graph_traits<Graph>::vertex_iterator VertexIter;
   typedef boost::graph_traits<Graph>::edge_descriptor Edge;
   typedef boost::graph_traits<Graph>::edge_iterator EdgeIter;
   typedef boost::graph_traits<Graph>::in_edge_iterator InEdgeIter;
   typedef boost::graph_traits<Graph>::out_edge_iterator OutEdgeIter;
   
   const Family _family;
   bool _has_changed;
   
   // view into the family
   std::vector<std::string> _sets;
   //std::vector<double> var_costs;
   
   // compound sentences (permanent)
   // A1 n A2 n A3 => C
   struct ConjunctionImplication
   {
      std::vector<std::size_t> antecedents;
      std::size_t consequent;
   };
   std::vector<ConjunctionImplication> _conjunction_implications;
   
   std::vector< std::vector<bool> > _truth_table;
   
   // graph
   Graph _g;
   
   //ompl::base::SpaceInformationPtr si_target;
   size_t _var_target;
   
   // subsumes var_costs and subsets[g[e].var].second.si->isValid()
   std::vector<SetChecker> _checkers;
};

typedef boost::shared_ptr<FamilyUtilityChecker> FamilyUtilityCheckerPtr;

} // namespace ompl_lemur
