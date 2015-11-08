/* File: params_e8roadmap.h
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

namespace or_multiset
{

// <startstate> and <goalstate> can be specified multiple times
class E8RoadmapParameters : public OpenRAVE::PlannerBase::PlannerParameters
{
public:
   std::string roadmap_id;
   unsigned int num_batches_init;
   double coeff_distance;
   double coeff_checkcost;
   double coeff_batch;
   bool do_timing;
   std::string search_type;
   std::string eval_type;
   std::string alglog;
   std::string graph;
   
   E8RoadmapParameters():
      roadmap_id(""), num_batches_init(1),
      coeff_distance(1.), coeff_checkcost(0.), coeff_batch(0.),
      do_timing(false), search_type("astar"), eval_type("alt")
   {
      _vXMLParameters.push_back("roadmap_id");
      _vXMLParameters.push_back("num_batches_init");
      _vXMLParameters.push_back("coeff_distance");
      _vXMLParameters.push_back("coeff_checkcost");
      _vXMLParameters.push_back("coeff_batch");
      _vXMLParameters.push_back("do_timing");
      _vXMLParameters.push_back("search_type");
      _vXMLParameters.push_back("eval_type");
      _vXMLParameters.push_back("alglog");
      _vXMLParameters.push_back("graph");
   }
   
private:
   std::string el_deserializing;
   
   bool serialize(std::ostream& sout, int options) const
   {
      if (!OpenRAVE::PlannerBase::PlannerParameters::serialize(sout))
         return false;
      sout << "<roadmap_id>" << roadmap_id << "</roadmap_id>";
      sout << "<num_batches_init>" << num_batches_init << "</num_batches_init>";
      sout << "<coeff_distance>" << coeff_distance << "</coeff_distance>";
      sout << "<coeff_checkcost>" << coeff_checkcost << "</coeff_checkcost>";
      sout << "<coeff_batch>" << coeff_batch << "</coeff_batch>";
      sout << "<do_timing>" << do_timing << "</do_timing>";
      sout << "<search_type>" << search_type << "</search_type>";
      sout << "<search_type>" << eval_type << "</search_type>";
      sout << "<alglog>" << alglog << "</alglog>";
      sout << "<graph>" << graph << "</graph>";
      return !!sout;
   }
   
   OpenRAVE::BaseXMLReader::ProcessElement startElement(
      const std::string & name, const OpenRAVE::AttributesList & atts)
   {
      if (el_deserializing.size())
         return PE_Ignore;
      // ask base calss
      enum OpenRAVE::BaseXMLReader::ProcessElement base;
      base = OpenRAVE::PlannerBase::PlannerParameters::startElement(name,atts);
      if (base != PE_Pass) return base;
      // can we handle it?
      if (name == "roadmap_id"
         || name == "num_batches_init"
         || name == "coeff_distance"
         || name == "coeff_checkcost"
         || name == "coeff_batch"
         || name == "do_timing"
         || name == "search_type"
         || name == "eval_type"
         || name == "alglog"
         || name == "graph")
      {
         el_deserializing = name;
         return PE_Support;
      }
      return PE_Pass;
   }

   bool endElement(const std::string & name)
   {
      if (!el_deserializing.size())
         return OpenRAVE::PlannerBase::PlannerParameters::endElement(name);
      if (name == el_deserializing)
      {
         if (el_deserializing == "roadmap_id")
            roadmap_id = _ss.str();
         if (el_deserializing == "num_batches_init")
            _ss >> num_batches_init;
         if (el_deserializing == "coeff_distance")
            _ss >> coeff_distance;
         if (el_deserializing == "coeff_checkcost")
            _ss >> coeff_checkcost;
         if (el_deserializing == "coeff_batch")
            _ss >> coeff_batch;
         if (el_deserializing == "do_timing")
         {
            std::ios state(0);
            state.copyfmt(_ss);
            _ss >> std::boolalpha >> do_timing;
            _ss.copyfmt(state);
         }
         if (el_deserializing == "search_type")
            search_type = _ss.str();
         if (el_deserializing == "eval_type")
            eval_type = _ss.str();
         if (el_deserializing == "alglog")
            alglog = _ss.str();
         if (el_deserializing == "graph")
            graph = _ss.str();
      }
      else
         RAVELOG_WARN("closing tag doesnt match opening tag!\n");
      el_deserializing.clear();
      return false;
   }
};

typedef boost::shared_ptr<E8RoadmapParameters> E8RoadmapParametersPtr;
typedef boost::shared_ptr<E8RoadmapParameters const> E8RoadmapParametersConstPtr;

} // namespace or_multiset
