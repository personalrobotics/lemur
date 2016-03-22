/*! \file params_lemur.h
 * \author Chris Dellin <cdellin@gmail.com>
 * \copyright 2015 Carnegie Mellon University
 * \copyright License: BSD
 */

namespace or_lemur
{

// remember, <startstate> and <goalstate> can be specified multiple times
class LEMURParameters : public OpenRAVE::PlannerBase::PlannerParameters
{
public:

   bool has_roadmap_type;
   std::string roadmap_type;
   
   // key/value strings for roadmap (keys without roadmap. prefix)
   std::vector< std::pair<std::string,std::string> > roadmap_params;
   
   bool has_do_roadmap_save;
   bool do_roadmap_save;
   
   bool has_num_batches_init;
   unsigned int num_batches_init;
   
   bool has_alglog;
   std::string alglog;

   bool has_do_alglog_append;
   bool do_alglog_append;
   
   bool has_graph;
   std::string graph;
   
   bool has_check_cost;
   double check_cost;
   
   bool has_coeff_distance;
   double coeff_distance;
   
   bool has_coeff_checkcost;
   double coeff_checkcost;
   
   bool has_coeff_batch;
   double coeff_batch;
   
   bool has_do_timing;
   bool do_timing;
   
   bool has_persist_roots;
   bool persist_roots;
   
   bool has_max_batches;
   unsigned int max_batches;
   
   bool has_time_limit;
   double time_limit;
   
   bool has_search_type;
   std::string search_type;
   
   bool has_eval_type;
   std::string eval_type;
   
   bool has_do_solve_all;
   bool do_solve_all;
   
   LEMURParameters():
      has_roadmap_type(false),
      has_do_roadmap_save(false),
      has_num_batches_init(false),
      has_alglog(false),
      has_do_alglog_append(false),
      has_graph(false),
      has_check_cost(false),
      has_coeff_distance(false),
      has_coeff_checkcost(false),
      has_coeff_batch(false),
      has_do_timing(false),
      has_persist_roots(false),
      has_max_batches(false),
      has_time_limit(false),
      has_search_type(false),
      has_eval_type(false),
      has_do_solve_all(false)
   {
      // top-level tags we can process
      _vXMLParameters.push_back("roadmap_type");
      _vXMLParameters.push_back("roadmap_param");
      _vXMLParameters.push_back("do_roadmap_save");
      _vXMLParameters.push_back("num_batches_init");
      _vXMLParameters.push_back("alglog");
      _vXMLParameters.push_back("do_alglog_append");
      _vXMLParameters.push_back("graph");
      _vXMLParameters.push_back("check_cost");
      _vXMLParameters.push_back("coeff_distance");
      _vXMLParameters.push_back("coeff_checkcost");
      _vXMLParameters.push_back("coeff_batch");
      _vXMLParameters.push_back("do_timing");
      _vXMLParameters.push_back("persist_roots");
      _vXMLParameters.push_back("max_batches");
      _vXMLParameters.push_back("time_limit");
      _vXMLParameters.push_back("search_type");
      _vXMLParameters.push_back("eval_type");
      _vXMLParameters.push_back("do_solve_all");
   }
   
private:
   std::string lemur_deserializing;
   
protected:
   bool serialize(std::ostream& sout, int options=0) const
   {
      if (!OpenRAVE::PlannerBase::PlannerParameters::serialize(sout,options))
         return false;
      if (has_roadmap_type)
         sout << "<roadmap_type>" << roadmap_type << "</roadmap_type>";
      for (unsigned int ui=0; ui<roadmap_params.size(); ui++)
      {
         sout << "<roadmap_param>"
            << roadmap_params[ui].first << "=" << roadmap_params[ui].second
            << "</roadmap_param>";
      }
      if (has_do_roadmap_save)
         sout << "<do_roadmap_save>" << (do_roadmap_save?"true":"false") << "</do_roadmap_save>";
      if (has_num_batches_init)
         sout << "<num_batches_init>" << num_batches_init << "</num_batches_init>";
      if (has_alglog)
         sout << "<alglog>" << alglog << "</alglog>";
      if (has_do_alglog_append)
         sout << "<do_alglog_append>" << (do_alglog_append?"true":"false") << "</do_alglog_append>";
      if (has_graph)
         sout << "<graph>" << graph << "</graph>";
      if (has_check_cost)
         sout << "<check_cost>" << check_cost << "</check_cost>";
      if (has_coeff_distance)
         sout << "<coeff_distance>" << coeff_distance << "</coeff_distance>";
      if (has_coeff_checkcost)
         sout << "<coeff_checkcost>" << coeff_checkcost << "</coeff_checkcost>";
      if (has_coeff_batch)
         sout << "<coeff_batch>" << coeff_batch << "</coeff_batch>";
      if (has_do_timing)
         sout << "<do_timing>" << (do_timing?"true":"false") << "</do_timing>";
      if (has_persist_roots)
         sout << "<persist_roots>" << (persist_roots?"true":"false") << "</persist_roots>";
      if (has_max_batches)
         sout << "<max_batches>" << max_batches << "</max_batches>";
      if (has_time_limit)
         sout << "<time_limit>" << time_limit << "</time_limit>";
      if (has_search_type)
         sout << "<search_type>" << search_type << "</search_type>";
      if (has_eval_type)
         sout << "<eval_type>" << eval_type << "</eval_type>";
      if (has_do_solve_all)
         sout << "<do_solve_all>" << (do_solve_all?"true":"false") << "</do_solve_all>";
      return !!sout;
   }
   
   OpenRAVE::BaseXMLReader::ProcessElement startElement(
      const std::string & name, const OpenRAVE::AttributesList & atts)
   {
      if (lemur_deserializing.size())
         return PE_Ignore;
      // ask base calss
      enum OpenRAVE::BaseXMLReader::ProcessElement base;
      base = OpenRAVE::PlannerBase::PlannerParameters::startElement(name,atts);
      if (base != PE_Pass) return base;
      // can we handle it?
      if (name == "roadmap_type"
         || name == "roadmap_param"
         || name == "do_roadmap_save"
         || name == "num_batches_init"
         || name == "alglog"
         || name == "do_alglog_append"
         || name == "graph"
         || name == "check_cost"
         || name == "coeff_distance"
         || name == "coeff_checkcost"
         || name == "coeff_batch"
         || name == "do_timing"
         || name == "persist_roots"
         || name == "max_batches"
         || name == "time_limit"
         || name == "search_type"
         || name == "eval_type"
         || name == "do_solve_all")
      {
         lemur_deserializing = name;
         _ss.str("");
         return PE_Support;
      }
      return PE_Pass;
   }

   bool endElement(const std::string & name)
   {
      if (!lemur_deserializing.size())
         return OpenRAVE::PlannerBase::PlannerParameters::endElement(name);
      if (name == lemur_deserializing)
      {
         if (lemur_deserializing == "roadmap_type")
         {
            roadmap_type = _ss.str();
            has_roadmap_type = true;
         }
         if (lemur_deserializing == "roadmap_param")
         {
            std::string roadmap_param = _ss.str();
            size_t eq = roadmap_param.find('=');
            if (eq != roadmap_param.npos)
               roadmap_params.push_back(std::make_pair(
                  roadmap_param.substr(0,eq), roadmap_param.substr(eq+1)));
            else
               RAVELOG_WARN("no = found in roadmap_param!\n");
         }
         if (lemur_deserializing == "do_roadmap_save")
         {
            std::ios state(0);
            state.copyfmt(_ss);
            _ss >> std::boolalpha >> do_roadmap_save;
            _ss.copyfmt(state);
            has_do_roadmap_save = true;
         }
         if (lemur_deserializing == "num_batches_init")
         {
            _ss >> num_batches_init;
            has_num_batches_init = true;
         }
         if (lemur_deserializing == "alglog")
         {
            alglog = _ss.str();
            has_alglog = true;
         }
         if (lemur_deserializing == "do_alglog_append")
         {
            std::ios state(0);
            state.copyfmt(_ss);
            _ss >> std::boolalpha >> do_alglog_append;
            _ss.copyfmt(state);
            has_do_alglog_append = true;
         }
         if (lemur_deserializing == "graph")
         {
            graph = _ss.str();
            has_graph = true;
         }
         if (lemur_deserializing == "check_cost")
         {
            _ss >> check_cost;
            has_check_cost = true;
         }
         if (lemur_deserializing == "coeff_distance")
         {
            _ss >> coeff_distance;
            has_coeff_distance = true;
         }
         if (lemur_deserializing == "coeff_checkcost")
         {
            _ss >> coeff_checkcost;
            has_coeff_checkcost = true;
         }
         if (lemur_deserializing == "coeff_batch")
         {
            _ss >> coeff_batch;
            has_coeff_batch = true;
         }
         if (lemur_deserializing == "do_timing")
         {
            std::ios state(0);
            state.copyfmt(_ss);
            _ss >> std::boolalpha >> do_timing;
            _ss.copyfmt(state);
            has_do_timing = true;
         }
         if (lemur_deserializing == "persist_roots")
         {
            std::ios state(0);
            state.copyfmt(_ss);
            _ss >> std::boolalpha >> persist_roots;
            _ss.copyfmt(state);
            has_persist_roots = true;
         }
         if (lemur_deserializing == "max_batches")
         {
            _ss >> max_batches;
            has_max_batches = true;
         }
         if (lemur_deserializing == "time_limit")
         {
            _ss >> time_limit;
            has_time_limit = true;
         }
         if (lemur_deserializing == "search_type")
         {
            search_type = _ss.str();
            has_search_type = true;
         }
         if (lemur_deserializing == "eval_type")
         {
            eval_type = _ss.str();
            has_eval_type = true;
         }
         if (lemur_deserializing == "do_solve_all")
         {
            std::ios state(0);
            state.copyfmt(_ss);
            _ss >> std::boolalpha >> do_solve_all;
            _ss.copyfmt(state);
            has_do_solve_all = true;
         }
      }
      else
         RAVELOG_WARN("closing tag doesnt match opening tag!\n");
      lemur_deserializing.clear();
      return false;
   }
};

typedef boost::shared_ptr<LEMURParameters> LEMURParametersPtr;
typedef boost::shared_ptr<LEMURParameters const> LEMURParametersConstPtr;

} // namespace or_lemur
