
namespace or_multiset
{

// <startstate> and <goalstate> can be specified multiple times
class PlannerParameters : public OpenRAVE::PlannerBase::PlannerParameters
{
public:
   // extra parameters
   std::vector< std::vector<OpenRAVE::dReal> > startstates;
   std::vector< std::vector<OpenRAVE::dReal> > goalstates;

   PlannerParameters():
      deser_startstate(false),
      deser_goalstate(false)
   {
      _vXMLParameters.push_back("startstate");
      _vXMLParameters.push_back("goalstate");
   }
   
protected:
   bool deser_startstate;
   bool deser_goalstate;
   
   bool serialize(std::ostream& sout, int options) const
   {
      if (!OpenRAVE::PlannerBase::PlannerParameters::serialize(sout))
         return false;
      printf("serializing ...\n");
      for (unsigned int si=0; si<startstates.size(); si++)
      {
         sout << "<startstate>";
         for (unsigned int j=0; j<startstates[si].size(); j++)
         {
            if (j) sout << " ";
            sout << startstates[si][j];
         }
         sout << "</startstate>" << std::endl;
      }
      for (unsigned int si=0; si<goalstates.size(); si++)
      {
         sout << "<goalstate>";
         for (unsigned int j=0; j<goalstates[si].size(); j++)
         {
            if (j) sout << " ";
            sout << goalstates[si][j];
         }
         sout << "</goalstate>" << std::endl;
      }
      printf(" ... serializing done!\n");
      return !!sout;
   }
   
   OpenRAVE::BaseXMLReader::ProcessElement startElement(
      const std::string & name, const OpenRAVE::AttributesList & atts)
   {
      if (deser_startstate || deser_goalstate)
         return PE_Ignore;
      // ask base calss
      enum OpenRAVE::BaseXMLReader::ProcessElement base;
      base = OpenRAVE::PlannerBase::PlannerParameters::startElement(name,atts);
      if (base != PE_Pass) return base;
      // can we handle it?
      if (name == "startstate")
      {
         deser_startstate = true;
         return PE_Support;
      }
      if (name == "goalstate")
      {
         deser_goalstate = true;
         return PE_Support;
      }
      return PE_Pass;
   }
   
   bool endElement(const std::string & name)
   {
      if (deser_startstate)
      {
         if (name == "startstate")
         {
            std::vector<OpenRAVE::dReal> state;
            while (_ss.good())
            {
               OpenRAVE::dReal val;
               _ss >> val;
               state.push_back(val);
            }
            startstates.push_back(state);
         }
         else
            RAVELOG_WARN(str(boost::format("unknown tag %s\n")%name));
         deser_startstate = false;
         return false;
      }
      if (deser_goalstate)
      {
         if (name == "goalstate")
         {
            std::vector<OpenRAVE::dReal> state;
            while (_ss.good())
            {
               OpenRAVE::dReal val;
               _ss >> val;
               state.push_back(val);
            }
            goalstates.push_back(state);
         }
         else
            RAVELOG_WARN(str(boost::format("unknown tag %s\n")%name));
         deser_goalstate = false;
         return false;
      }
      // give a chance for the default parameters to get processed
      return OpenRAVE::PlannerBase::PlannerParameters::endElement(name);
   }
};

typedef boost::shared_ptr<PlannerParameters> PlannerParametersPtr;
typedef boost::shared_ptr<PlannerParameters const> PlannerParametersConstPtr;

} // namespace or_multiset
