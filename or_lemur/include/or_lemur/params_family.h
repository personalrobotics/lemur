/*! \file params_family.h
 * \author Chris Dellin <cdellin@gmail.com>
 * \copyright 2015 Carnegie Mellon University
 * \copyright License: BSD
 */

namespace or_lemur
{

// <startstate> and <goalstate> can be specified multiple times
class FamilyParameters : public or_lemur::LEMURParameters
{
public:

   bool has_family_module;
   std::string family_module;
   
   FamilyParameters():
      has_family_module(false)
   {
      _vXMLParameters.push_back("family_module");
   }
   
private:
   std::string family_deserializing;
   
protected:
   bool serialize(std::ostream& sout, int options=0) const
   {
      if (!or_lemur::LEMURParameters::serialize(sout,options))
         return false;
      if (has_family_module)
         sout << "<family_module>" << family_module << "</family_module>";
      return !!sout;
   }
   
   OpenRAVE::BaseXMLReader::ProcessElement startElement(
      const std::string & name, const OpenRAVE::AttributesList & atts)
   {
      if (family_deserializing.size())
         return PE_Ignore;
      // ask base calss
      enum OpenRAVE::BaseXMLReader::ProcessElement base;
      base = or_lemur::LEMURParameters::startElement(name,atts);
      if (base != PE_Pass) return base;
      // can we handle it?
      if (name == "family_module")
      {
         family_deserializing = name;
         return PE_Support;
      }
      return PE_Pass;
   }

   bool endElement(const std::string & name)
   {
      if (!family_deserializing.size())
         return or_lemur::LEMURParameters::endElement(name);
      if (name == family_deserializing)
      {
         if (family_deserializing == "family_module")
         {
            family_module = _ss.str();
            has_family_module = true;
         }
      }
      else
         RAVELOG_WARN("closing tag doesnt match opening tag!\n");
      family_deserializing.clear();
      return false;
   }
};

typedef boost::shared_ptr<FamilyParameters> FamilyParametersPtr;
typedef boost::shared_ptr<FamilyParameters const> FamilyParametersConstPtr;

} // namespace or_lemur
