/*! \file params_family.h
 * \author Chris Dellin <cdellin@gmail.com>
 * \copyright 2015 Carnegie Mellon University
 * \copyright License: BSD
 */

namespace or_lemur
{

// 
class FamilyParameters : public or_lemur::LEMURParameters
{
public:

   bool has_family_module;
   std::string family_module;
   
   // list of family cache filenames
   // note there's a difference between no <family_caches> and an empty <family_caches>
   struct SetCache
   {
      std::string name;
      std::string filename;
   };
   bool has_family_setcaches;
   std::vector< SetCache > family_setcaches;
   
   FamilyParameters():
      has_family_module(false)
   {
      _vXMLParameters.push_back("family_module");
      _vXMLParameters.push_back("family_setcaches");
   }
   
private:
   // path we're currently deserializing
   std::string family_path;
   SetCache family_setcache_cur;
   
protected:
   bool serialize(std::ostream& sout, int options=0) const
   {
      if (!or_lemur::LEMURParameters::serialize(sout,options))
         return false;
      if (has_family_module)
         sout << "<family_module>" << family_module << "</family_module>";
      if (has_family_setcaches)
      {
         sout << "<family_setcaches>";
         for (unsigned int ui=0; ui<family_setcaches.size(); ui++)
         {
            sout << "<setcache>";
            sout << "<name>" << family_setcaches[ui].name << "</name>";
            sout << "<filename>" << family_setcaches[ui].filename << "</filename>";
            sout << "</setcache>";
         }
         sout << "</family_setcaches>";
      }
      return !!sout;
   }
   
   OpenRAVE::BaseXMLReader::ProcessElement startElement(
      const std::string & name, const OpenRAVE::AttributesList & atts)
   {
      OpenRAVE::BaseXMLReader::ProcessElement ret = PE_Ignore;
      if (family_path == "")
      {
         if (name == "family_module"
            || name == "family_setcaches")
         {
            family_path = name;
            ret = PE_Support;
         }
         else
         {
            // ask base class
            return or_lemur::LEMURParameters::startElement(name,atts);
         }
      }
      else if (family_path == "family_setcaches")
      {
         if (name == "setcache")
         {
            family_path = "family_setcaches/" + name;
            family_setcache_cur.name = "";
            family_setcache_cur.filename = "";
            ret = PE_Support;
         }
      }
      else if (family_path == "family_setcaches/setcache")
      {
         if (name == "name"
            || name == "filename")
         {
            family_path = "family_setcaches/setcache/" + name;
            ret = PE_Support;
         }
      }
      if (ret == PE_Support)
         _ss.str("");
      if (ret == PE_Ignore)
         RAVELOG_WARN("Ignoring unknown tag <%s>!\n", name.c_str());
      return ret;
   }

   bool endElement(const std::string & name)
   {
      if (family_path == "")
         return or_lemur::LEMURParameters::endElement(name);
      if (family_path == "family_module")
      {
         printf("found family module!\n");
         family_module = _ss.str();
         has_family_module = true;
         family_path = "";
         return false;
      }
      if (family_path == "family_setcaches/setcache/name")
      {
         if (name != "name")
            RAVELOG_WARN("Closing tag <%s> doesn't match opening tag <name>!\n", name.c_str());
         family_setcache_cur.name = _ss.str();
         family_path = "family_setcaches/setcache";
         return false;
      }
      if (family_path == "family_setcaches/setcache/filename")
      {
         if (name != "filename")
            RAVELOG_WARN("Closing tag <%s> doesn't match opening tag <filename>!\n", name.c_str());
         family_setcache_cur.filename = _ss.str();
         family_path = "family_setcaches/setcache";
         return false;
      }
      if (family_path == "family_setcaches/setcache")
      {
         family_setcaches.push_back(family_setcache_cur);
         family_path = "family_setcaches";
         return false;
      }
      if (family_path == "family_setcaches")
      {
         has_family_setcaches = true;
         family_path = "";
         return false;
      }
      RAVELOG_ERROR("Parse error!");
      return false;
   }
};

typedef boost::shared_ptr<FamilyParameters> FamilyParametersPtr;
typedef boost::shared_ptr<FamilyParameters const> FamilyParametersConstPtr;

} // namespace or_lemur
