/* File: planner_e8roadmapselfcc.h
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

namespace or_multiset
{

class E8RoadmapSelfCC : public OpenRAVE::PlannerBase
{
public:
   
   // <startstate> and <goalstate> can be specified multiple times
   class PlannerParameters : public OpenRAVE::PlannerBase::PlannerParameters
   {
   public:
      std::string roadmap_id;
      double coeff_distance;
      double coeff_checkcost;
      double coeff_batch;
      std::string alglog;
      std::string graph;
      PlannerParameters();
   private:
      std::string el_deserializing;
      bool serialize(std::ostream& sout, int options) const;
      OpenRAVE::BaseXMLReader::ProcessElement startElement(
         const std::string & name, const OpenRAVE::AttributesList & atts);
      bool endElement(const std::string & name);
   };
   typedef boost::shared_ptr<PlannerParameters> PlannerParametersPtr;
   typedef boost::shared_ptr<PlannerParameters const> PlannerParametersConstPtr;
   
   class TagCache : public ompl_multiset::TagCache
   {
   public:
      std::string selffile_header;
      std::string selffile_header_md5;
      FILE * fp;
      std::vector<char> tag_letters;
      void load_vertex(size_t v_index, size_t & v_tag);
      void load_edge(size_t e_index, std::vector< size_t > & e_tags);
      void save_begin();
      void save_vertex(size_t v_index, size_t & v_tag);
      void save_edge(size_t e_index, std::vector< size_t > & e_tags);
      void save_end();
   };

   const OpenRAVE::EnvironmentBasePtr env;
   
   // all of this set by InitPlan
   PlannerParametersConstPtr params;
   OpenRAVE::RobotBasePtr robot;
   std::vector<int> robot_adofs;
   ompl::base::StateSpacePtr ompl_space;
   boost::shared_ptr<ompl_multiset::Family> family;
   boost::shared_ptr<ompl_multiset::FamilyEffortModel> fem;
   boost::shared_ptr<TagCache> tag_cache;
   ompl_multiset::E8Roadmap::RoadmapPtr roadmapgen;
   boost::shared_ptr<ompl_multiset::E8Roadmap> ompl_planner;
   ompl::base::ProblemDefinitionPtr ompl_pdef;
   
   // the set of ilcs
   std::vector<or_multiset::InterLinkCheck> ilcs_self;
   std::vector<or_multiset::InterLinkCheck> ilcs_targ;
   std::vector<or_multiset::InterLinkCheck> ilcs_self_only;
   std::vector<or_multiset::InterLinkCheck> ilcs_targ_only;
   std::vector<or_multiset::InterLinkCheck> ilcs_both;
   
   E8RoadmapSelfCC(OpenRAVE::EnvironmentBasePtr env);
   ~E8RoadmapSelfCC();
   
   bool InitPlan(OpenRAVE::RobotBasePtr robot, std::istream & isParameters);
   bool InitPlan(OpenRAVE::RobotBasePtr robot, OpenRAVE::PlannerBase::PlannerParametersConstPtr params);
   OpenRAVE::PlannerBase::PlannerParametersConstPtr GetParameters() const;
   
   OpenRAVE::PlannerStatus PlanPath(OpenRAVE::TrajectoryBasePtr);
   
   bool CacheCalculateSave(std::ostream & sout, std::istream & sin);
   bool GetTimes(std::ostream & sout, std::istream & sin) const;
   
};

} // namespace or_multiset
