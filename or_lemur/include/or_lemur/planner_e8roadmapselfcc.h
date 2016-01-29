/* File: planner_e8roadmapselfcc.h
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

namespace or_lemur
{

class E8RoadmapSelfCC : public OpenRAVE::PlannerBase
{
public:
   
   class TagCache : public ompl_lemur::TagCache<ompl_multiset::E8Roadmap::VIdxTagMap,ompl_multiset::E8Roadmap::EIdxTagsMap>
   {
   public:
      std::string selffile_header;
      std::string selffile_header_md5;
      FILE * fp;
      // for saving
      std::vector<char> tag_letters;
      // for loading
      size_t tag_self_valid;
      size_t tag_self_invalid;
      // methods
      TagCache();
      void load_begin(void);
      void load_vertices(ompl_lemur::E8Roadmap::VIdxTagMap v_tag_map, size_t v_from, size_t v_to);
      void load_edges(ompl_lemur::E8Roadmap::EIdxTagsMap e_tags_map, size_t e_from, size_t e_to);
      void load_end(void);
      void save_begin(void);
      void save_vertices(ompl_lemur::E8Roadmap::VIdxTagMap v_tag_map, size_t v_from, size_t v_to);
      void save_edges(ompl_lemur::E8Roadmap::EIdxTagsMap e_tags_map, size_t e_from, size_t e_to);
      void save_end(void);
   };

   const OpenRAVE::EnvironmentBasePtr env;
   
   // all of this set by InitPlan
   E8RoadmapParametersConstPtr params_ptr;
   std::string alglog;
   bool do_alglog_append;
   std::string graph;
   OpenRAVE::RobotBasePtr robot;
   std::vector<int> robot_adofs;
   ompl::base::StateSpacePtr ompl_space;
   boost::shared_ptr<ompl_lemur::Family> family;
   boost::shared_ptr<ompl_lemur::FamilyEffortModel> fem;
   boost::shared_ptr<TagCache> tag_cache;
   ompl_lemur::E8Roadmap::RoadmapPtr roadmapgen;
   boost::shared_ptr<ompl_lemur::E8Roadmap> ompl_planner;
   ompl::base::ProblemDefinitionPtr ompl_pdef;
   
   // the set of ilcs
   std::vector<or_lemur::InterLinkCheck> ilcs_self;
   std::vector<or_lemur::InterLinkCheck> ilcs_targ;
   std::vector<or_lemur::InterLinkCheck> ilcs_self_only;
   std::vector<or_lemur::InterLinkCheck> ilcs_targ_only;
   std::vector<or_lemur::InterLinkCheck> ilcs_both;
   
   E8RoadmapSelfCC(OpenRAVE::EnvironmentBasePtr env);
   ~E8RoadmapSelfCC();
   
   bool InitPlan(OpenRAVE::RobotBasePtr robot, std::istream & isParameters);
   bool InitPlan(OpenRAVE::RobotBasePtr robot, OpenRAVE::PlannerBase::PlannerParametersConstPtr params);
   OpenRAVE::PlannerBase::PlannerParametersConstPtr GetParameters() const;
   
   OpenRAVE::PlannerStatus PlanPath(OpenRAVE::TrajectoryBasePtr);
   
   bool GetSelfHeader(std::ostream & sout, std::istream & sin) const;
   bool GetSelfHash(std::ostream & sout, std::istream & sin) const;
   bool CacheCalculateSave(std::ostream & sout, std::istream & sin);
   bool GetTimes(std::ostream & sout, std::istream & sin) const;
   
};

} // namespace or_lemur
