/*! \file module_family.h
 * \author Chris Dellin <cdellin@gmail.com>
 * \copyright 2015 Carnegie Mellon University
 * \copyright License: BSD
 */

namespace or_lemur
{

/*!
 * 
 * This is tied to a particular robot instance
 * 
 * the job of the family object is to maintain/deduplicate
 * canonical names for linkdata objects,
 * save/load to disk, etc
 * 
 * it's either loaded from disk, or it isnt!
 * 
 * this maintains pointers to sets
 * sets persist if they're named,
 * or if other people own pointers to them
 * 
 * a family is a set of sets
 * bound to a particular robot (for now)
 * 
 * NOTE: The current set includes all robot self and environment
 * inter-link collisions, with the following known differences:
 * - Initially colliding links of the grabber (with the grabbee)
 *   are not ignored.
 * - Pairs are only checked if they traverse a joint which contains
 *   an active DOF.
 */
class FamilyModule : public OpenRAVE::ModuleBase
{
private:
   struct Set; // opaque type for module users
   
public:
   // types
   // setptrs are unique w.r.t. what checks they entail
   typedef boost::shared_ptr<Set> SetPtr;
   
   // first:antecedents, second:consequent
   // C > A1 n A2 n A2
   typedef std::pair< std::set<SetPtr>, SetPtr > Relation;
   
   struct Family
   {
      std::set<SetPtr> sets;
      std::set<Relation> relations;
   };
   
   typedef boost::function<bool (const std::vector<OpenRAVE::dReal> & values)> Indicator;
   
   FamilyModule(OpenRAVE::EnvironmentBasePtr penv);
   ~FamilyModule();
   
   std::string GetInstanceId();
   
   // on environment add/remove
   int main(const std::string & cmd);
   void Destroy();
   
   std::string GetFamilyId();
   
   // note: these will generate the new set and put it in _sets_all,
   // but it's unnamed, so on next cleanup() it will be removed
   // if the pointer is destroyed
   
   SetPtr GetCurrentSet();
   
   SetPtr GetSetFromExpression(std::string input);
   
   // returns nothing if literal not bound
   SetPtr GetSet(std::string literal);
   
   void Let(std::string literal, SetPtr set);
   
   void Del(std::string literal);
   
   std::string GetHeaderFromSet(SetPtr set);
   
   SetPtr GetSetFromHeader(std::string set_header);
   
   // returns all currently named sets,
   // as well as other sets need for all of their relations
   Family GetCurrentFamily(std::set<SetPtr> sets_ext = std::set<SetPtr>());
   
   // should return a map from SetPtr -> bool (*)(vec<dReal> q)
   // internally, this delegates to something else?
   // these indicators are only valid if the env is unchanged exernally!
   // this ignores the relations
   std::map< SetPtr, std::pair<double,Indicator> > GetIndicators(const Family & family);
   
   // makes up names for unbound sets
   std::map<SetPtr,std::string> GetCanonicalNames(const Family & family);

   bool CmdGetInstanceId(std::ostream & soutput, std::istream & sinput);
   
   bool CmdGetFamilyId(std::ostream & soutput, std::istream & sinput);

   // assignment statement
   // Let A = $live
   // Let B = $file{something.txt}
   // Let C = A \ B
   // Let D = (A u C) n B
   bool CmdLet(std::ostream & soutput, std::istream & sinput);
   
   // input: expression
   bool CmdNamesOf(std::ostream & soutput, std::istream & sinput);
   
   bool CmdPrintCurrentFamily(std::ostream & soutput, std::istream & sinput);
   
   // input: expression
   bool CmdGetHeaderFromSet(std::ostream & soutput, std::istream & sinput);

private:

   /*! \brief these are set once on init, and then fixed */
   bool _initialized;
   bool _has_use_baked_checker; // whether it was passed
   bool _use_baked_checker;
   boost::weak_ptr<OpenRAVE::RobotBase> _robot;
   std::vector<int> _active_dofs;
   std::string _id;
   std::vector<int> _proxidxs;
   
   // types
   
   // these are deduplicated, between in-memory and on-disk!
   // (this is a hard problem in itself; what if two processes generate
   // slightly different ones? simultaneously?)
   // a posed link is relative to a proxidx link (relative to this family only)
   struct PosedLink
   {
      int proxidx;
      std::string geomhash;
      OpenRAVE::Transform pose;
   };

   /*! \brief inter-link check
    * first < second (pointers)
    */
   typedef std::pair<PosedLink *, PosedLink *> Check;

   struct Atom
   {
      std::set<Check> checks; // set of checks this atom entails
   };

   struct Set
   {
      std::set<std::string> literals; // how everyone pointing to me from _sets_bound
      std::set<Check> checks; // set of checks this set entails; doesnt change
      std::set<Atom *> atoms;
   };
   
   // the family module owns everything!
   // it's sure to deduplicate stuff too
   // this owns PosedLinks
   //
   // the family ONLY maintains posedlinks needed by existing sets!
   std::map< std::pair<int,std::string>, std::set<PosedLink *> > _posedlinks;
   
   // this owns the Atoms
   std::set<Atom *> _atoms;
   
   // all sets (but may still be bound to literals)
   // when this is the only pointer, we can clean up
   std::set<SetPtr> _sets_all;
   
   // sets we want to own
   // we delete from here
   // sets currently bound to a literal
   std::map<std::string, SetPtr> _sets_bound;
   
   // this function considers all links in the environment
   // (enabled or not)
   // and finds an existing matching posedlink if it exists
   // (we dont add them cause they might not be used)
   std::map<PosedLink *, OpenRAVE::KinBody::LinkPtr> live_links();
   
   SetPtr set_from_checks(const std::set<Check> & checks);
   
};

} // namespace or_lemur
