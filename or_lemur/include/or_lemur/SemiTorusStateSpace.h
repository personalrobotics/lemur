/*! \file SemiTorusStateSpace.h
 * \author Chris Dellin <cdellin@gmail.com>
 * \copyright 2015 Carnegie Mellon University
 * \copyright License: BSD
 */

namespace or_lemur {

/*! \brief Semi-torus OMPL state space for wrapping dimensions.
 * 
 * This is a generalization of ompl::base::RealVectorStateSpace
 * which supports wrapping on each dimension individually
 * (that is, the dimension's lower bound and upper bound correspond).
 * This is especially useful for handling robots with circular joints.
 * It should be funcionally equivalent to building a coupound state
 * space with many SO(2) components, except it should be faster because
 * all states are stored contiguously.
 */
class SemiTorusStateSpace: public ompl::base::RealVectorStateSpace
{
public:
   SemiTorusStateSpace(unsigned int dim=0);
   
   virtual void setIsWrapping(const std::vector<bool> &isWrapping);
   virtual const std::vector<bool> & getIsWrapping() const { return isWrapping_; }

   virtual void addDimension(double minBound=0.0, double maxBound=0.0, bool isWrapping=false);
   virtual void addDimension(const std::string &name, double minBound=0.0, double maxBound=0.0, bool isWrapping=false);
   
   virtual double getMaximumExtent() const;
   virtual double distance(const ompl::base::State *state1, const ompl::base::State *state2) const;
   virtual bool equalStates(const ompl::base::State *state1, const ompl::base::State *state2) const;
   virtual void interpolate(const ompl::base::State *from, const ompl::base::State *to, const double t, ompl::base::State *state) const;
   
private:
   std::vector<bool> isWrapping_;
};

} // namespace or_lemur
