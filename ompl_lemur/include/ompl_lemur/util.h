/*! \file util.h
 * \author Chris Dellin <cdellin@gmail.com>
 * \copyright 2015 Carnegie Mellon University
 * \copyright License: BSD
 */

namespace ompl_lemur
{
namespace util
{

std::string sf(const char * fmt, ...);

bool startswith(std::string s, std::string prefix);

std::string sha1(std::string in);

std::string file_sha1(std::string in);

// volume of an n-ball
double volume_n_ball(unsigned int n);

std::string double_to_text(double in);

// this rounds the given double value to 14 decimal digits
// (in scientific notation)
// this is a workaround for a bug in the boost lexical_cast
// https://svn.boost.org/trac/boost/ticket/10639
// this workaround assumes that the actual value being specified
// does not require more than 14 digits in decimal
void snap_decimal(double & value);

std::size_t get_prime(std::size_t which);

double halton(std::size_t prime, std::size_t index);

} // namespace util
} // namespace ompl_lemur
