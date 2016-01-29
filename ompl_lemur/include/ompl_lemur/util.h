/* File: util.h
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
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

std::size_t get_prime(std::size_t which);

double halton(std::size_t prime, std::size_t index);

} // namespace util
} // namespace ompl_lemur
