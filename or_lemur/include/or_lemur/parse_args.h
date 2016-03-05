/*! \file parse_args.h
 * \author Chris Dellin <cdellin@gmail.com>
 * \copyright 2015 Carnegie Mellon University
 * \copyright License: BSD
 */

#define OR_LEMUR_PRINT_USAGE(desc) \
   do { \
      std::stringstream ss; \
      ss << desc; \
      RAVELOG_INFO("Usage:\n"); \
      std::string line; \
      while (std::getline(ss, line)) \
         RAVELOG_INFO("%s\n", line.c_str()); \
   } while (0)

namespace or_lemur
{

boost::program_options::variables_map
parse_args(const boost::program_options::options_description & desc, std::istream & sinput);

boost::program_options::variables_map
parse_args(const boost::program_options::options_description & desc, std::string str);

} // namespace or_lemur
