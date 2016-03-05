/*! \file parse_args.cpp
 * \author Chris Dellin <cdellin@gmail.com>
 * \copyright 2015 Carnegie Mellon University
 * \copyright License: BSD
 */

#include <boost/program_options.hpp>

#include <or_lemur/parse_args.h>

boost::program_options::variables_map
or_lemur::parse_args(const boost::program_options::options_description & desc, std::istream & sinput)
{
   std::ostringstream oss;
   oss << sinput.rdbuf();
   return or_lemur::parse_args(desc, oss.str());
}

boost::program_options::variables_map
or_lemur::parse_args(const boost::program_options::options_description & desc, std::string str)
{
   std::vector<std::string> args = boost::program_options::split_unix(str);
   boost::program_options::command_line_parser parser(args);
   parser.options(desc);
   boost::program_options::variables_map vm;
   boost::program_options::store(parser.run(), vm);
   boost::program_options::notify(vm);   
   return vm;
}
