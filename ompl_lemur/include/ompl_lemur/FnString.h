/* File: FnString.h
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

namespace ompl_lemur
{

class FnString
{
public:
   std::string name;
   std::string argstring;
   std::vector< std::pair<std::string,std::string> > args;
   FnString(const std::string & in)
   {
      int state = 0; // reading fn name
      int argstring_start = 0;
      int token_start = 0;
      int argval_numparens;
      for (unsigned int i=0; i<in.size(); i++)
      {
         if (state == 0) // reading fn name
         {
            if (in[i] == '(')
            {
               name = in.substr(token_start,i-token_start);
               state = 1; // reading arg name
               token_start = i + 1;
               argstring_start = i + 1;
            }
            continue;
         }
         if (state == 1)
         {
            if (in[i] == '=')
            {
               std::string argname = in.substr(token_start,i-token_start);
               args.push_back(std::make_pair(argname,""));
               state = 2; // reading arg value
               token_start = i + 1;
               argval_numparens = 0;
            }
            continue;
         }
         if (state == 2)
         {
            if (argval_numparens > 0)
            {
               if (in[i] == ')')
                  argval_numparens--;
            }
            else if (in[i] == '(')
            {
               argval_numparens++;
            }
            else if (in[i] == ' ' || in[i] == ')')
            {
               std::string argval = in.substr(token_start,i-token_start);
               args.back().second = argval;
               state = 1;
               token_start = i + 1;
               if (in[i] == ')')
               {
                  argstring = in.substr(argstring_start,i-argstring_start);
                  state = 3;
               }
            }
            continue;
         }
         if (state == 3)
         {
            throw std::runtime_error("FnString borked early!");
         }
      }
   }
};

} // namespace ompl_lemur
