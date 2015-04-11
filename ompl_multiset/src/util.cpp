/* File: util.cpp
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

#include <cmath>
#include <cstdarg>
#include <cstdlib>
#include <cstdio>
#include <string>
#include <openssl/sha.h>
#include <ompl_multiset/util.h>

using namespace ompl_multiset::util;

std::string ompl_multiset::util::sf(const char * fmt, ...)
{
  va_list ap;
  va_start(ap, fmt);
  int size = vsnprintf(0, 0, fmt, ap);
  va_end(ap);
  char * buf = new char[size+1];
  va_start(ap, fmt);
  vsnprintf(buf, size+1, fmt, ap);
  va_end(ap);
  std::string ret = std::string(buf);
  delete[] buf;
  return ret;
}

bool ompl_multiset::util::startswith(std::string s, std::string prefix)
{
   return s.substr(0,prefix.size()) == prefix;
}

std::string ompl_multiset::util::sha1(std::string in)
{
   unsigned char hashed[20];
   SHA1((const unsigned char *)in.c_str(), in.size(), hashed); // openssl
   std::string out;
   for (int i=0; i<20; i++)
      out += sf("%02x", hashed[i]);
   return out;
}
   
// volume of an n-ball
// https://en.wikipedia.org/wiki/Volume_of_an_n-ball
double ompl_multiset::util::volume_n_ball(unsigned int n)
{
   unsigned int k = n / 2;
   if (n % 2 == 0)
   {
      // even
      double ret = pow(M_PI,k);
      for (unsigned int i=2; i<=k; i++)
         ret /= i;
      return ret;
   }
   else
   {
      // odd
      double ret = 2. * pow(4.*M_PI, k);
      for (unsigned int i=k+1; i<=n; i++)
         ret /= i;
      return ret;
   }
}

std::string ompl_multiset::util::double_to_text(double in)
{
   char buf[2048];
   int min = 0;
   int max = 2047;
   // invariant: min DOESNT WORK, max DOES WORK
   // validate invariants
   sprintf(buf, "%.*f", min, in);
   if (in == strtod(buf,0))
      return std::string(buf);
   sprintf(buf, "%.*f", max, in);
   if (in != strtod(buf,0))
      return std::string(buf);
   // binary search
   for (;;)
   {
      int diff = max - min;
      if (diff == 1)
         break;
      int test = min + diff/2;
      sprintf(buf, "%.*f", test, in);
      if (in == strtod(buf,0))
         max = test;
      else
         min = test;
   }
   sprintf(buf, "%.*f", max, in);
   return std::string(buf);
}
