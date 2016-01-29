/* File: util.cpp
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

#include <cmath>
#include <cstdarg>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <openssl/sha.h>
#include <ompl_lemur/util.h>

namespace {

int primes[] =
{
     2,   3,   5,   7,  11,  13,  17,  19,  23,  29,
    31,  37,  41,  43,  47,  53,  59,  61,  67,  71,
    73,  79,  83,  89,  97, 101, 103, 107, 109, 113,
   127, 131, 137, 139, 149, 151, 157, 163, 167, 173,
   179, 181, 191, 193, 197, 199, 211, 223, 227, 229,
   233, 239, 241, 251, 257, 263, 269, 271, 277, 281,
   283, 293, 307, 311, 313, 317, 331, 337, 347, 349,
   353, 359, 367, 373, 379, 383, 389, 397, 401, 409,
   419, 421, 431, 433, 439, 443, 449, 457, 461, 463,
   467, 479, 487, 491, 499, 503, 509, 521, 523, 541
};

} // anonymous namespace

std::string ompl_lemur::util::sf(const char * fmt, ...)
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

bool ompl_lemur::util::startswith(std::string s, std::string prefix)
{
   return s.substr(0,prefix.size()) == prefix;
}

std::string ompl_lemur::util::sha1(std::string in)
{
   unsigned char hashed[20];
   SHA1((const unsigned char *)in.c_str(), in.size(), hashed); // openssl
   std::string out;
   for (int i=0; i<20; i++)
      out += sf("%02x", hashed[i]);
   return out;
}

std::string ompl_lemur::util::file_sha1(std::string fname)
{
   std::ifstream fp;
   fp.open(fname.c_str());;
   std::stringstream ss;
   ss << fp.rdbuf();
   return sha1(ss.str());
}

// volume of an n-ball
// https://en.wikipedia.org/wiki/Volume_of_an_n-ball
double ompl_lemur::util::volume_n_ball(unsigned int n)
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

std::string ompl_lemur::util::double_to_text(double in)
{
   char buf[2048];
   // invariant: min DOESNT WORK, max DOES WORK
   // validate invariants
   int min = 0;
   sprintf(buf, "%.*f", min, in);
   if (in == strtod(buf,0))
      return std::string(buf);
   // what is it at 1?
   sprintf(buf, "%.*f", 1, in);
   int max = sizeof(buf)-strlen(buf);
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


// prime[0] = 2
// returns 0 if its not hardcoded!
std::size_t ompl_lemur::util::get_prime(std::size_t which)
{
   if (sizeof(primes)/sizeof(primes[0]) <= which)
      return 0;
   return primes[which];
}

// index is 0-indexed
double ompl_lemur::util::halton(std::size_t prime, std::size_t index)
{
   double sample = 0.0;
   double denom = prime;
   for (index++; index; index/=prime, denom*=prime)
      sample += (index % prime) / denom;
   return sample;
}
