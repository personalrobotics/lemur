/* File: BisectPerm.h
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */
 
namespace ompl_lemur
{

class BisectPerm
{
public:
   BisectPerm() {}
  
   // from calc_order_endgood
   // output is a vector
   // first element is from 0 to n-1 (which one to check)
   // second element is distance to nearest already-evaluated thing
   // (assuming the ends are already evaluated)
   // TODO: back with a vector for faster indexing!
   const std::vector< std::pair<int,int> > & get(int n)
   {
      // look it up in the cache
      std::map<int, const std::vector< std::pair<int,int> > >::iterator it;
      it = cache.find(n);
      if (it != cache.end())
         return it->second;
      // else calculate it
      //printf("calculating perm for n=%d ...\n", n);
      int i;
      int last_true;
      int max_i;
      int max_val;
      std::vector< std::pair<int,int> > perm;
      std::vector<bool> done(n, false);
      std::vector<int> dist(n);
      for (;;)
      {
         last_true = -1;
         for (i=0; i<n; i++)
         {
            if (done[i]) last_true = i;
            dist[i] = (i-last_true);
         }
         last_true = n;
         for (i=n-1; i>=0; i--)
         {
            if (done[i]) last_true = i;
            dist[i] = (last_true-i) < dist[i] ? (last_true-i) : dist[i];
         }
         max_val = 0;
         max_i = 0;
         for (i=0; i<n; i++) if (max_val < dist[i])
         {
            max_val = dist[i];
            max_i = i;
         }
         if (!max_val)
            break;
         perm.push_back(std::make_pair(max_i,max_val));
         done[max_i] = true;
      }
#if 0
      printf("  result: [");
      for (i=0; i<n; i++)
         printf(" %d-%d", perm[i].first, perm[i].second);
      printf(" ]\n");
#endif
      cache.insert(std::make_pair(n,perm));
      return cache[n];
   }

private:
   // cache of edge permutations
   std::map<int, const std::vector< std::pair<int,int> > > cache;
};

} // namespace ompl_lemur
