/*! \file heap_indexed.h
 * \author Chris Dellin <cdellin@gmail.com>
 * \copyright 2015 Carnegie Mellon University
 * \copyright License: BSD
 * 
 * \brief Contains pr_bgl::heap_indexed.
 */

/* requires:
#include <vector>
 */

namespace pr_bgl
{

/*! \brief Binary min-heap with index lookups.
 * 
 * The HeapIndexed class implements a binary min-heap with index
 * lookups. Elements are identified with an index value
 * (e.g. [0,num_vertices)). The heap also maintains a vector backing,
 * wich each element at a particular location.
 *
 * \todo this may duplicate functionality implemented in BGL.
 * 
 * KeyType: e.g. double
 *
\verbatim
for example:
backing: [ (??,?), (0.3,3), (0.5,7), (0.7,2), (0.6,1) ]
locs: [0, 4, 3, 1, 0, 0, 0, 2]
\endverbatim
 */
template <typename KeyType>
class heap_indexed
{
   struct element
   {
      KeyType key;
      size_t idx;
      element(KeyType key, size_t idx): key(key), idx(idx) {}
   };
   std::vector<element> backing;
   std::vector<size_t> locs; // indexed by idx, 0=not in heap
   
public:
   heap_indexed(): backing(1,element(KeyType(),0)), locs(0) {}
   
   // simple queries
   size_t size()
   {
      return backing.size() - 1;
   }
   bool contains(size_t idx)
   {
      if (idx < locs.size() && locs[idx])
         return true;
      else
         return false;
   }
   
   // make an inconsistent heap consistent
   // by up-heaping the element idx with key key
   // which is CURRENTLY at loc loc
   inline void up_heap(size_t idx, KeyType key, size_t loc)
   {
      for (;;)
      {
         // get parent location
         size_t loc_parent = loc / 2;
         // done if we have no parent or are bigger or equal to parent
         if (!loc_parent || backing[loc_parent].key <= key)
            return;
         // upheap!
         size_t idx_parent = backing[loc_parent].idx;
         std::swap(backing[loc_parent], backing[loc]);
         std::swap(locs[idx_parent], locs[idx]);
         loc = loc_parent;
      }
   }
   
   // make an inconsistent heap consistent
   // by down-heaping the element idx with key key
   // which is CURRENTLY at loc loc
   inline void down_heap(size_t idx, KeyType key, size_t loc)
   {
      for (;;)
      {
         // get child locations
         size_t loc_left = 2*loc;
         size_t loc_right = 2*loc+1;
         // find largest among family
         size_t loc_min = loc;
         KeyType key_min = key;
         if (loc_left < backing.size() && backing[loc_left].key < key_min)
         {
            loc_min = loc_left;
            key_min = backing[loc_left].key;
         }
         if (loc_right < backing.size() && backing[loc_right].key < key_min)
         {
            loc_min = loc_right;
            key_min = backing[loc_right].key;
         }
         // are we already the min in the family?
         if (loc_min == loc)
            break;
         size_t idx_min = backing[loc_min].idx;
         // ok, down-heap!
         std::swap(backing[loc], backing[loc_min]);
         std::swap(locs[idx], locs[idx_min]);
         loc = loc_min;
      }
   }
   
   // only valid if contains(idx) is false
   void insert(size_t idx, KeyType key)
   {
      // resize loc if necessary
      if (locs.size() < idx+1)
         locs.resize(idx+1, 0);
      // insert at end of heap
      size_t loc = backing.size();
      backing.push_back(element(key,idx));
      locs[idx] = loc;
      // up-heap child (at loc)
      up_heap(idx,key,loc);
   }
   
   // only valid if contains(idx) is true
   void update(size_t idx, KeyType key)
   {
      size_t loc = locs[idx];
      if (key < backing[loc].key)
      {
         backing[loc].key = key;
         up_heap(idx, key, loc);
         return;
      }
      if (key > backing[loc].key)
      {
         backing[loc].key = key;
         down_heap(idx, key, loc);
         return;
      }
   }
   
   // only valid if its in the heap
   void remove(size_t idx)
   {
      size_t loc = locs[idx];
      KeyType key = backing[loc].key;
      size_t loc_last = backing.size()-1;
      // if already last, then we're basically done
      if (loc == loc_last)
      {
         backing.pop_back();
         locs[idx] = 0;
         return;
      }
      // else swap with whatever's last
      size_t idx_last = backing[loc_last].idx;
      backing[loc] = backing[loc_last];
      backing.pop_back();
      locs[idx_last] = loc;
      locs[idx] = 0;
      // maintain proper heap order (old last item is now in loc)
      if (backing[loc].key < key)
         up_heap(idx_last, backing[loc].key, loc);
      if (backing[loc].key > key)
         down_heap(idx_last, backing[loc].key, loc);
   }
   
   // assumes non-empty
   KeyType top_key()
   {
      return backing[1].key;
   }
   size_t top_idx()
   {
      return backing[1].idx;
   }
   void remove_min()
   {
      // remove
      locs[top_idx()] = 0;
      if (size() > 1)
      {
         // move last heap element to root
         backing[1] = backing[backing.size()-1];
         locs[backing[1].idx] = 1;
      }
      // resize heap backing
      backing.pop_back();
      // down-heap the new root
      if (size())
         down_heap(backing[1].idx, backing[1].key, 1);
   }
   
   void print()
   {
      printf("backing:");
      for (size_t ui=0; ui<backing.size(); ui++)
         printf(" %u:(%.1f,%u)", ui, backing[ui].key, backing[ui].idx);
      printf("\n");
      printf("locs:");
      for (size_t ui=0; ui<locs.size(); ui++)
         printf(" %u:%u", ui, locs[ui]);
      printf("\n");
   }
};

} // namespace pr_bgl
