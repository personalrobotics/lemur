/*! \file FamilyTagCache.h
 * \author Chris Dellin <cdellin@gmail.com>
 * \copyright 2015 Carnegie Mellon University
 * \copyright License: BSD
 */

namespace ompl_lemur
{

/*! \brief Tag cache which uses a family checker to save/load
 *         particular sets.
 * 
 * The tag cache is constructed outside LEMUR, and provided by pointer
 * via its LEMUR::_tag_cache public member.
 * 
 * The outside code should configure it by providing a pointer to the
 * ompl_lemur::FamilyUtilityChecker object which maintains tags for the
 * state space.
 * It can then call FamilyTagCache::addCachedSet() to load particular
 * cache files.
 * 
 * LEMUR will call the tag cache instance's load() and save() methods
 * during planning, or when requested (e.g. via a call to its
 * LEMUR::saveTagCache() method).
 * Since the number of tags managed by the underlying checker will
 * grow during the planning episode,
 * each call to loadBegin() or saveBegin() will resize the appropriate
 * load_valid, load_invalid, and save maps (with 0 values).
 * Then, during the actual load of individual vertices/edges,
 * the correct transitions will be calculated on-demand
 * (and potentially new tags will be created).
 * 
 * This will also create associated tags in the underlying checker.
 * 
 * This may throw on addCachedSet (e.g. if the set was not found).
 * 
 * On a load/save file error, this class will log the error and
 * then ignore it.
 */
template <class VTagMap, class ETagMap>
class FamilyTagCache: public TagCache<VTagMap,ETagMap>
{
public:

   // ok, we need access to the FamilyUtilityChecker object
   FamilyUtilityCheckerPtr _checker;
   
   // we pre-computed mapping vectors given the fixed input family
   // for each cached set
   // for loading
   struct CachedSet
   {
      std::string name; // set name
      std::string filename;
      std::string header;
      size_t var;
      std::vector<size_t> load_valid_map; // map if state is valid
      std::vector<size_t> load_invalid_map; // map if state is invalid
#if 0
      std::vector<char> save_map; // from tag to 'U' 'V' or 'I'
#endif
      // live stuff
      FILE * fp;
   };
   std::map<size_t, CachedSet> _cached_sets;
   
   FamilyTagCache(FamilyUtilityCheckerPtr checker):
      _checker(checker)
   {
   }
   
   void addCachedSet(std::string set_name, std::string cache_filename, std::string header)
   {
      CachedSet newset;
      
      newset.name = set_name;
      newset.filename = cache_filename;
      newset.header = header;
      newset.fp = 0;
      
      // find var for this target set
      // this will throw if it's not found!
      newset.var = _checker->getSetIndex(set_name);
      
#if 0
      // compute save maps
      newset.save_map.resize(num_vertices(_checker->_g));
#endif
      
      // compute load maps
      // these are now extended on loadBegin() from the underlying checker's numTags()
      // and then calculated on-demand!
      //newset.load_valid_map.resize(num_vertices(_checker->_g));
      //newset.load_invalid_map.resize(num_vertices(_checker->_g));
      
      _cached_sets.insert(std::make_pair(newset.var, newset));
   }
   
   // below is used by the planner
   
   bool hasChanged()
   {
      return false;
   }
   
   void loadBegin()
   {
      for (typename std::map<size_t, CachedSet>::iterator
         iset=_cached_sets.begin(); iset!=_cached_sets.end(); iset++)
      {
         const char * filename = iset->second.filename.c_str();
         iset->second.fp = fopen(filename, "r");
         if (!iset->second.fp)
         {
            OMPL_INFORM("Family tag cache file \"%s\" not found.", filename);
            continue;
         }
         // make sure header matches
         size_t size_expected = iset->second.header.size();
         std::string header_read(size_expected, ' ');
         size_t size_read = fread(&header_read[0], 1, size_expected, iset->second.fp);
         if (size_read != size_expected || header_read != iset->second.header)
         {
            OMPL_ERROR("Error, header mismatch in family tag cache file \"%s\".", filename);
            fclose(iset->second.fp);
            iset->second.fp = 0;
            continue;
         }
         // extend the load maps with 0's
         iset->second.load_valid_map.resize(_checker->numTags(), 0);
         iset->second.load_invalid_map.resize(_checker->numTags(), 0);
      }
   }
   
   void loadBatch(size_t batch,
      VTagMap v_tag_map, size_t v_from, size_t v_to,
      ETagMap e_tag_map, size_t e_from, size_t e_to)
   {
      if (batch != 0 || v_from != 0 || e_from != 0)
      {
         OMPL_ERROR("We can only load the first batch for now.");
         return;
      }
      for (typename std::map<size_t, CachedSet>::iterator
         iset=_cached_sets.begin(); iset!=_cached_sets.end(); iset++)
      {
         OMPL_INFORM("Loading batch %lu for set \"%s\" from file \"%s\" ...",
            batch, iset->second.name.c_str(), iset->second.filename.c_str());
         if (!iset->second.fp)
            continue;
         // read batch header
         size_t read_batch;
         size_t read_vertices;
         size_t read_edges;
         int n_stored = fscanf(iset->second.fp, "batch %lu num_vertices %lu num_edges %lu\n",
            &read_batch, &read_vertices, &read_edges);
         if (n_stored != 3 || read_vertices != (v_to-v_from) || read_edges != (e_to-e_from))
         {
            OMPL_ERROR("Batch mismatch from file.");
            continue;
         }
         // get ready to fread
         char buffer[65536];
         size_t bytes_read;
         // read vertex start
         bytes_read = fread(buffer, 1, 9, iset->second.fp);
         if (bytes_read != 9 || strncmp(buffer,"vertices ",9)!=0)
         {
            OMPL_ERROR("Vertex mismatch from file.");
            continue;
         }
         // read vertices loop
         size_t v_index;
         for (v_index=v_from; v_index<v_to;)
         {
            size_t bytes = sizeof(buffer);
            if (bytes > v_to - v_index)
               bytes = v_to - v_index;
            bytes_read = fread(buffer, 1, bytes, iset->second.fp);
            if (bytes_read != bytes)
            {
               OMPL_ERROR("Vertex mismatch from file.");
               break;
            }
            for (size_t i=0; i<bytes; i++,v_index++)
            {
               size_t new_tag;
               switch (buffer[i])
               {
               case 'U': break;
               case 'V':
                  new_tag = iset->second.load_valid_map[v_tag_map[v_index]];
                  if (!new_tag)
                  {
                     new_tag = _checker->tagIfSetKnown(v_tag_map[v_index], iset->second.var, true);
                     iset->second.load_valid_map[v_tag_map[v_index]] = new_tag;
                  }
                  v_tag_map[v_index] = new_tag;
                  break;
               case 'I':
                  new_tag = iset->second.load_invalid_map[v_tag_map[v_index]];
                  if (!new_tag)
                  {
                     new_tag = _checker->tagIfSetKnown(v_tag_map[v_index], iset->second.var, false);
                     iset->second.load_valid_map[v_tag_map[v_index]] = new_tag;
                  }
                  v_tag_map[v_index] = new_tag;
                  break;
               default:
                  OMPL_ERROR("Unknown character: %c", buffer[i]);
               }
            }
         }
         if (v_index<v_to) // premature break
            continue;
         // read edge start
         bytes_read = fread(buffer, 1, 7, iset->second.fp);
         if (bytes_read != 7 || strncmp(buffer,"\nedges ",7)!=0)
         {
            OMPL_ERROR("Edge mismatch from file.");
            continue;
         }
         // read edges loop
         size_t e_index;
         for (e_index=e_from; e_index<e_to;)
         {
            size_t bytes = sizeof(buffer);
            if (bytes > e_to - e_index)
               bytes = e_to - e_index;
            bytes_read = fread(buffer, 1, bytes, iset->second.fp);
            if (bytes_read != bytes)
            {
               OMPL_ERROR("Edge mismatch from file.");
               break;
            }
            for (size_t i=0; i<bytes; i++,e_index++)
            {
               size_t new_tag;
               switch (buffer[i])
               {
               case 'U': break;
               case 'V':
                  new_tag = iset->second.load_valid_map[e_tag_map[e_index]];
                  if (!new_tag)
                  {
                     new_tag = _checker->tagIfSetKnown(e_tag_map[e_index], iset->second.var, true);
                     iset->second.load_valid_map[e_tag_map[e_index]] = new_tag;
                  }
                  e_tag_map[e_index] = new_tag;
                  break;
               case 'I':
                  new_tag = iset->second.load_invalid_map[e_tag_map[e_index]];
                  if (!new_tag)
                  {
                     new_tag = _checker->tagIfSetKnown(e_tag_map[e_index], iset->second.var, false);
                     iset->second.load_valid_map[e_tag_map[e_index]] = new_tag;
                  }
                  e_tag_map[e_index] = new_tag;
                  break;
               default:
                  OMPL_ERROR("Unknown character: %c", buffer[i]);
               }
            }
         }
         if (e_index<e_to) // premature break
            continue;
         // read edge end
         bytes_read = fread(buffer, 1, 1, iset->second.fp);
         if (bytes_read != 1 || strncmp(buffer,"\n ",1)!=0)
         {
            OMPL_ERROR("Edge mismatch from file.");
            continue;
         }
      }
   }

   void loadEnd()
   {
      for (typename std::map<size_t, CachedSet>::iterator
         iset=_cached_sets.begin(); iset!=_cached_sets.end(); iset++)
      {
         if (iset->second.fp)
            fclose(iset->second.fp);
      }
   }
   
   void saveBegin()
   {
      for (typename std::map<size_t, CachedSet>::iterator
         iset=_cached_sets.begin(); iset!=_cached_sets.end(); iset++)
      {
         const char * filename = iset->second.filename.c_str();
         iset->second.fp = fopen(filename, "w");
         if (!iset->second.fp)
         {
            OMPL_ERROR("Could not save to file \"%s\".", filename);
            continue;
         }
         // write header
         fprintf(iset->second.fp, "%s", iset->second.header.c_str());
      }
   }
   
   void saveBatch(size_t batch,
      VTagMap v_tag_map, size_t v_from, size_t v_to,
      ETagMap e_tag_map, size_t e_from, size_t e_to)
   {
      printf("new save not implemented!\n");
#if 0
      for (typename std::map<size_t, CachedSet>::iterator
         iset=_cached_sets.begin(); iset!=_cached_sets.end(); iset++)
      {
         if (!iset->second.fp)
            continue;
         // batch header
         fprintf(iset->second.fp, "batch %lu num_vertices %lu num_edges %lu\n",
            batch, v_to - v_from, e_to - e_from);
         // save vertices
         fprintf(iset->second.fp, "vertices ");
         for (size_t v_index=v_from; v_index<v_to; v_index++)
         {
            size_t & tag = v_tag_map[v_index];
            fprintf(iset->second.fp, "%c", iset->second.save_map[tag]);
         }
         fprintf(iset->second.fp, "\n");
         // save edges
         fprintf(iset->second.fp, "edges ");
         for (size_t e_index=e_from; e_index<e_to; e_index++)
         {
            size_t & tag = e_tag_map[e_index];
            fprintf(iset->second.fp, "%c", iset->second.save_map[tag]);
         }
         fprintf(iset->second.fp, "\n");
      }
#endif
   }
   
   void saveEnd()
   {
      for (typename std::map<size_t, CachedSet>::iterator
         iset=_cached_sets.begin(); iset!=_cached_sets.end(); iset++)
      {
         if (iset->second.fp)
            fclose(iset->second.fp);
      }
   }
};

} // namespace ompl_lemur
