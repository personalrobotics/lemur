from __future__ import print_function
from __future__ import unicode_literals
from __future__ import absolute_import
from __future__ import division
from collections import namedtuple as nt

AAGrid = nt('AAGrid', 'res')
FromFile = nt('FromFile', 'filename root_radius')
Halton = nt('Halton', 'num radius')
HaltonDens = nt('HaltonDens', 'num_per_batch radius_first_batch')
HaltonOffDens = nt('HaltonOffDens', 'num_per_batch gamma_factor scaling seed')
RGG = nt('RGG', 'num radius seed')
RGGDens = nt('RGGDens', 'num_per_batch radius_first_batch seed')
RGGDensConst = nt('RGGDensConst', 'num_per_batch radius seed')

def make_cached(roadmap_type):
   cached_name = 'Cached{}'.format(roadmap_type.__name__)
   cached_fields = list(roadmap_type._fields) + ['is_cache_required']
   cached_type = nt(cached_name, cached_fields)
   cached_type.__new__.__defaults__ = (False,)
   return cached_type

CachedAAGrid = make_cached(AAGrid)
CachedHalton = make_cached(Halton)
CachedHaltonDens = make_cached(HaltonDens)
CachedHaltonOffDens = make_cached(HaltonOffDens)
CachedRGG = make_cached(RGG)
CachedRGGDens = make_cached(RGGDens)
CachedRGGDensConst = make_cached(RGGDensConst)

def get_roadmap_id(roadmap):
   roadmap_type = type(roadmap).__name__
   if roadmap_type.startswith('Cached'):
      roadmap_type = roadmap_type[6:]
   roadmap_id = roadmap_type
   for k,v in sorted(roadmap._asdict().items()):
      roadmap_id += ',{}'.format(v)
   return roadmap_id
