from __future__ import print_function
from __future__ import unicode_literals
from __future__ import absolute_import
from __future__ import division
from collections import namedtuple as nt

AAGrid = nt('AAGrid', 'res')
FromFile = nt('FromFile', 'filename root_radius')
Halton = nt('Halton', 'num radius')
HaltonDens = nt('HaltonDens', 'num_per_batch radius_first_batch')
HaltonOffDens = nt('HaltonOffDens', 'num_per_batch radius_first_batch seed')
RGG = nt('RGG', 'num radius seed')
RGGDens = nt('RGGDens', 'num_per_batch radius_first_batch seed')
RGGDensConst = nt('RGGDensConst', 'num_per_batch radius seed')

CachedAAGrid = nt('CachedAAGrid', AAGrid._fields)
CachedHalton = nt('CachedHalton', Halton._fields)
CachedHaltonDens = nt('CachedHaltonDens', HaltonDens._fields)
CachedHaltonOffDens = nt('CachedHaltonOffDens', HaltonOffDens._fields)
CachedRGG = nt('CachedRGG', RGG._fields)
CachedRGGDens = nt('CachedRGGDens', RGGDens._fields)
CachedRGGDensConst = nt('CachedRGGDensConst', RGGDensConst._fields)

def get_roadmap_id(roadmap):
   roadmap_type = type(roadmap).__name__
   if roadmap_type.startswith('Cached'):
      roadmap_type = roadmap_type[6:]
   roadmap_id = roadmap_type
   for k,v in sorted(roadmap._asdict().items()):
      roadmap_id += ',{}'.format(v)
   return roadmap_id
