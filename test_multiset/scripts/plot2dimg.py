#!/usr/bin/env python3

# File: plot2dimg.py
# Author: Chris Dellin <cdellin@gmail.com>
# Copyright: 2014, 2015 Carnegie Mellon University
# License: None

import math
import sys
import cairo

# cairo origin is in top left (opposite of world x,y)

if sys.version_info.major != 3:
   raise RuntimeError('python 3 only!')

print('dumping ...')

vertices = []
edges = []

# map from v or e to statuses string
statusdict = {}

# r p rbn
colors = [
   ('U U U', (0.7,0.7,0.7)),
   ('I U U', (1., 0., 0. )),
   ('I I U', (1., 0., 0. )),
   ('U I U', (1., 0., 1. )),
   ('V U U', (0., 0., 0. )),
   ('V V U', (0., 0., 0. )),
   ('V I U', (0., 0., 0. )),
   # rbn stuff
   ('U U V', (0., 0., 0. )),
   ('U U I', (1., 0., 0. )),
]

# path is sticky
path = None
imgnum = 1
first_path = False
for num,line in enumerate(open('dump.txt')):
   line = line.strip()
   
   cmd,args = line.split(None,1)
   if cmd == 'add_vertex':
      path = None
      sxy,statuses = args.split(None,1)
      (x,y) = tuple(map(float,sxy.split(',')))
      vertices.append((x,y))
      statusdict[(x,y)] = statuses
   elif cmd == 'add_edge':
      path = None
      v1,v2,statuses = args.split(None,2)
      (x1,y1) = tuple(map(float,v1.split(',')))
      (x2,y2) = tuple(map(float,v2.split(',')))
      edges.append(((x1,y1),(x2,y2)))
      statusdict[((x1,y1),(x2,y2))] = statuses
   elif cmd == 'candidate_path':
      first_path = True
      path = []
      spoints = args.split()[1:]
      for spoint in spoints:
         (x,y) = tuple(map(float,spoint.split(',')))
         path.append((x,y))
   elif cmd == 'update_vertex':
      sxy,statuses = args.split(None,1)
      (x,y) = tuple(map(float,sxy.split(',')))
      if (x,y) not in statusdict:
         raise RuntimeError('update failed, unknown vertex!')
      statusdict[(x,y)] = statuses
   elif cmd == 'update_edge':
      v1,v2,statuses = args.split(None,2)
      (x1,y1) = tuple(map(float,v1.split(',')))
      (x2,y2) = tuple(map(float,v2.split(',')))
      key1 = ((x1,y1),(x2,y2))
      key2 = ((x2,y2),(x1,y1))
      if key1 in statusdict:
         statusdict[key1] = statuses
      elif key2 in statusdict:
         statusdict[key2] = statuses
      else:
         raise RuntimeError('update failed, unknown edge!')
   elif cmd in ('candidate_path_cost_vertex','candidate_path_cost_edge'):
      continue
   else:
      raise RuntimeError('unknown cmd: {}'.format(cmd))
   
   s = cairo.ImageSurface.create_from_png(open('w-padding1.png','rb'))
   c = cairo.Context(s)
   
   if path is not None:
      c.set_source_rgb(0., 255., 0.)
      c.set_line_width(10.0)
      for i,(x,y) in enumerate(path):
         if i == 0:
            c.move_to(y, x)
         else:
            c.line_to(y, x)
      c.stroke()
   
   
   for color_statuses,color in colors:
      for ((x1,y1),(x2,y2)) in edges:
         key1 = ((x1,y1),(x2,y2))
         key2 = ((x2,y2),(x1,y1))
         if key1 in statusdict:
            statuses = statusdict[key1]
         else:
            statuses = statusdict[key2]
         if statuses != color_statuses:
            continue
         c.set_source_rgb(*color)
         c.set_line_width(2.0)
         c.move_to(y1, x1)
         c.line_to(y2, x2)
         c.stroke()
   for ((x1,y1),(x2,y2)) in edges:
      key1 = ((x1,y1),(x2,y2))
      key2 = ((x2,y2),(x1,y1))
      if key1 in statusdict:
         statuses = statusdict[key1]
      else:
         statuses = statusdict[key2]
      for color_statuses,_ in colors:
         if statuses == color_statuses:
            break
      else:
         raise RuntimeError('unknown statuses: {}'.format(statuses))
   
   for (x,y) in vertices:
      statuses = statusdict[(x,y)]
      for color_statuses,color in colors:
         if statuses == color_statuses:
            c.set_source_rgb(*color)
            break
      else:
         raise RuntimeError('unknown statuses: {}'.format(statuses))
      c.arc(y, x, 3.0, 0, 2*math.pi)
      c.fill()

   if first_path:
      imgnum += 1
   s.write_to_png('dump/frame-{:05d}.png'.format(imgnum))
   
   
   
