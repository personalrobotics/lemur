#!/usr/bin/env python3

# File: plot2dimg-allpaths.py
# Author: Chris Dellin <cdellin@gmail.com>
# Copyright: 2014, 2015 Carnegie Mellon University
# License: None

import math
import os
import sys
import cairo

# cairo origin is in top left (opposite of world x,y)

if sys.version_info.major != 3:
   raise RuntimeError('python 3 only!')

print('dumping ...')

# r p rbn
colors = [
   ('U U U', (0.7,0.7,0.7)),
   ('V U U', (0., 0., 0. )),
   ('V V U', (0., 0., 0. )),
   ('V I U', (0., 0., 0. )),
   ('I U U', (1., 0., 0. )),
   ('I I U', (1., 0., 0. )),
   ('U I U', (1., 0., 1. )),
   # rbn stuff
   ('U U V', (0., 0., 0. )),
   ('U U I', (1., 0., 0. )),
]

# create the image
s = cairo.ImageSurface.create_from_png(open('w-padding1-nop.png','rb'))
c = cairo.Context(s)

# add the final path from all dumps
for filename in os.listdir('dumps'):
   print('dumping from dumps/{} ...'.format(filename))

   # find the last path (assumed valid)
   path = None
   
   for num,line in enumerate(open('dumps/{}'.format(filename))):
      line = line.strip()
   
      cmd,args = line.split(None,1)
      if cmd != 'candidate_path':
         continue
      
      # save the path
      path = []
      spoints = args.split()[1:]
      for spoint in spoints:
         (x,y) = tuple(map(float,spoint.split(',')))
         path.append((x,y))
   
   if path is None:
      raise RuntimeError('no path found!')

   if path is not None:
      c.set_source_rgb(0.0, 0.0, 1.0)
      c.set_line_width(0.1)
      for i,(x,y) in enumerate(path):
         if i == 0:
            c.move_to(y, x)
         else:
            c.line_to(y, x)
      c.stroke()

s.write_to_png('dump-output.png')
   
   
