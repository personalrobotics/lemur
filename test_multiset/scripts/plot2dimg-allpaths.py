#!/usr/bin/env python3

# File: plot2dimg-allpaths.py
# Author: Chris Dellin <cdellin@gmail.com>
# Copyright: 2015 Carnegie Mellon University
# License: BSD

import argparse
import glob
import math
import os
import sys
import cairo

# cairo origin is in top left (opposite of world x,y)

if sys.version_info.major != 3:
   raise RuntimeError('python 3 only!')

parser = argparse.ArgumentParser()
parser.add_argument('--input-img',required=True)
parser.add_argument('--input-dumps-glob',required=True)
parser.add_argument('--output-png',required=True)
args = parser.parse_args()

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

# save averages
avg_count = 0
avg_len = 0.0
avg_checkcost = 0.0

# create the image
s = cairo.ImageSurface.create_from_png(open(args.input_img,'rb'))
c = cairo.Context(s)

# add the final path from all dumps
for filename in sorted(glob.glob(args.input_dumps_glob)):
   #print('dumping from {} ...'.format(filename))

   # find the last path (assumed valid)
   path = None
   final_check_cost = None
   
   for num,line in enumerate(open(filename)):
      line = line.strip()
   
      cmd,cmdargs = line.split(None,1)
      if cmd == 'candidate_path':
         # save the path
         path = []
         spoints = cmdargs.split()[1:]
         for spoint in spoints:
            (x,y) = tuple(map(float,spoint.split(',')))
            path.append((x,y))
      if cmd == 'final_check_cost':
         avg_checkcost += float(cmdargs)
   
   if path is None:
      raise RuntimeError('no path found!')
   
   # get path length
   path_len = 0
   for i in range(len(path)-1):
      x1,y1 = path[i]
      x2,y2 = path[i+1]
      path_len += math.sqrt((x2-x1)**2 + (y2-y1)**2)
      
   print('  found len:', path_len)
   
   avg_len += path_len
   avg_count += 1
   
   c.set_source_rgb(0.0, 0.0, 1.0)
   c.set_line_width(0.4)
   for i,(x,y) in enumerate(path):
      if i == 0:
         c.move_to(y, x)
      else:
         c.line_to(y, x)
   c.stroke()

avg_len /= avg_count
avg_checkcost /= avg_count
print('averaged {} values!'.format(avg_count))
print('average path length:', avg_len)
print('average check cost:', avg_checkcost)

s.write_to_png(args.output_png)
