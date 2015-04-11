#!/usr/bin/env python3

# File: plot2d.py
# Author: Chris Dellin <cdellin@gmail.com>
# Copyright: 2015 Carnegie Mellon University
# License: BSD

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

# four colors
colors = [
   (1.0, 0.0, 0.0), # r - red
   (1.0, 0.5, 0.0), # p - orange
   (0.0, 1.0, 0.0), # a - green
   (0.0, 0.0, 1.0)  # b - blue
]

for num,line in enumerate(open('dump.txt')):
   line = line.strip()
   
   path = None
   
   cmd,args = line.split(None,1)
   if cmd == 'add_vertex':
      sxy,statuses = args.split(None,1)
      (x,y) = tuple(map(float,sxy.split(',')))
      vertices.append((x,y))
      statusdict[(x,y)] = statuses
   elif cmd == 'add_edge':
      v1,v2,statuses = args.split(None,2)
      (x1,y1) = tuple(map(float,v1.split(',')))
      (x2,y2) = tuple(map(float,v2.split(',')))
      edges.append(((x1,y1),(x2,y2)))
      statusdict[((x1,y1),(x2,y2))] = statuses
   elif cmd == 'candidate_path':
      print(cmd)
      path = []
      spoints = args.split()
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
   else:
      raise RuntimeError('unknown cmd: {}'.format(cmd))
   
   s = cairo.ImageSurface(cairo.FORMAT_ARGB32, 500, 500)
   c = cairo.Context(s)
   c.rectangle(0,0,500,500)
   c.set_source_rgb(255., 255., 255.)
   c.fill()
   
   c.arc(250.0, 250.0, 0.3*500.0, 0.0, 2.0*math.pi)
   c.set_source_rgb(255., 0., 0.)
   c.fill()
   
   c.rectangle(50,200,400,100)
   c.set_source_rgb(0., 255., 0.)
   c.fill()
   
   c.rectangle(200,50,100,400)
   c.set_source_rgb(0., 0., 255.)
   c.fill()
   
   if path is not None:
      c.set_source_rgb(0., 255., 0.)
      c.set_line_width(10.0)
      for i,(x,y) in enumerate(path):
         if i == 0:
            c.move_to(500.0*x, s.get_height()-500.0*y)
         else:
            c.line_to(500.0*x, s.get_height()-500.0*y)
      c.stroke()
   
   for ((x1,y1),(x2,y2)) in edges:
      key1 = ((x1,y1),(x2,y2))
      key2 = ((x2,y2),(x1,y1))
      if key1 in statusdict:
         statuses = statusdict[key1]
      else:
         statuses = statusdict[key2]
      # draw each line segment
      for i in range(4):
         xa = x1 + (1.0*i/4)*(x2-x1)
         ya = y1 + (1.0*i/4)*(y2-y1)
         xb = x1 + (1.0*(i+1)/4)*(x2-x1)
         yb = y1 + (1.0*(i+1)/4)*(y2-y1)
         color = list(colors[i])
         if statuses.split()[i] == 'U':
            c.set_line_width(2.0)
            color = (0.7,0.7,0.7) # grey
         elif statuses.split()[i] == 'V':
            c.set_line_width(3.0)
            pass # nice dark color
         else: # assumed invalid
            c.set_line_width(1.0)
            color = (0.0,0.0,0.0) # black
         c.set_source_rgb(*color)
         c.move_to(500.0*xa, s.get_height()-500.0*ya)
         c.line_to(500.0*xb, s.get_height()-500.0*yb)
         c.stroke()
   
   for (x,y) in vertices:
      statuses = statusdict[(x,y)]
      # draw each circle quadrant
      for i in range(4):
         color = list(colors[i])
         if statuses.split()[i] == 'U':
            radius = 3.0
            color = (0.7,0.7,0.7) # grey
         elif statuses.split()[i] == 'V':
            radius = 5.0
            pass # nice dark color
         else: # assumed invalid
            radius = 5.0
            color = (0.0,0.0,0.0) # black
         c.set_source_rgb(*color)
         c.arc(500.0*x, s.get_height()-500.0*y, radius, (0.5*i)*math.pi, (0.5*(i+1))*math.pi)
         c.line_to(500.0*x, s.get_height()-500.0*y)
         c.fill()
      

   
   s.write_to_png('dump/frame-{:05d}.png'.format(num))
   
   
