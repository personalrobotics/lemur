#!/usr/bin/env python3
import math
import sys
import cairo

# cairo origin is in top left (opposite of world x,y)

if sys.version_info.major != 3:
   raise RuntimeError('python 3 only!')

print('dumping ...')

vertices = []
edges = []

for num,line in enumerate(open('dump.txt')):
   line = line.strip()
   
   path = None
   
   cmd,args = line.split(None,1)
   if cmd == 'add_vertex':
      (x,y) = tuple(map(float,args.split(',')))
      vertices.append((x,y))
   elif cmd == 'add_edge':
      v1,v2 = args.split()
      (x1,y1) = tuple(map(float,v1.split(',')))
      (x2,y2) = tuple(map(float,v2.split(',')))
      edges.append(((x1,y1),(x2,y2)))
   else:
      print(cmd)
      path = []
      spoints = args.split()
      for spoint in spoints:
         (x,y) = tuple(map(float,spoint.split(',')))
         path.append((x,y))
   
   s = cairo.ImageSurface(cairo.FORMAT_ARGB32, 500, 500)
   c = cairo.Context(s)
   c.rectangle(0,0,500,500)
   c.set_source_rgb(255., 255., 255.)
   c.fill()
   
   c.arc(250.0, 250.0, 0.3*500.0, 0.0, 2.0*math.pi)
   c.set_source_rgb(255., 0., 0.)
   c.fill()
   
   c.rectangle(50,200,400,100)
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
         
   
   for (x,y) in vertices:
      c.set_source_rgb(0., 0., 0.)
      c.arc(500.0*x, s.get_height()-500.0*y, 2, 0.0, 2.0*math.pi)
      c.fill()
      
   for ((x1,y1),(x2,y2)) in edges:
      c.set_source_rgb(0., 0., 0.)
      c.set_line_width(1.0)
      c.move_to(500.0*x1, s.get_height()-500.0*y1)
      c.line_to(500.0*x2, s.get_height()-500.0*y2)
      c.stroke()
   
   s.write_to_png('dump/frame-{:05d}.png'.format(num))
   
   
