#!/usr/bin/python3 -E
# allows python3 under a ros environment

# File: graph-to-image.py
# Author: Chris Dellin <cdellin@gmail.com>
# Copyright: 2015 Carnegie Mellon University
# License: BSD

import argparse
import collections
import math
import re
import cairo

#import util

# cairo coords, origin top left, x RIGHT y DOWN

parser = argparse.ArgumentParser(
   description='create image of a 2d roadmap graph')
parser.add_argument('--graph', required=True)
parser.add_argument('--image', required=True)
args = parser.parse_args()

# read graph io format from pr_bgl
# with general properties
def graph_from_file(fname):
   vnum = 0
   edges = {} # eidx -> (vidx1, vidx2)
   vprops = {} # vidx -> {prop -> string-val}
   eprops = {} # eidx -> {prop -> string-val}
   fp = open(fname,'r')
   for line in fp:
      m = re.match('vertex ([0-9]*)$',line)
      if m:
         vidx, = m.groups()
         vidx = int(vidx)
         if vnum < vidx + 1:
            vnum = vidx + 1
         if vidx not in vprops:
            vprops[vidx] = {}
         continue
      m = re.match('edge ([0-9]*) source ([0-9]*) target ([0-9]*)$',line)
      if m:
         eidx,vidx1,vidx2 = m.groups()
         eidx = int(eidx)
         vidx1 = int(vidx1)
         vidx2 = int(vidx2)
         edges[eidx] = (vidx1,vidx2)
         if eidx not in eprops:
            eprops[eidx] = {}
         continue
      m = re.match('vprop ([0-9]*) ([^ ]*) (.*)$',line)
      if m:
         vidx,propname,propval = m.groups()
         vidx = int(vidx)
         if vidx not in vprops:
            vprops[vidx] = {}
         vprops[vidx][propname] = propval
         continue
      m = re.match('eprop ([0-9]*) ([^ ]*) (.*)$',line)
      if m:
         eidx,propname,propval = m.groups()
         eidx = int(eidx)
         if eidx not in eprops:
            eprops[eidx] = {}
         eprops[eidx][propname] = propval
         continue
   fp.close()
   Ret = collections.namedtuple('Graph', 'vnum edges vprops eprops')
   return Ret(vnum, edges, vprops, eprops)

graph = graph_from_file(args.graph)

px_size = 200
s = cairo.ImageSurface(cairo.FORMAT_RGB24, px_size, px_size)
c = cairo.Context(s)
c.scale(px_size, px_size)

# background is white
c.rectangle(0., 0., 1., 1.)
c.set_source_rgb(1., 1., 1.)
c.fill()

c.set_source_rgb(0.5, 0.5, 0.5)
c.set_line_width(0.005)
c.move_to(0., 0.)
c.line_to(0., 1.)
c.line_to(1., 1.)
c.line_to(1., 0.)
c.line_to(0., 0.)
c.stroke()

# draw graph lines
numbatches = 0
for eidx in graph.edges:
   ibatch = int(graph.eprops[eidx]['batch'])
   if numbatches < ibatch+1:
      numbatches = ibatch+1

c.set_line_width(0.005)
level_colors = [
   (0.0, 0.0, 0.0),
   (0.3, 0.3, 0.3),
   (0.6, 0.6, 0.6),
]
for ibatch in reversed(range(numbatches)):
   color = level_colors[ibatch]
   c.set_source_rgb(*color)
   for eidx,(vidx1,vidx2) in graph.edges.items():
      if int(graph.eprops[eidx]['batch']) != ibatch:
         continue
      # compute actual states in floats
      x1,y1 = map(float,graph.vprops[vidx1]['state'].split())
      x2,y2 = map(float,graph.vprops[vidx2]['state'].split())
      c.move_to(x1,y1)
      c.line_to(x2,y2)
      c.stroke()

s.write_to_png(args.image)
