#!/usr/bin/python3 -E
import argparse
import math
import glob
import os
import re
import subprocess
import tempfile
import sys

import cairo

def linspace(a, b, num):
   vals = []
   for i in range(num):
      vals.append(a+1.0*i*(b-a)/(num-1))
   return vals

# e.g. PYTHONPATH= rosrun pr_bgl sebc-chirp-to-movie.py --img=prob1.png --graph-txt=prob1-graph.txt --out-mp4 sebc.mp4

parser = argparse.ArgumentParser(description='Render a graph using sebc chirp to a movie')
parser.add_argument('--img', required=True)
parser.add_argument('--graph-txt', required=True)
parser.add_argument('--out-mp4', required=True)
args = parser.parse_args()

# get basic graph structure / mapping to physical locations
vertices = {} #idx -> (i,j)
edges = {} # idx -> (v1,v2)
edge_xs = {}
fp = open(args.graph_txt,'r')
for line in fp:
   m = re.match('vertex ([0-9]*) at ([0-9]*),([0-9]*)',line)
   if m:
      vidx,i,j = map(int,m.groups())
      vertices[vidx] = (i,j)
   m = re.match('edge ([0-9]*) source ([0-9]*) target ([0-9]*) w_x ([^ ]*) w_xhat ([^ ]*) w_phat ([^ ]*)',line)
   if m:
      eidx,v1,v2,w_x,w_xhat,w_phat = m.groups()
      eidx = int(eidx)
      v1 = int(v1)
      v2 = int(v2)
      w_x = float(w_x)
      w_xhat = float(w_xhat)
      w_phat = float(w_phat)
      edges[eidx] = (v1,v2)
      edge_xs[eidx] = w_x
fp.close()

# chirp!

with tempfile.TemporaryDirectory() as dirname:

   frame = 0
   for ratio in linspace(1.0,2.25,26):
      print(ratio)
      
      sebc_result = '{}/blah.txt'.format(dirname)
      
      subprocess.check_call(['rosrun','pr_bgl','test_sebc_dump',
         args.graph_txt,str(ratio),sebc_result])
      
      edge_sebc_scores = {} # eidx -> score
      fp = open(sebc_result,'r')
      for line in fp:
         fields = line.strip().split()
         if fields[0] == 'edge_sebc_score':
            edge_sebc_scores[int(fields[1])] = float(fields[2])
      fp.close()
      
      s = cairo.ImageSurface.create_from_png(open(args.img,'rb'))
      c = cairo.Context(s)
      
      # draw edges colored by score
      for eidx,(v1,v2) in edges.items():
         i1,j1 = vertices[v1]
         i2,j2 = vertices[v2]
         color = 1.0 - edge_sebc_scores[eidx]
         c.set_source_rgb(color, color, color)
         c.set_line_width(4.0)
         c.move_to(j1, i1)
         c.line_to(j2, i2)
         c.stroke()
      
      # draw all edges (in grey)
      #for eidx,(v1,v2) in edges.items():
      #   i1,j1 = vertices[v1]
      #   i2,j2 = vertices[v2]
      #   c.set_source_rgb(0.7,0.7,0.7)
      #   c.set_line_width(0.5)
      #   c.move_to(j1, i1)
      #   c.line_to(j2, i2)
      #   c.stroke()
      
      c.set_source_rgb(0., 0., 0.)
      c.move_to(5, 10)
      c.show_text('ratio:{}'.format(ratio))
      
      for _ in range(30):
         s.write_to_png('{}/frame-{:05d}.png'.format(dirname,frame))
         frame += 1

   subprocess.check_call(['avconv','-f','image2','-framerate','30',
      '-i',os.path.join(dirname,'frame-%05d.png'),'-r','30',
      '-f','mp4','-c:v','libx264','-b','16384k','-bf','1','-y',args.out_mp4])

