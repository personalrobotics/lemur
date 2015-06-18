#!/usr/bin/python3 -E
import argparse
import math
import glob
import os
import re
import subprocess
import tempfile

import cairo

parser = argparse.ArgumentParser(description='Make a lattice graph over an image.')
parser.add_argument('--img', required=True)
parser.add_argument('--graph-txt', required=True)
parser.add_argument('--dump-glob', required=True)
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

edges_evaled = set()

with tempfile.TemporaryDirectory() as dirname:

   frame = 0
   for fn in sorted(glob.glob(args.dump_glob)):
      
      print(fn)
      
      # read data file
      # get queue states vidx -> 'open' or 'closed'
      v_start = {}
      v_goal = {}
      v_conn = {}
      incbi_values = {}
      path = None
      edge_to_eval = None
      fp = open(fn,'r')
      for line in fp:
         # simple vertex tags
         m = re.match('([a-z_]*) ([0-9]*)$', line)
         if m:
            tag = m.group(1)
            idx = int(m.group(2))
            if tag == 'start_open':
               v_start[idx] = 'open'
               continue
            if tag == 'start_closed':
               v_start[idx] = 'closed'
               continue
            if tag == 'goal_open':
               v_goal[idx] = 'open'
               continue
            if tag == 'goal_closed':
               v_goal[idx] = 'closed'
               continue
            if tag == 'conn':
               v_conn[idx] = 'open'
               continue
            if tag == 'edge_evaled':
               edges_evaled.add(idx)
               continue
            raise RuntimeError('unknown line: {}'.format(line))
         if line.startswith('candidate_path'):
            path = map(int,line.strip().split()[1:])
            continue
         if line.startswith('edge_to_eval'):
            edge_to_eval = tuple(map(int,line.strip().split()[1:]))
            continue
         if line.startswith('incbi_values'):
            _, a, b, c, d, e = line.strip().split()
            incbi_values[int(a)] = tuple(map(float,[b,c,d,e]))
            continue
         raise RuntimeError('unknown line: {}'.format(line))
      fp.close()
      
      num_frames = 1

      s = cairo.ImageSurface.create_from_png(open(args.img,'rb'))
      c = cairo.Context(s)

      # draw edge_to_eval if there is one
      if edge_to_eval is not None:
         i1,j1 = vertices[edge_to_eval[0]]
         i2,j2 = vertices[edge_to_eval[1]]
         c.set_source_rgb(0.0,0.0,1.0)
         c.set_line_width(8.0)
         c.move_to(j1, i1)
         c.line_to(j2, i2)
         c.stroke()
         num_frames = 30

      # draw path if there is one
      if path is not None:
         for i,vidx in enumerate(path):
            c.set_source_rgb(0.0,0.0,1.0)
            c.set_line_width(4.0)
            if i == 0:
               fn = c.move_to
            else:
               fn = c.line_to
            i,j = vertices[vidx]
            fn(j, i)
         c.stroke()
         num_frames = 30

      # draw edges (not evaled)
      for eidx,(v1,v2) in edges.items():
         if eidx in edges_evaled:
            continue
         i1,j1 = vertices[v1]
         i2,j2 = vertices[v2]
         c.set_source_rgb(0.7,0.7,0.7)
         c.set_line_width(0.5)
         c.move_to(j1, i1)
         c.line_to(j2, i2)
         c.stroke()

      # draw edges (evaled)
      for eidx,(v1,v2) in edges.items():
         if eidx not in edges_evaled:
            continue
         i1,j1 = vertices[v1]
         i2,j2 = vertices[v2]
         if edge_xs[eidx] > 999.9:
            c.set_source_rgb(1., 0., 0.)
         else:
            c.set_source_rgb(0., 0., 0.)
         c.set_line_width(10.0)
         c.move_to(j1, i1)
         c.line_to(j2, i2)
         c.stroke()

      # draw vertices
      for i,j in vertices.values():
         c.set_source_rgb(0.7,0.7,0.7)
         c.arc(j, i, 2.0, 0, 2*math.pi)
         c.fill()

      # draw start/goal open/closed list states
      for vidx,(i,j) in vertices.items():
         if vidx in v_conn:
            c.set_source_rgb(0.0,0.0,1.0)
            c.arc(j, i, 8.0, 0, 2*math.pi)
            c.fill()
         if vidx in v_start:
            if v_start[vidx] == 'open':
               c.set_source_rgb(0.0,1.0,0.0)
            else:
               c.set_source_rgb(0.6,0.8,0.6)
            c.arc(j, i, 5.0, 0.5*math.pi, 1.5*math.pi)
            c.fill()
         if vidx in v_goal:
            if v_goal[vidx] == 'open':
               c.set_source_rgb(1.0,0.0,0.0)
            else:
               c.set_source_rgb(0.8,0.6,0.6)
            c.arc(j, i, 5.0, -0.5*math.pi, 0.5*math.pi)
            c.fill()
      
      # show incbi values
      c.set_font_size(8)
      for vidx,(i,j) in vertices.items():
         if vidx not in incbi_values:
            continue
         ds,rs,dg,rg = incbi_values[vidx]
         # ds
         text = '{:.0f}'.format(ds)
         _, _, tw, th, _, _ = c.text_extents(text)
         c.set_source_rgb(1., 1., 1.)
         c.move_to(j-tw-1-1, i+1+1)
         c.show_text(text)
         c.set_source_rgb(0., 0., 0.)
         c.move_to(j-tw-1, i+1)
         c.show_text(text)
         # rs
         text = '{:.0f}'.format(rs)
         _, _, tw, th, _, _ = c.text_extents(text)
         c.set_source_rgb(1., 1., 1.)
         c.move_to(j-tw-1-1, i-th-1+1)
         c.show_text(text)
         c.set_source_rgb(0., 0., 0.)
         c.move_to(j-tw-1, i-th-1)
         c.show_text(text)
         # dg
         text = '{:.0f}'.format(dg)
         _, _, tw, th, _, _ = c.text_extents(text)
         c.set_source_rgb(1., 1., 1.)
         c.move_to(j+1-1, i+1+1)
         c.show_text(text)
         c.set_source_rgb(0., 0., 0.)
         c.move_to(j+1, i+1)
         c.show_text(text)
         # rg
         text = '{:.0f}'.format(rg)
         _, _, tw, th, _, _ = c.text_extents(text)
         c.set_source_rgb(1., 1., 1.)
         c.move_to(j+1-1, i-th-1+1)
         c.show_text(text)
         c.set_source_rgb(0., 0., 0.)
         c.move_to(j+1, i-th-1)
         c.show_text(text)

      for _ in range(num_frames):
         s.write_to_png('{}/frame-{:05d}.png'.format(dirname,frame))
         frame += 1

   subprocess.check_call(['avconv','-f','image2','-framerate','30',
      '-i',os.path.join(dirname,'frame-%05d.png'),'-r','30',
      '-f','mp4','-c:v','libx264','-b','16384k','-bf','1','-y',args.out_mp4])
