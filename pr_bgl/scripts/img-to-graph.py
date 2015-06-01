#!/usr/bin/env python3
import argparse
import math
import subprocess

# image i,j starts at the TOP LEFT
# cairo origin is also top left

def get_rgbdata(filename):
   w,h = map(int,subprocess.check_output(['convert',filename,'-format','%w %h','info:']).decode().split())
   rbgdata = bytearray(subprocess.check_output(['convert',filename,'rgb:-']))
   if w*h*3 != len(rbgdata):
      raise RuntimeError('image size mismatch!')
   return (w,h),rbgdata

def line_has_dark_blue(w,rbgdata,i1,j1,i2,j2):
   dd = max(abs(i2-i1),abs(j2-j1))
   for d in range(dd+1):
      i = i1 + round((i2-i1)*(d/dd))
      j = j1 + round((j2-j1)*(d/dd))
      px = tuple(rbgdata[3*(i*w+j):3*(i*w+j+1)])
      # we're good if any channel is colored
      # (and bad if everything's black)
      for pxval in px:
         if 128 < pxval:
            break
      else:
         return True
   return False

parser = argparse.ArgumentParser(description='Make a lattice graph over an image.')
parser.add_argument('--img', required=True)
parser.add_argument('--graph-txt', required=True)
args = parser.parse_args()

(w,h),rbgdata = get_rgbdata(args.img)

RES = 40

# create a sparse square grid graph every 20 pixels, offset by 10 pixels
nh = (h-RES//2) // RES + 1;
nw = (w-RES//2) // RES + 1;
oh = (h - (nh-1)*RES)//2;
ow = (w - (nw-1)*RES)//2;

# all vertices (indexed by idx)
vertices = []
class Vertex:
   def __init__(self, idx, i, j):
      self.idx = idx
      self.i = i
      self.j = j

# all edges (indexed by idx)
edges = []
class Edge:
   def __init__(self, idx, v1, v2, w_x, w_xhat, w_phat):
      self.idx = idx
      self.v1 = v1
      self.v2 = v2
      self.w_x = w_x
      self.w_xhat = w_xhat
      self.w_phat = w_phat
      
# map from gridij to vertex
gridij_to_v = {}
for gridi in range(nh):
   for gridj in range(nw):
      i = oh+gridi*RES
      j = ow+gridj*RES
      v = Vertex(len(vertices),i,j)
      vertices.append(v)
      gridij_to_v[(gridi,gridj)] = v

# add all grid edges with weights
# horizontal
for gridi in range(nh):
   for gridj in range(nw-1):
      v1 = gridij_to_v[(gridi,gridj)]
      v2 = gridij_to_v[(gridi,gridj+1)]
      i1 = vertices[v1.idx].i
      j1 = vertices[v1.idx].j
      i2 = vertices[v2.idx].i
      j2 = vertices[v2.idx].j
      w_phat = RES
      w_xhat = RES
      collides = line_has_dark_blue(w,rbgdata,i1,j1,i2,j2)
      if collides:
         w_x = float('inf')
      else:
         w_x = RES
      edges.append(Edge(len(edges),v1,v2,w_x,w_xhat,w_phat))
      edges.append(Edge(len(edges),v2,v1,w_x,w_xhat,w_phat))
# vertical
for gridj in range(nw):
   for gridi in range(nh-1):
      v1 = gridij_to_v[(gridi,gridj)]
      v2 = gridij_to_v[(gridi+1,gridj)]
      i1 = vertices[v1.idx].i
      j1 = vertices[v1.idx].j
      i2 = vertices[v2.idx].i
      j2 = vertices[v2.idx].j
      w_phat = RES
      w_xhat = RES
      collides = line_has_dark_blue(w,rbgdata,i1,j1,i2,j2)
      if collides:
         w_x = float('inf')
      else:
         w_x = RES
      edges.append(Edge(len(edges),v1,v2,w_x,w_xhat,w_phat))
      edges.append(Edge(len(edges),v2,v1,w_x,w_xhat,w_phat))

# find start/goal vertices
v_start = None
v_goal = None
for i in range(h):
   for j in range(w):
      px = tuple(rbgdata[3*(i*w+j):3*(i*w+j+1)])
      if px == (0,255,0):
         if v_start is not None:
            raise RuntimeError('found multiple starts!')
         dist_closest = 9999.0
         v_closest = None
         for v in vertices:
            dist = math.sqrt((i-v.i)**2+(j-v.j)**2)
            if dist_closest <= dist:
               continue
            dist_closest = dist
            v_closest = v
         if dist_closest == 0.0:
            v_start = v_closest
         else:
            v_start = Vertex(len(vertices),i,j)
            vertices.append(v_start)
            i1 = vertices[v_start.idx].i
            j1 = vertices[v_start.idx].j
            i2 = vertices[v_closest.idx].i
            j2 = vertices[v_closest.idx].j
            w_phat = dist_closest
            w_xhat = dist_closest
            collides = line_has_dark_blue(w,rbgdata,i1,j1,i2,j2)
            if collides:
               w_x = float('inf')
            else:
               w_x = dist_closest
            edges.append(Edge(len(edges),v_start,v_closest, w_x,w_xhat,w_phat))
            edges.append(Edge(len(edges),v_closest,v_start, w_x,w_xhat,w_phat))
      if px == (255,0,0):
         if v_goal is not None:
            raise RuntimeError('found multiple goals!')
         dist_closest = 9999.0
         v_closest = None
         for v in vertices:
            dist = math.sqrt((i-v.i)**2+(j-v.j)**2)
            if dist_closest <= dist:
               continue
            dist_closest = dist
            v_closest = v
         if dist_closest == 0.0:
            v_goal = v_closest
         else:
            v_goal = Vertex(len(vertices),i,j)
            vertices.append(v_goal)
            i1 = vertices[v_goal.idx].i
            j1 = vertices[v_goal.idx].j
            i2 = vertices[v_closest.idx].i
            j2 = vertices[v_closest.idx].j
            w_phat = dist_closest
            w_xhat = dist_closest
            collides = line_has_dark_blue(w,rbgdata,i1,j1,i2,j2)
            if collides:
               w_x = float('inf')
            else:
               w_x = dist_closest
            edges.append(Edge(len(edges),v_goal,v_closest, w_x,w_xhat,w_phat))
            edges.append(Edge(len(edges),v_closest,v_goal, w_x,w_xhat,w_phat))

if v_start is None or v_goal is None:
   raise RuntimeError('couldnt find a start or goal state!')

fp = open(args.graph_txt,'w')
for vidx,v in enumerate(vertices):
   fp.write('vertex {} at {},{}\n'.format(vidx,v.i,v.j))
for eidx,e in enumerate(edges):
   fp.write('edge {} source {} target {} w_x {} w_xhat {} w_phat {}\n'.format(
      eidx, e.v1.idx, e.v2.idx, e.w_x, e.w_xhat, e.w_phat))
fp.write('vertex_start {}\n'.format(v_start.idx))
fp.write('vertex_goal {}\n'.format(v_goal.idx))


