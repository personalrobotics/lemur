#!/usr/bin/env python2
from __future__ import print_function, unicode_literals, absolute_import, division
import argparse
import binascii
import os
import struct

parser = argparse.ArgumentParser()
parser.add_argument('--roadmap-file',required=True)
parser.add_argument('--output-tex',required=True)
args = parser.parse_args()

fp = open(args.roadmap_file,'r')

subgraphs = {} # i -> (nv,ne)
vertices = {} # i -> (x,y)
edges = {} # i -> (va,vb)

for line in fp:
   fields = line.strip().split()
   if fields[0] == 'subgraph' and fields[2] == 'nv' and fields[4] == 'ne':
      subgraphs[int(fields[1])] = (int(fields[3]),int(fields[5]))
   if fields[0] == 'vertex':
      binary = binascii.unhexlify(fields[2])
      point = struct.unpack(b'd'*(len(binary)//8), binary)
      vertices[int(fields[1])] = point
   if fields[0] == 'edge' and fields[2] == 'va' and fields[4] == 'vb':
      edges[int(fields[1])] = (int(fields[3]),int(fields[5]))

fp.close()

fp = open(args.output_tex, 'w')

fp.write(br'''
\documentclass{standalone}
\usepackage{tikz}
%\usetikzlibrary{positioning,shapes}
\begin{document}
\begin{tikzpicture}

\draw[black!10] (0,0) rectangle (1,1);
''')

# start from the BOTTOM
for i in range(len(subgraphs)-1,-1,-1):
   v_to, e_to = subgraphs[i]
   v_from, e_from = 0,0
   if i > 0:
      v_from, e_from = subgraphs[i-1]
   color = 100 - (100.0/len(subgraphs))*i
   # edges
   for e in range(e_from,e_to):
      v1,v2 = edges[e]
      x1,y1 = vertices[v1]
      x2,y2 = vertices[v2]
      fp.write(
         b'\\draw[black!{},line width=0.001cm] ({},{}) -- ({},{});\n'.format(
         color,x1,y1,x2,y2))
   # vertices
   for v in range(v_from,v_to):
      x,y = vertices[v]
      fp.write(
         b'\\fill[black!{}] ({},{}) circle (0.005cm);\n'.format(
         color,x,y))


fp.write(br'''
\end{tikzpicture}
\end{document}
''')


fp.close()


