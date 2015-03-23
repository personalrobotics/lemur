#!/usr/bin/env python2
from __future__ import print_function, unicode_literals, absolute_import, division
import subprocess

fp = open('fig-data.txt', 'r')

# from id -> [vals]
times = {}
lens = {}

for line in fp:
   vals = line.strip().split()
   name = vals.pop(0)
   key,datatype = name.rsplit('-',1)
   if datatype == 'times':
      datadict = times
   elif datatype == 'lens':
      datadict = lens
   else:
      raise RuntimeError('unknown datatype {}!'.format(datatype))
   datadict[key] = map(float,vals)

fp.close()

plots = []
datas = []

# plot rrt stuff
for seed in [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]:
   # make a line per seed for now
   seed_points = []
   for range_ in [0.05, 0.10, 0.15, 0.20, 0.25, 0.30]:
      # average across runs
      sum_time = 0.0
      sum_len = 0.0
      runs = [1, 2, 3, 4, 5]
      for run in runs:
         key = 'OMPL_RRTConnect-range{:0.7f}-seed{}-run{}'.format(
            range_ - 0.0000001, seed, run)
         sum_time += times[key][0]
         sum_len += lens[key][0]
      sum_time /= len(runs)
      sum_len /= len(runs)
      seed_points.append((sum_time, sum_len))
   #plots.append('"-" with lines')
   #datas.append('\n'.join([' '.join(map(str,sp)) for sp in seed_points]))

range_averages = []
for range_ in [0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.40, 0.50, 0.75, 1.0, 1.5, 2.0, 4.0, 10.0]:
   seed_points = []
   for seed in [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]:
      # collect all run-average times for each seed
      sum_time = 0.0
      sum_len = 0.0
      runs = [1, 2, 3, 4, 5]
      for run in runs:
         key = 'OMPL_RRTConnect-range{:0.7f}-seed{}-run{}'.format(
            range_ - 0.0000001, seed, run)
         sum_time += times[key][0]
         sum_len += lens[key][0]
      sum_time /= len(runs)
      sum_len /= len(runs)
      seed_points.append((sum_time, sum_len))
   # compute average point for this range
   range_avg_time = 0.0
   range_avg_len = 0.0
   for seed_time,seed_len in seed_points:
      range_avg_time += seed_time
      range_avg_len += seed_len
   range_avg_time /= len(seed_points)
   range_avg_len /= len(seed_points)
   range_averages.append((range_avg_time,range_avg_len))
   plots.append('"-" notitle with lines lc rgb "#cccccc"')
   data = ''
   for seed_time,seed_len in seed_points:
      data += '{} {}\n{} {}\n\n'.format(
         range_avg_time, range_avg_len,
         seed_time, seed_len)
   datas.append(data)
plots.append('"-" notitle with lines lc rgb "#000000"')
data = ''
for range_avg_time,range_avg_len in range_averages:
   data += '{} {}\n'.format(range_avg_time, range_avg_len)
datas.append(data)

command = 'set title "RRTConnect (across ranges)"\n'
command += 'plot {}\n'.format(', '.join(plots))
for data in datas:
   command += data
   command += 'e\n'

p = subprocess.Popen(['gnuplot','-persist'],stdin=subprocess.PIPE)
p.communicate(command)





# do the multiset prm stats

plots = []
datas = []

range_averages = []
for lambda_ in [0.0001, 0.2500, 0.5000, 0.7500, 0.9999]:
   seed_points = []
   for seed in [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]:
      # collect all run-average times for each seed
      sum_time = 0.0
      sum_len = 0.0
      runs = [1, 2, 3, 4, 5]
      for run in runs:
         key = 'MultiSetPRM-lambda{:0.4f}-seed{}-run{}'.format(
            lambda_, seed, run)
         sum_time += times[key][0]
         sum_len += lens[key][0]
      sum_time /= len(runs)
      sum_len /= len(runs)
      seed_points.append((sum_time, sum_len))
   # compute average point for this range
   range_avg_time = 0.0
   range_avg_len = 0.0
   for seed_time,seed_len in seed_points:
      range_avg_time += seed_time
      range_avg_len += seed_len
   range_avg_time /= len(seed_points)
   range_avg_len /= len(seed_points)
   range_averages.append((range_avg_time,range_avg_len))
   plots.append('"-" notitle with lines lc rgb "#cccccc"')
   data = ''
   for seed_time,seed_len in seed_points:
      data += '{} {}\n{} {}\n\n'.format(
         range_avg_time, range_avg_len,
         seed_time, seed_len)
   datas.append(data)
plots.append('"-" notitle with lines lc rgb "#000000"')
data = ''
for range_avg_time,range_avg_len in range_averages:
   data += '{} {}\n'.format(range_avg_time, range_avg_len)
datas.append(data)

command = 'set title "RRTConnect (across ranges)"\n'
command += 'plot {}\n'.format(', '.join(plots))
for data in datas:
   command += data
   command += 'e\n'

p = subprocess.Popen(['gnuplot','-persist'],stdin=subprocess.PIPE)
p.communicate(command)

