#!/usr/bin/env python2
from __future__ import print_function, unicode_literals, absolute_import, division
import subprocess

fp = open('fig-data.txt', 'r')

# from id -> [vals]
times = {}
lens = {}

for line in fp:
   vals = line.strip().split()
   if not vals:
      continue
   name = vals.pop(0)
   key,datatype = name.rsplit('-',1)
   if datatype == 'times':
      datadict = times
   elif datatype == 'lens':
      datadict = lens
   else:
      raise RuntimeError('unknown datatype {}!'.format(datatype))
   if key in datadict:
      raise RuntimeError('key {} already found!'.format(key))
   datadict[key] = map(float,vals)

fp.close()





# [(i,'planner')] -> [(param_val, avg_time, avg_len)]
avgs = {}

# get multiset averages
for i in range(3):
   avgs[(i,'multiset')] = []
   for lambda_ in [0.0001, 0.2500, 0.5000, 0.7500, 0.9999]:
      seed_points = []
      for seed in [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]:
         key = 'MultiSetPRM-lambda{:0.4f}-seed{}'.format(lambda_, seed)
         seed_points.append((times[key][i], lens[key][i]))
      # compute average point for this lambda
      range_avg_time = 0.0
      range_avg_len = 0.0
      for seed_time,seed_len in seed_points:
         range_avg_time += seed_time
         range_avg_len += seed_len
      range_avg_time /= len(seed_points)
      range_avg_len /= len(seed_points)
      avgs[(i,'multiset')].append((lambda_, range_avg_time, range_avg_len))

# get interstep averages
for i in range(3):
   avgs[(i,'multiset-interstep')] = []
   for lambda_ in [0.0001, 0.2500, 0.5000, 0.7500, 0.9999]:
      seed_points = []
      for seed in [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]:
         key = 'MultiSetPRM-interstep-lambda{:0.4f}-seed{}'.format(lambda_, seed)
         seed_points.append((times[key][i], lens[key][i]))
      # compute average point for this lambda
      range_avg_time = 0.0
      range_avg_len = 0.0
      for seed_time,seed_len in seed_points:
         range_avg_time += seed_time
         range_avg_len += seed_len
      range_avg_time /= len(seed_points)
      range_avg_len /= len(seed_points)
      avgs[(i,'multiset-interstep')].append((lambda_, range_avg_time, range_avg_len))

# get selfcc averages
for i in range(3):
   avgs[(i,'multiset-selfcc')] = []
   for lambda_ in [0.0001, 0.2500, 0.5000, 0.7500, 0.9999]:
      seed_points = []
      for seed in [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]:
         key = 'MultiSetPRM-selfcc-lambda{:0.4f}-seed{}'.format(lambda_, seed)
         seed_points.append((times[key][i], lens[key][i]))
      # compute average point for this lambda
      range_avg_time = 0.0
      range_avg_len = 0.0
      for seed_time,seed_len in seed_points:
         range_avg_time += seed_time
         range_avg_len += seed_len
      range_avg_time /= len(seed_points)
      range_avg_len /= len(seed_points)
      avgs[(i,'multiset-selfcc')].append((lambda_, range_avg_time, range_avg_len))

# get interstep-selfcc averages
for i in range(3):
   avgs[(i,'multiset-interstep-selfcc')] = []
   for lambda_ in [0.0001, 0.2500, 0.5000, 0.7500, 0.9999]:
      seed_points = []
      # HACK HACK HACK!
      #for seed in [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]:
      for seed in [1, 3, 4, 5, 6, 7, 8, 9, 10]:
         key = 'MultiSetPRM-interstep-selfcc-lambda{:0.4f}-seed{}'.format(lambda_, seed)
         seed_points.append((times[key][i], lens[key][i]))
      # compute average point for this lambda
      range_avg_time = 0.0
      range_avg_len = 0.0
      for seed_time,seed_len in seed_points:
         range_avg_time += seed_time
         range_avg_len += seed_len
      range_avg_time /= len(seed_points)
      range_avg_len /= len(seed_points)
      avgs[(i,'multiset-interstep-selfcc')].append((lambda_, range_avg_time, range_avg_len))

# get rrtconnect averages
for i in range(3):
   avgs[(i,'rrtconnect')] = []
   for range_ in [0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.40, 0.50, 0.75, 1.0, 1.5, 2.0, 4.0, 10.0]:
      seed_points = []
      for seed in [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]:
         key = 'OMPL_RRTConnect-range{:0.7f}-seed{}'.format(range_ - 0.0000001, seed)
         seed_points.append((times[key][i], lens[key][i]))
      # compute average point for this lambda
      range_avg_time = 0.0
      range_avg_len = 0.0
      for seed_time,seed_len in seed_points:
         range_avg_time += seed_time
         range_avg_len += seed_len
      range_avg_time /= len(seed_points)
      range_avg_len /= len(seed_points)
      avgs[(i,'rrtconnect')].append((range_, range_avg_time, range_avg_len))


# send out tikz plots
for i in range(3):

   fp = open('plot-{}.tex'.format(i+1),'w')
   fp.write(br'''
   \documentclass{standalone}
   \usepackage{tikz}
   %\usetikzlibrary{positioning,shapes}
   \usepackage{pgfplots}
   \begin{document}
   \begin{tikzpicture}
   ''')

   fp.write(br'''
   \begin{axis}[
      xlabel=Planning Time (s),
      ylabel=Solution Length (rad),
      ylabel near ticks,
      xlabel near ticks,
      xmin=0,xmax=20,
      ymin=5,ymax=25]
   ''')
   
   fp.write(br'''
   \addplot[mark=*,red] plot coordinates {
   ''')
   for lambda_,avg_time,avg_len in avgs[(i,'rrtconnect')]:
      fp.write('  ({},{})\n'.format(avg_time,avg_len))
   fp.write(br'''
   };
   \addlegendentry{RRT Connect}
   ''')
   
   fp.write(br'''
   \addplot[mark=*,blue] plot coordinates {
   ''')
   for lambda_,avg_time,avg_len in avgs[(i,'multiset')]:
      fp.write('  ({},{})\n'.format(avg_time,avg_len))
   fp.write(br'''
   };
   \addlegendentry{Multi-Set PRM}
   ''')
   
   fp.write(br'''
   \addplot[mark=*,green] plot coordinates {
   ''')
   for lambda_,avg_time,avg_len in avgs[(i,'multiset-interstep')]:
      fp.write('  ({},{})\n'.format(avg_time,avg_len))
   fp.write(br'''
   };
   \addlegendentry{w/ Inter-Step}
   ''')
   
   fp.write(br'''
   \addplot[mark=*,yellow] plot coordinates {
   ''')
   for lambda_,avg_time,avg_len in avgs[(i,'multiset-selfcc')]:
      fp.write('  ({},{})\n'.format(avg_time,avg_len))
   fp.write(br'''
   };
   \addlegendentry{w/ SelfCC1}
   ''')
   
   fp.write(br'''
   \addplot[mark=*,black] plot coordinates {
   ''')
   for lambda_,avg_time,avg_len in avgs[(i,'multiset-interstep-selfcc')]:
      fp.write('  ({},{})\n'.format(avg_time,avg_len))
   fp.write(br'''
   };
   \addlegendentry{w/ Both}
   ''')
   
   fp.write(br'''
   \end{axis}
   ''')

   fp.write(br'''
   \end{tikzpicture}
   \end{document}
   ''')
   fp.close()






# for runs below
if False:
   # plot rrt stuff
   plots = []
   datas = []
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




if False:
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

   command = 'set title "MultiSetPRM (across lambas)"\n'
   command += 'plot {}\n'.format(', '.join(plots))
   for data in datas:
      command += data
      command += 'e\n'

   p = subprocess.Popen(['gnuplot','-persist'],stdin=subprocess.PIPE)
   p.communicate(command)

