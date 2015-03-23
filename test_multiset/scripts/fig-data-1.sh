#!/bin/bash

#rm -f fig-data.txt

for lambda in 0.2500 0.7500
do
   for seed in 1 2 3 4 5 6 7 8 9 10
   do
      for run in 1 2 3 4 5
      do
         rosrun test_multiset test-herb-compare.py \
            --planner=MultiSetPRM --lambda=${lambda} \
            --seed=${seed} --stop-after=1 \
            --outfile=fig-data.txt --outfile-lineprefix="MultiSetPRM-lambda${lambda}-seed${seed}-run${run}-"
      done
   done
done

#for range in 0.0499999 0.0999999 0.1499999 0.1999999 0.2499999 0.2999999
#for range in 0.3999999 0.4999999 0.7499999 0.9999999
#for range in 1.4999999 1.9999999 3.9999999 9.9999999
#do
#   for seed in 1 2 3 4 5 6 7 8 9 10
#   do
#      for run in 1 2 3 4 5
#      do
#         rosrun test_multiset test-herb-compare.py \
#            --planner=OMPL_RRTConnect --range=${range} \
#            --seed=${seed} --stop-after=1 \
#            --outfile=fig-data.txt \
#            --outfile-lineprefix="OMPL_RRTConnect-range${range}-seed${seed}-run${run}-"
#      done
#   done
#done

