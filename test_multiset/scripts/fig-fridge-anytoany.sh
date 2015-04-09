#!/bin/bash

#rm -f fig-data.txt

#for lambda in 0.0001 0.2500 0.5000 0.7500 0.9999
#for lambda in 0.1250 0.3750 0.6250 0.8750
for lambda in
do
   for seed in 1 2 3 4 5 6 7 8 9 10
   do
      rosrun test_multiset test-herb-tray.py \
         --planner=MultiSetPRM --lambda=${lambda} \
         --seed=${seed} \
         --w-any-to-any \
         --outfile=fig-fridge-anytoany.txt --outfile-lineprefix="MultiSetPRM-lambda${lambda}-seed${seed}-"
   done
done

#for lambda in 0.0001 0.2500 0.5000 0.7500 0.9999
#for lambda in 0.1250 0.3750 0.6250 0.8750
for lambda in
do
   for seed in 1 2 3 4 5 6 7 8 9 10
   do
      rosrun test_multiset test-herb-tray.py \
         --planner=MultiSetPRM --lambda=${lambda} \
         --seed=${seed} \
         --w-interstep \
         --outfile=fig-fridge-anytoany.txt --outfile-lineprefix="MultiSetPRM-interstep-lambda${lambda}-seed${seed}-"
   done
done

#for lambda in 0.0001 0.2500 0.5000 0.7500 0.9999
#for lambda in 0.1250 0.3750 0.6250 0.8750
for lambda in
do
   for seed in 1 2 3 4 5 6 7 8 9 10
   do
      rosrun test_multiset test-herb-tray.py \
         --planner=MultiSetPRM --lambda=${lambda} \
         --seed=${seed} \
         --w-selfcc \
         --outfile=fig-fridge-anytoany.txt --outfile-lineprefix="MultiSetPRM-selfcc-lambda${lambda}-seed${seed}-"
   done
done


#for range in 0.0499999 0.0999999 0.1499999 0.1999999 0.2499999 0.2999999 0.3999999 0.4999999 0.7499999 0.9999999 1.4999999 1.9999999 3.9999999 9.9999999
#for range in 0.2499999 0.2999999 0.3999999 0.4999999 0.7499999 0.9999999 1.4999999 1.9999999 3.9999999 9.9999999
for range in 2.2
#for range in
do
   for seed in 1 2 3 4 5 6 7 8 9 10
   do
      rosrun test_multiset test-herb-tray.py \
         --planner=OMPL_RRTConnect --range=${range} \
         --seed=${seed} \
         --w-any-to-any \
         --outfile=fig-fridge-anytoany.txt \
         --outfile-lineprefix="OMPL_RRTConnect-range${range}-seed${seed}-"
   done
done

#for range in 0.0499999 0.0999999 0.1499999 0.1999999 0.2499999 0.2999999 0.3999999 0.4999999 0.7499999 0.9999999 1.4999999 1.9999999 3.9999999 9.9999999
#do
#   for seed in 1 2 3 4 5 6 7 8 9 10
#   do
#      for run in 1 2 3 4 5
#      do
#         rosrun test_multiset test-herb-tray.py \
#            --planner=OMPL_LazyPRM --range=${range} \
#            --seed=${seed} --stop-after=1 \
#            --outfile=fig-fridge-anytoany.txt \
#            --outfile-lineprefix="OMPL_LazyPRM-range${range}-seed${seed}-run${run}-"
#      done
#   done
#done

