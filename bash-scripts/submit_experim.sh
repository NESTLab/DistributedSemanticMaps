#!/usr/bin/env bash

# Stop execution after any error
set -e

# Useful variables
RUN=A
BASE_LOC=$PWD

#Go through all the values for seed
for NB_ROBOTS in 30 60 90
do
   for MIN_VOTES in 3 4 5 6
   do
      # Submit job
      sbatch automate.sh $PARAM1 $PARAM2 $RUN
      # Sleep for 1 second to avoid overloading the machine
      sleep 1
   done 
done
