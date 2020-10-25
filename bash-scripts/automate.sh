#!/usr/bin/env bash
#SBATCH -J argosPerception
#SBATCH -n 1
#SBATCH -N 1
#SBATCH -p short
#SBATCH --mem 32G
# #SBATCH -C E5-2680

# Stop execution after any error
set -e

NB_ROBOTS=${1}
MIN_VOTES=${2}
JOB_LOC=run${3}

# Useful variables
#JOB_LOC=run${1}
BASE_LOC=$PWD
DATADIR=$BASE_LOC/../output_data #where you want your data to be stored
COUNT=0

SEED=1
STORAGE=10
ROUTING=5
HASHING=5

WORKDIR=$DATADIR/$JOB_LOC
CONFDIR=$WORKDIR/experiments/

mkdir -p $CONFDIR
cd $WORKDIR

echo $NB_ROBOTS $MIN_VOTES $JOB_LOC

sed -e "s|SEED|${SEED}|;s|BASELOC|${BASE_LOC}|g;s|MINVOTES|${MIN_VOTES}|;s|NB_ROBOTS|${NB_ROBOTS}|g;s|STORAGE|${STORAGE}|;s|ROUTING|${ROUTING}|;s|HASHING|${HASHING}|" $BASE_LOC/experiments/collective_perception_mixed_environment_cluster.argos > $CONFDIR/exp_${NB_ROBOTS}_${MIN_VOTES}_${SEED}_${STORAGE}_${ROUTING}_${HASHING}.argos
cd $BASE_LOC
# # Execute program (this also writes files in work dir)
argos3 -l /dev/null -c $CONFDIR/exp_${NB_ROBOTS}_${MIN_VOTES}_${SEED}_${STORAGE}_${ROUTING}_${HASHING}.argos 2> "log_${NB_ROBOTS}_${MIN_VOTES}_${SEED}_${STORAGE}_${ROUTING}_${HASHING}.txt"

echo $(( COUNT++ ))
echo "If you see this, experiment $COUNT/12 ${NB_ROBOTS}_${MIN_VOTES}_${SEED}_${STORAGE}_${ROUTING}_${HASHING} is done"

mv *.dat $WORKDIR
echo "Copying files"
#rm -rf $CONFDIR
