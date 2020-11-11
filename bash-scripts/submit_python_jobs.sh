#!/usr/bin/env bash
#SBATCH -J vscbpp
#SBATCH -n 1
#SBATCH -N 1
#SBATCH -p short
#SBATCH --mem 32G

# Stop execution after any error
set -e

python vscbpp_cluster.py
