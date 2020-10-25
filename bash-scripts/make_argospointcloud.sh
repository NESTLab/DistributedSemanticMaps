#!/usr/bin/env bash

ARGOSSOURCEPATH=$HOME/DistributedSemanticMaps/argos-point-cloud/
ARGOSBUNDLEPATH=$HOME/argos3bundle


cd $ARGOSSOURCEPATH
#git stash save
#git pull
#git stash pop

# Compile ARGoS
mkdir -p $ARGOSSOURCEPATH/build
cd $ARGOSSOURCEPATH/build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$ARGOSBUNDLEPATH -DARGOS_DOCUMENTATION=OFF -DARGOS_INSTALL_LDSOCONF=OFF -DARGOS_BUILD_NATIVE=ON ../src
make

# Install ARGoS in bundle
mkdir -p $ARGOSBUNDLEPATH
make install          
