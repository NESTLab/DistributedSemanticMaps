#!/usr/bin/env bash

SMSOURCEPATH=$HOME/SwarmMeshLibrary
SMBUNDLEPATH=$HOME/swarmmeshbundle

# Get latest SwarmMesh sources
if [[ ! -e $SMSOURCEPATH ]]; then
    git clone https://github.com/NESTLab/SwarmMeshLibrary.git
fi

cd $SMSOURCEPATH
git stash save
git pull
git stash pop

# Compile SwarmMesh
mkdir -p $SMSOURCEPATH/src/swarmmesh/build
cd $SMSOURCEPATH/src/swarmmesh/build
cmake -DCMAKE_INSTALL_PREFIX=$SMBUNDLEPATH -DCMAKE_CXX_FLAGS=-std=c++17 ..
make

# Install SwarmMesh in bundle
mkdir -p $SMBUNDLEPATH
make install
