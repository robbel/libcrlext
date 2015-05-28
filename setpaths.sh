#!/bin/sh

#
# source with ". ./setpaths.sh" or "source ./setpaths.sh"
# Note that libcrlext also needs rl-glue core, strxml, diastream (jasmuth) and
# gzstream installed.
#
export CPATH=$CPATH:/home/philipp/Builds/maastricht/3rdParty/libcrlext/libcrl/include:/home/philipp/Builds/maastricht/3rdParty/libcrlext/rlgnm/include:/home/philipp/Builds/maastricht/3rdParty/libcrlext/glue-crl/include:/home/philipp/Builds/maastricht/3rdParty/libdai/include:/home/philipp/Builds/maastricht/3rdParty/libcrlext/glue-environments/include:/home/philipp/Builds/maastricht/3rdParty/libcrlext/graphprop/include:/home/philipp/Builds/rddlsimSPUDD/SPUDD/spudd/Spudd/src/MDP:/home/philipp/Builds/rddlsimSPUDD/SPUDD/spudd/CuddPP/include:/opt/gurobi603/linux64/include
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/philipp/Builds/maastricht/3rdParty/libcrlext/libcrl/lib:/home/philipp/Builds/maastricht/3rdParty/libcrlext/rlgnm/lib:/home/philipp/Builds/maastricht/3rdParty/libcrlext/glue-crl/lib:/usr/local/lib:/home/philipp/Builds/maastricht/3rdParty/libdai/lib:/home/philipp/Builds/maastricht/3rdParty/libcrlext/glue-environments/lib:/home/philipp/Builds/maastricht/3rdParty/libcrlext/graphprop/lib:/home/philipp/Builds/rddlsimSPUDD/SPUDD/spudd/CuddPP/lib/linux:/home/philipp/Builds/rddlsimSPUDD/SPUDD/spudd/Spudd/lib/linux
export PATH=$PATH:/home/philipp/Builds/maastricht/3rdParty/libcrlext/glue-crl/bin:/home/philipp/Builds/maastricht/3rdParty/libcrlext/glue-environments/bin:/home/philipp/Builds/maastricht/3rdParty/libcrlext/graphprop/bin:/home/philipp/Builds/maastricht/3rdParty/libcrlext/mazes/bin:/home/philipp/Builds/maastricht/3rdParty/libcrlext/experimentor/bin:/home/philipp/Builds/maastricht/3rdParty/libcrlext/glue-agents/bin:/home/philipp/Builds/maastricht/3rdParty/libcrlext/cluster-boss/bin
export PYTHONPATH=$PYTHONPATH:.:/home/philipp/Builds/maastricht/3rdParty/libcrlext/experimentor/lib
