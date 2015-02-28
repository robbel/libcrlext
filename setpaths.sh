#!/bin/sh

#
# source with ". ./setpaths.sh" or "source ./setpaths.sh"
# Note that libcrlext also needs rl-glue core, strxml, diastream (jasmuth) and
# gzstream installed.
#
export CPATH=$CPATH:/home/philipp/Builds/maastricht/3rdParty/csolvers/libcrlext/libcrl/include:/home/philipp/Builds/maastricht/3rdParty/csolvers/libcrlext/rlgnm/include:/home/philipp/Builds/maastricht/3rdParty/csolvers/libcrlext/glue-crl/include
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/philipp/Builds/maastricht/3rdParty/csolvers/libcrlext/libcrl/lib:/home/philipp/Builds/maastricht/3rdParty/csolvers/libcrlext/rlgnm/lib:/home/philipp/Builds/maastricht/3rdParty/csolvers/libcrlext/glue-crl/lib:/usr/local/lib
