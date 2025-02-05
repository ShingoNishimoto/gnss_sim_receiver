#! /bin/bash

trap 'stty sane; bladeRF-cli -e "set gain TX1 -22"' INT TERM

###
# get absolute path to conf dir
confpath=$(cd $(dirname $0) && pwd)/../conf
conffile=$confpath/gnss-sdr_GPS_L1_bladeRF2_micro.conf
if [ ! -f $conffile ]; then
    echo;echo not found : $conffile;echo
    exit 1
fi

###
# set parameters manually.
bladerfargs="-t 2024/01/15,01:00:00 -e $confpath/../src/bladeGPS/brdc0150.24n -d 3000 -l 0.0,135.0,400000000.0 -s ./log/ -a 5 -r 0,-90 -p -I -E -v"

date_str="`date +'%Y%m%d%H%M%S'`"
mkdir $date_str

###
# run
gnss_sim_receiver \
    --config_file=$conffile \
    --log_dir=$date_str \
    --bladegps_args="$bladerfargs"
