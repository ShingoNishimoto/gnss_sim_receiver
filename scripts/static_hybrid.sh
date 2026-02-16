#! /bin/bash

trap 'stty sane; bladeRF-cli -e "set gain TX1 -22"' INT TERM

###
# get absolute path to conf dir
confpath=$(cd $(dirname $0) && pwd)/../conf
conffile=$confpath/gnss-sdr_GPS_L1_bladeRF2_micro_hybrid_nav.conf
if [ ! -f $conffile ]; then
    echo;echo not found : $conffile;echo
    exit 1
fi

###
# Lod directory
date_str="`date +'%Y%m%d%H%M%S'`"
mkdir $date_str

# set parameters manually.
bladerfargs="-t 2024/01/15,05:00:00 -e $confpath/../src/bladeGPS/brdc0150.24n -d 86400 -l 0.0,135.0,0.0 -L 0.0,135.0,400000000.0 -s ./$date_str/ -a -20 -r 0,90 -R 0,-90 -p -E -I -v"
# bladerfargs="-t 2024/01/07,00:00:00 -e $confpath/../src/bladeGPS/brdc0070.24n -d 86400 -l 0.0,135.0,0.0 -L 0,135.0,400000000.0 -s ./$date_str/ -a -22 -r 0,90 -R 0,-90 -p -E -I -v"

###
# run
# valgrind --tool=cachegrind gnss_sim_receiver \
export SPICE_KERNEL_DIR="$HOME/work/cspice"
gnss_sim_receiver \
    --config_file=$conffile \
    --log_dir=$date_str \
    --bladegps_args="$bladerfargs"

###
# copy the pvt dump
cp pvt.dat $date_str
# generate log of clock offset
pushd .
cd $date_str
cat gnss_sim_receiver.INFO | grep "dt_current" > dt_current.txt && sed -i 's/.*current: //g' dt_current.txt
cat gnss_sim_receiver.INFO | grep "dt0_current" > dt0_current.txt && sed -i 's/.*current: //g' dt0_current.txt
cat gnss_sim_receiver.INFO | grep "dt_0" > dt0.txt && sed -i 's/.*\[s\]: //g' dt0.txt
cat gnss_sim_receiver.INFO | grep "dt_GNSSR-AOWR \[s\]" > dt_gnssr_aowr.txt && sed -i 's/.*\[s\]: //g' dt_gnssr_aowr.txt
cat gnss_sim_receiver.INFO | grep "dt_GNSSR-AOWR CP \[s\]" > dt_gnssr_aowr_cp.txt && sed -i 's/.*\[s\]: //g' dt_gnssr_aowr_cp.txt
cat gnss_sim_receiver.INFO | grep "tau \[s\]" > tau.txt && sed -i 's/.*\[s\]: //g' tau.txt

merge_script=$confpath"/../scripts/merge_txt_to_csv.sh"
bash $merge_script
plot_script=$confpath"/../scripts/python/plot_non_pvt_log.py"
python3 $plot_script
popd