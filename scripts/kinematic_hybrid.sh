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
date_str="`date +'%Y%m%d%H%M%S'`"
mkdir $date_str

# set parameters manually. NOTE: SC came out from the lunar occultation 01:18 <- LLO
# NOTE: user_motion_file should has the suffix of _ecef and _eci
u_option="" # Ch1
U_option="-U $confpath/../src/bladeGPS/lto_states_ecef.csv" # Ch2
bladerfargs="-t 2024/01/15,01:00:00 -e $confpath/../src/bladeGPS/brdc0150.24n -d 4400 -l 0.0,135.0,0.0 $u_option $U_option -s ./$date_str/ -a -15 -r 0,90 -R 0,-90 -p -E -I"
# bladerfargs="-t 2024/01/15,01:00:00 -e $confpath/../src/bladeGPS/brdc0150.24n -d 86400 -l 0.0,135.0,0.0 -U ../src/bladeGPS/LLO_1818_1day.csv -s ./$date_str/ -a -15 -r 0,90 -R 0,-90 -p -E -I -f"

###
# run
# valgrind --tool=cachegrind gnss_sim_receiver \
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

merge_script=$confpath"/../scripts/merge_txt_to_csv.sh"
bash $merge_script
plot_script=$confpath"/../scripts/python/plot_non_pvt_log.py"
python3 $plot_script

eci_log_generator=$confpath"/../scripts/python/generate_user_states_eci.py"
# Ch1
if [ ! -z "$u_option" ]; then
    ecef_motion_file="${u_option:3}"
    python3 $eci_log_generator ./user_states_ch1.txt "${ecef_motion_file:0:-8}eci.csv"
fi
# Ch2
if [ ! -z "$U_option" ]; then
    ecef_motion_file="${U_option:3}"
    python3 $eci_log_generator ./user_states_ch2.txt "${ecef_motion_file:0:-8}eci.csv"
fi

popd