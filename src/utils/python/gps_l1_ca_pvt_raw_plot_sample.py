"""
 gps_l1_ca_pvt_raw_plot_sample.py

 Reads GNSS-SDR PVT raw dump binary file using the provided function and plots
 some internal variables

 Irene Pérez Riega, 2023. iperrie@inta.es

 Modifiable in the file:
   sampling_freq     - Sampling frequency [Hz]
   channels          - Number of channels to check if they exist
   path              - Path to folder which contains raw file
   pvt_raw_log_path  - Completed path to PVT raw data file
   nav_sol_period_ms    - Measurement period [ms]
   plot_skyplot      - = 1 -> Sky Plot (TO DO) // = 0 -> No Sky Plot
   true_position     - In settings, If not known enter all NaN's and mean
                        position will be used as a reference in UTM
                        coordinate system
   plot_position     - Optional function at the end
   plot_oneVStime    - Optional function at the end, select variable to plot

 -----------------------------------------------------------------------------

 GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 This file is part of GNSS-SDR.

 Copyright (C) 2022  (see AUTHORS file for a list of contributors)
 SPDX-License-Identifier: GPL-3.0-or-later

 -----------------------------------------------------------------------------
"""

import matplotlib.pyplot as plt
import numpy as np
import pyproj
import utm
from lib.gps_l1_ca_read_pvt_dump import gps_l1_ca_read_pvt_dump
from lib.plotNavigation import plotNavigation
from lib.plotPosition import plot_oneVStime, plot_position
from lib.plotVisibility import plotVisibility
from lib.read_user_position import ecef_to_utm, get_interpolated_positions

settings = {}

# ---------- CHANGE HERE:
# samplingFreq = 3e6
# channels = 8
# path = '/home/junichiro/Desktop/'
path = '/home/junichiro/work/gnss_sim_receiver/test/'
# path = '/home/junichiro/work/gnss_sim_receiver/test/cislunar/'
pvt_raw_log_path = path + 'pvt.dat'
nav_sol_period_ms = 100
plot_skyplot = 0
user_position_file_path = path + 'log/20241216173110_ch1.txt'
visibility_file_path = path + "log/20241216173110_visibility_ch1.txt"
dynamic = True

settings['navSolPeriod'] = nav_sol_period_ms

navSolutions = gps_l1_ca_read_pvt_dump(pvt_raw_log_path)
# FIXME: add csv generator from pvt dump to generate plot.
if dynamic:
    true_position = get_interpolated_positions(user_position_file_path,
                                               np.array(navSolutions['RxTime']) - np.array(navSolutions['dt']))
    # For debug
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

    position_time = np.array(navSolutions['RxTime']) - np.array(navSolutions['dt'])
    color = 'tab:blue'
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('X (m)', color=color)
    ax1.plot(position_time, true_position[0], color=color, label='X')
    ax1.tick_params(axis='y', labelcolor=color)
    ax1.set_title('SC position vs Time')
    ax1.grid(True)
    ax2.set_ylabel('Y (m)', color=color)
    ax2.plot(position_time, true_position[1], color=color, label='Y')
    ax2.tick_params(axis='y', labelcolor=color)
    ax2.grid(True)
    ax3.set_ylabel('Z (m)', color=color)
    ax3.plot(position_time, true_position[2], color=color, label='Z')
    ax3.tick_params(axis='y', labelcolor=color)
    ax3.grid(True)

    plt.tight_layout()
    plt.show()

settings['true_position'] = {
                            # 'E_UTM':np.nan,'N_UTM':np.nan,'U_UTM':np.nan, 'X_ECEF':-4474292, 'Y_ECEF':2675793, 'Z_ECEF':-3663100} # Birch building -35.274508, 149.119008, 568
                            # 'E_UTM':500000,'N_UTM':3873043.06,'U_UTM':0, 'X_ECEF':-3698470, 'Y_ECEF':3698470, 'Z_ECEF':3637867} # 35, 135, 0
                            # 'E_UTM':500000,'N_UTM':0.0,'U_UTM':0, 'X_ECEF':-4510024, 'Y_ECEF':4510024, 'Z_ECEF':0.0} # 0, 135, 0
                            # 'E_UTM':690940.77,'N_UTM':6091664.70,'U_UTM':578.0, 'X_ECEF':-4472009, 'Y_ECEF':2676442, 'Z_ECEF':-3665415} # -35.3, 149.1, 578.0 (ANU)
                            # 'E_UTM':500000,'N_UTM':0,'U_UTM':4e8, 'X_ECEF':-287352736.0, 'Y_ECEF':287352736.0, 'Z_ECEF':0.0} # 0, 135, 4e8
                            'E_UTM': true_position[3],'N_UTM': true_position[4],'U_UTM': true_position[5], 'X_ECEF': true_position[0], 'Y_ECEF': true_position[1], 'Z_ECEF': true_position[2]} # dynamic, LEO

# NOTE: this is in ECEF
X, Y, Z = navSolutions['X'], navSolutions['Y'], navSolutions['Z']
# copy
navSolutions['X_ECEF'] = np.array(X)
navSolutions['Y_ECEF'] = np.array(Y)
navSolutions['Z_ECEF'] = np.array(Z)

ecef_positions = np.array([X, Y, Z])
utm_position = ecef_to_utm(ecef_positions)
E_UTM = utm_position[0]
N_UTM = utm_position[1]
# To avoid the discontinuity in UTM result TODO: for dynamic data.
if not dynamic and settings['true_position']['N_UTM'] == 0:
  N_UTM = np.where(N_UTM >= 9e6, N_UTM - 1e7, N_UTM)
U_UTM = utm_position[2]

navSolutions['E_UTM'] = np.array(E_UTM)
navSolutions['N_UTM'] = np.array(N_UTM)
navSolutions['U_UTM'] = np.array(U_UTM)

plotNavigation(navSolutions, settings, path, 'UTM', plot_skyplot, dynamic)
plotNavigation(navSolutions, settings, path, 'ECEF', plot_skyplot, dynamic)
plotVisibility(visibility_file_path, path)

# OPTIONAL: Other plots ->
if not dynamic:
    plot_position(navSolutions, path)

# plot_oneVStime(navSolutions, 'X_vel', path)
plot_oneVStime(navSolutions, 'Tot_Vel', path)