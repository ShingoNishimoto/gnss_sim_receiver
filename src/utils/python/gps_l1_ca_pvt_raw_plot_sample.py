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

from datetime import datetime, timedelta

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import pyproj
import utm
from lib.gps_l1_ca_read_pvt_dump import gps_l1_ca_read_pvt_dump
from lib.plotNavigation import plotNavigation
from lib.plotPosition import plot_oneVStime, plot_position
from lib.plotVisibility import plotVisibility
from lib.read_user_position import (batch_eci_to_rtn_full, ecef_to_eci_simple,
                                    ecef_to_ecij2000, ecef_to_utm,
                                    get_interpolated_positions, gps_to_gmst,
                                    gps_to_utc)

settings = {}

# ---------- CHANGE HERE:
# samplingFreq = 3e6
# channels = 8
is_GS = False
dynamic = True
full_ephemeris = False
# gs_log_path = '/home/junichiro/work/gnss_sim_receiver/test/20250908123648/' # 2body
# gs_log_path = '/home/junichiro/work/gnss_sim_receiver/test/20250909102229/' # 2body simple
# gs_log_path = '/home/junichiro/work/gnss_sim_receiver/test/20250808101451/' # Full dynamics
gs_log_path = '/home/junichiro/work/gnss_sim_receiver/test/20251012170235/'
# gs_log_path = '/home/junichiro/Desktop/'
if is_GS:
  path = gs_log_path
  log_suffix = "_ch1.txt"
else:
  path = '/home/junichiro/Desktop/'
  # path = '/home/junichiro/work/gnss_sim_receiver/test/cislunar/'
  log_suffix = "_ch2.txt"

pvt_raw_log_path = path + 'pvt.dat'
nav_sol_period_ms = 1000
plot_skyplot = 0
user_states_file_path = gs_log_path + "user_states" + log_suffix
# NOTE: this is important only when dynamic mode
user_states_eci_file_path = gs_log_path + "user_states_eci" + log_suffix
visibility_file_path = gs_log_path + "visibility" + log_suffix

settings['navSolPeriod'] = nav_sol_period_ms

navSolutions = gps_l1_ca_read_pvt_dump(pvt_raw_log_path)
if dynamic:
  clock_offset = np.array(navSolutions['dt[s]'])
  true_position = get_interpolated_positions(user_states_file_path,
                                               np.array(navSolutions['RxTime']) - clock_offset)
  true_position_inertial = get_interpolated_positions(user_states_eci_file_path,
                                               np.array(navSolutions['RxTime']) - clock_offset,
                                               True)

  # For debug
  fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

  position_time = np.array(navSolutions['RxTime']) - clock_offset
  color = 'tab:blue'
  ax1.set_xlabel('Time (s)')
  ax1.set_ylabel('X (m)', color=color)
  ax1.plot(position_time, true_position[1], color=color, label='X')
  ax1.tick_params(axis='y', labelcolor=color)
  ax1.set_title('SC position vs Time')
  ax1.grid(True)
  ax2.set_ylabel('Y (m)', color=color)
  ax2.plot(position_time, true_position[2], color=color, label='Y')
  ax2.tick_params(axis='y', labelcolor=color)
  ax2.grid(True)
  ax3.set_ylabel('Z (m)', color=color)
  ax3.plot(position_time, true_position[3], color=color, label='Z')
  ax3.tick_params(axis='y', labelcolor=color)
  ax3.grid(True)

  plt.tight_layout()
  plt.show()

rotating_states_label = [
  'X_ECEF', 'Y_ECEF', 'Z_ECEF',
  'VX_ECEF', 'VY_ECEF', 'VZ_ECEF',
  'E_UTM', 'N_UTM', 'U_UTM', 'lat', 'lon'
]
inertial_states_label = [
  'X_ECI', 'Y_ECI', 'Z_ECI',
  'VX_ECI', 'VY_ECI', 'VZ_ECI',
  # 'R_RTN', 'T_RTN', 'N_RTN',
  # 'VR_RTN', 'VT_RTN', 'VN_RTN',
]

if is_GS:
  settings['true_position'] = {
    'E_UTM':500000,'N_UTM':0.0,'U_UTM':0, 'X_ECEF':-4510024, 'Y_ECEF':4510024, 'Z_ECEF':0.0, 'lat': np.deg2rad(0), 'lon': np.deg2rad(135) # 0, 135, 0
    # 'E_UTM':690940.77,'N_UTM':6091664.70,'U_UTM':578.0, 'X_ECEF':-4472009, 'Y_ECEF':2676442, 'Z_ECEF':-3665415} # -35.3, 149.1, 578.0 (ANU)
    # 'E_UTM':500000,'N_UTM':3873043.06,'U_UTM':0, 'X_ECEF':-3698470, 'Y_ECEF':3698470, 'Z_ECEF':3637867} # 35, 135, 0
  }
else:
  if dynamic:
    settings['true_position'] = { rotating_states_label[i]: true_position[i + 1] for i in range(len(rotating_states_label)) }
    for i in range(len(inertial_states_label)):
      settings['true_position'][inertial_states_label[i]] = true_position_inertial[i + 1]
  else:
    settings['true_position'] = {
      'E_UTM':500000,'N_UTM':0,'U_UTM':4e8, 'X_ECEF':-287352736.0, 'Y_ECEF':287352736.0, 'Z_ECEF':0.0, 'lat': np.deg2rad(0), 'lon': np.deg2rad(135) # 0, 135, 4e8
      # 'E_UTM':500000,'N_UTM':0,'U_UTM':1e8, 'X_ECEF':-75220702.0, 'Y_ECEF':75220702.0, 'Z_ECEF':0.0, 'lat': np.deg2rad(0), 'lon': np.deg2rad(135)} # 0, 135, 1e8
    }

# NOTE: this is in ECEF
X, Y, Z, VX, VY, VZ = navSolutions['X'], navSolutions['Y'], navSolutions['Z'], navSolutions['X_vel'], navSolutions['Y_vel'], navSolutions['Z_vel']

ecef_positions = np.array([X, Y, Z])
ecef_velocities = np.array([VX, VY, VZ])
for i in range(3):
  navSolutions[rotating_states_label[i]] = ecef_positions[i]
  navSolutions[rotating_states_label[3 + i]] = ecef_velocities[i]

if not dynamic:
  ones_base = np.ones_like(X)
  true_position = np.array([
    settings['true_position']['X_ECEF'] * ones_base,
    settings['true_position']['Y_ECEF'] * ones_base,
    settings['true_position']['Z_ECEF'] * ones_base])
  # Add velocity (0)
  for i in range(3):
    settings['true_position'][rotating_states_label[3 + i]] = 0

if is_GS:
  utm_position = ecef_to_utm(ecef_positions, true_position)
else:
  utm_position = ecef_to_utm(ecef_positions, true_position[1:4])
E_UTM = utm_position[0]
N_UTM = utm_position[1]
# To avoid the discontinuity in UTM result TODO: for dynamic data.
if not dynamic and settings['true_position']['N_UTM'] == 0:
  N_UTM = np.where(N_UTM >= 9e6, N_UTM - 1e7, N_UTM)
U_UTM = utm_position[2]

navSolutions['E_UTM'] = np.array(E_UTM)
navSolutions['N_UTM'] = np.array(N_UTM)
navSolutions['U_UTM'] = np.array(U_UTM)

# ECEF to ENU
zero = np.zeros_like(settings['true_position']['lon']) if dynamic else 0
ECEF2ENU = np.array([np.array([-np.sin(settings['true_position']['lon']), np.cos(settings['true_position']['lon']), zero]),
                     np.array([-np.sin(settings['true_position']['lat']) * np.cos(settings['true_position']['lon']), -np.sin(settings['true_position']['lat']) * np.sin(settings['true_position']['lon']), np.cos(settings['true_position']['lat'])]),
                     np.array([ np.cos(settings['true_position']['lat']) * np.cos(settings['true_position']['lon']),  np.cos(settings['true_position']['lat']) * np.sin(settings['true_position']['lon']), np.sin(settings['true_position']['lat'])])])

if dynamic:
  # ECEF to ECI
  user_states_eci = np.loadtxt(user_states_eci_file_path, delimiter=',', skiprows=1)
  # sidereal_day = 86164.0905  # seconds
  t0_gps = user_states_eci[0, 0]
  t0_gmst = user_states_eci[0, 1]
  t_offset = (gps_to_gmst(navSolutions['WEEK'][0], t0_gps) - t0_gmst) # in SI seconds
  omega_earth = 7.2921150e-5
  one_rev_periods = 2 * np.pi / omega_earth # Earth rotation period not one day.
  if full_ephemeris:
    t_utc_solution = (gps_to_utc(position_time - t_offset, np.array(navSolutions['WEEK'])))
    position_eci, velocity_eci = ecef_to_ecij2000(ecef_positions.T, ecef_velocities.T, t_utc_solution)
  else:
    # t_gmst_solution = (gps_to_gmst(np.array(navSolutions['WEEK']), position_time) - t_offset) % one_rev_periods # in SI seconds
    gmst_solution = true_position_inertial[0] # Use inertial one for more precision.
    navSolutions['gmst[rad]'] = gmst_solution
    position_eci, velocity_eci = ecef_to_eci_simple(ecef_positions.T, ecef_velocities.T, gmst_solution)
  for i in range(3):
    navSolutions[inertial_states_label[i]] = position_eci.T[i]
    navSolutions[inertial_states_label[3 + i]] = velocity_eci.T[i]

  # ECI to RTN
  position_rtn, velocity_rtn = batch_eci_to_rtn_full(
    position_eci, velocity_eci,
    true_position_inertial[1:4].T,
    true_position_inertial[4:7].T
    )
  rtn_label = [
    'R_RTN', 'T_RTN', 'N_RTN',
    'VR_RTN', 'VT_RTN', 'VN_RTN',
  ]
  for i in range(3):
    navSolutions[rtn_label[i]] = position_rtn.T[i]
    navSolutions[rtn_label[3 + i]] = velocity_rtn.T[i]

# Compute states error
# ECEF
for label in rotating_states_label[:-2]:
  navSolutions['error_' + label] = navSolutions[label] - settings['true_position'][label]
if dynamic:
  # ECI
  for label in inertial_states_label:
    navSolutions['error_' + label] = navSolutions[label] - settings['true_position'][label]

# Stack ECEF error vectors into shape (N, 3)
ecef_errors = np.vstack([
    navSolutions['error_X_ECEF'],
    navSolutions['error_Y_ECEF'],
    navSolutions['error_Z_ECEF']
]).T  # shape (N, 3)

if dynamic:
    # ECEF2ENU: shape (3, 3, N)
    # Transpose each 3x3 rotation matrix: (3, 3, N) → (N, 3, 3)
    ECEF2ENU_T = np.transpose(ECEF2ENU, (2, 0, 1))  # (N, 3, 3)
    ECEF2ENU_T = np.transpose(ECEF2ENU_T, (0, 2, 1))  # Transpose each (3x3)
    # Transform each error vector: (N, 3, 3) @ (N, 3, 1) → (N, 3)
    enu_errors_array = np.einsum('nij,nj->ni', ECEF2ENU_T, ecef_errors)
else:
    # Static rotation: shape (3, 3)
    enu_errors_array = (ECEF2ENU @ ecef_errors.T).T  # shape (N, 3)

# Assign ENU components back to navSolutions
enu_label = ['E_ENU', 'N_ENU', 'U_ENU']
for i in range(3):
    navSolutions[enu_label[i]] = enu_errors_array[:, i]

# save to csv file
df = pd.DataFrame.from_dict(navSolutions)
csv_file_name = path + "pvt.csv"
df.to_csv(csv_file_name)
excel_file_name = path + "pvt.xlsx"
df.to_excel(excel_file_name)

plotNavigation(navSolutions, settings, path, 'UTM', plot_skyplot, dynamic)
plotNavigation(navSolutions, settings, path, 'ECEF', plot_skyplot, dynamic)
# plotVisibility(visibility_file_path, path)

# OPTIONAL: Other plots ->
if not dynamic:
    plot_position(navSolutions, path)

# plot_oneVStime(navSolutions, 'X_vel', path)
plot_oneVStime(navSolutions, 'Tot_Vel', path)