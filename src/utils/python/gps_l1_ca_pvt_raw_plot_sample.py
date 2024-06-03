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
   nav_sol_period    - Measurement period [ms]
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

import numpy as np
import pyproj
import utm
from lib.gps_l1_ca_read_pvt_dump import gps_l1_ca_read_pvt_dump
from lib.plotNavigation import plotNavigation
from lib.plotPosition import plot_oneVStime, plot_position

settings = {}
utm_e = []
utm_n = []
E_UTM = []
N_UTM = []
utm_zone = []

# ---------- CHANGE HERE:
# samplingFreq = 3e6
# channels = 8
path = '/home/junichiro/work/gnss_sim_receiver/test/on_ground/'
pvt_raw_log_path = path + 'pvt.dat'
nav_sol_period = 10
plot_skyplot = 0

# FIXME: should be read from file. add llh, it is easier?
settings['true_position'] = {
                            # 'E_UTM':np.nan,'N_UTM':np.nan,'U_UTM':np.nan, 'X_ECEF':-4474292, 'Y_ECEF':2675793, 'Z_ECEF':-3663100} # Birch building -35.274508, 149.119008, 568
                            'E_UTM':500000,'N_UTM':3873043.06,'U_UTM':0, 'X_ECEF':-3698470, 'Y_ECEF':3698470, 'Z_ECEF':3637867} # 35, 135, 0
                            # 'E_UTM':500000,'N_UTM':0,'U_UTM':4e8, 'X_ECEF':-287352736.0, 'Y_ECEF':287352736.0, 'Z_ECEF':0.0} # 0, 135, 4e8
settings['navSolPeriod'] = nav_sol_period

navSolutions = gps_l1_ca_read_pvt_dump(pvt_raw_log_path)
# NOTE: this is in ECEF
X, Y, Z = navSolutions['X'], navSolutions['Y'], navSolutions['Z']
# copy
navSolutions['X_ECEF'] = X
navSolutions['Y_ECEF'] = Y
navSolutions['Z_ECEF'] = Z

utm_coords = []

for i in range(len(navSolutions['longitude'])):
    utm_coords.append(utm.from_latlon(navSolutions['latitude'][i],
                                      navSolutions['longitude'][i]))



for i in range(len(utm_coords)):
    utm_e.append(utm_coords[i][0])
    utm_n.append(utm_coords[i][1])
    utm_zone.append(utm_coords[i][2])

# Transform from Lat Long degrees to UTM coordinate system
# It's supposed utm_zone and letter will not change during tracking
input_projection = pyproj.CRS.from_string("+proj=longlat "
                                          "+datum=WGS84 +no_defs")

for i in range(len(navSolutions['longitude'])):
    output_projection = pyproj.CRS (f"+proj=utm +zone={utm_zone[i]} "
                                    f"+datum=WGS84 +units=m +no_defs")
    transformer = pyproj.Transformer.from_crs(input_projection,
                                              output_projection)
    utm_e, utm_n = transformer.transform(navSolutions['longitude'][i],
                                         navSolutions['latitude'][i])
    E_UTM.append(utm_e)
    N_UTM.append(utm_n)

# For up
U_UTM = []
transformer = pyproj.Transformer.from_crs(
    {"proj":'geocent', "ellps":'WGS84', "datum":'WGS84'},
    {"proj":'latlong', "ellps":'WGS84', "datum":'WGS84'},
    )
for i in range(len(X)):
    lon, lat, alt = transformer.transform(X[i], Y[i], Z[i])
    U_UTM.append(alt)

navSolutions['E_UTM'] = E_UTM
navSolutions['N_UTM'] = N_UTM
navSolutions['U_UTM'] = U_UTM

plotNavigation(navSolutions, settings, path, 'UTM', plot_skyplot)
plotNavigation(navSolutions, settings, path, 'ECEF', plot_skyplot)

# OPTIONAL: Other plots ->
plot_position(navSolutions, path)
# plot_oneVStime(navSolutions, 'X_vel', path)
plot_oneVStime(navSolutions, 'Tot_Vel', path)