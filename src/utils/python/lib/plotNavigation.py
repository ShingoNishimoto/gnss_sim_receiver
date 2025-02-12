"""
 plotNavigation.py

 Function plots variations of coordinates over time and a 3D position
 plot. It plots receiver coordinates in UTM system or coordinate offsets if
 the true UTM receiver coordinates are provided.

 Irene Pérez Riega, 2023. iperrie@inta.es

 plotNavigation(navSolutions, settings, plot_skyplot)

   Args:
       navSolutions    - Results from navigation solution function. It
                       contains measured pseudoranges and receiver
                      coordinates.
       settings        - Receiver settings. The true receiver coordinates
                       are contained in this structure.
       plot_skyplot    - If == 1 then use satellite coordinates to plot the
                       satellite positions (not implemented yet TO DO)

   Modifiable in the file:
       fig_path        - Path where plots will be save

 -----------------------------------------------------------------------------

 GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 This file is part of GNSS-SDR.

 Copyright (C) 2022  (see AUTHORS file for a list of contributors)
 SPDX-License-Identifier: GPL-3.0-or-later

 -----------------------------------------------------------------------------
"""

import os

import matplotlib.pyplot as plt
import numpy as np


def plotNavigation(navSolutions, settings, path, coord='UTM', plot_skyplot=0, dynamic=False):

    # ---------- CHANGE HERE:
    fig_path = path + 'figure/'

    if not os.path.exists(fig_path):
        os.makedirs(fig_path)

    if coord == 'UTM':
        position_label = ['E_UTM', 'N_UTM', 'U_UTM']
        axis_label = ['East[m]', 'North[m]', 'Up[m]']
    elif coord == 'ECEF':
        position_label = ['X_ECEF', 'Y_ECEF', 'Z_ECEF']
        axis_label = ['X[m]', 'Y[m]', 'Z[m]']
    else:
        exit()

    if navSolutions:
        if dynamic:
            true_pos_x = settings['true_position'][position_label[0]].any()
            true_pos_y = settings['true_position'][position_label[1]].any()
            true_pos_z = settings['true_position'][position_label[2]].any()
        else:
            true_pos_x = settings['true_position'][position_label[0]]
            true_pos_y = settings['true_position'][position_label[1]]
            true_pos_z = settings['true_position'][position_label[2]]
        if (np.isnan(true_pos_x) or
                np.isnan(true_pos_y) or
                np.isnan(true_pos_z)):

            # Compute mean values
            ref_coord = {
                position_label[0]: np.nanmean(navSolutions[position_label[0]]),
                position_label[1]: np.nanmean(navSolutions[position_label[1]]),
                position_label[2]: np.nanmean(navSolutions[position_label[2]])
            }

            mean_latitude = np.nanmean(navSolutions['latitude'])
            mean_longitude = np.nanmean(navSolutions['longitude'])
            mean_height = np.nanmean(navSolutions['height'])
            ref_point_lg_text = (f"Mean Position\nLat: {mean_latitude}º\n"
                                 f"Long: {mean_longitude}º\n"
                                 f"Hgt: {mean_height:+6.1f}")

        else:
            # Compute the mean error for static receiver
            ref_coord = {
                position_label[0]: settings['true_position'][position_label[0]],
                position_label[1]: settings['true_position'][position_label[1]],
                position_label[2]: settings['true_position'][position_label[2]]
            }

            error_positions = {
                position_label[0]: navSolutions[position_label[0]] - ref_coord[position_label[0]],
                position_label[1]: navSolutions[position_label[1]] - ref_coord[position_label[1]],
                position_label[2]: navSolutions[position_label[2]] - ref_coord[position_label[2]]
            }

            rmse_3d = np.sqrt(
                np.nanmean(error_positions[position_label[0]] ** 2) +
                np.nanmean(error_positions[position_label[1]] ** 2) +
                np.nanmean(error_positions[position_label[2]] ** 2)
            )

            ref_point_lg_text = (f"Reference Position, Mean 3D error = "
                                 f"{rmse_3d} [m]")

        #Create plot and subplots
        plt.figure(figsize=(1920 / 120, 1080 / 120))
        plt.clf()
        # plt.title('Navigation solutions',fontweight='bold')

        ax1 = plt.subplot(4, 2, (1, 4))
        ax2 = plt.subplot(4, 2, (5, 7), projection='3d')
        ax3 = plt.subplot(4, 2, (6, 8), projection='3d')

        #  (ax1) Coordinate differences in the system from reference point
        ax1.plot(np.array(navSolutions["RxTime"]), np.vstack([np.array(navSolutions[position_label[0]]) - ref_coord[position_label[0]],
                    np.array(navSolutions[position_label[1]]) - ref_coord[position_label[1]],
                    np.array(navSolutions[position_label[2]]) - ref_coord[position_label[2]]]).T)
        ax1.set_title('Coordinates variations in ' + coord + ' system', fontweight='bold')
        ax1.legend([position_label[0], position_label[1], position_label[2]])
        ax1.set_xlabel(f"Rx Time (s)")
        # ax1.set_xlabel(f"Rx Time: {settings['navSolPeriod']} s")
        ax1.set_ylabel('Errors (m)')
        ax1.grid(True)
        ax1.axis('tight')

        # (ax2) Satellite sky plot
        if plot_skyplot: #TODO: posicion de los satelites
            skyPlot(ax2, navSolutions['channel']['az'],
                    navSolutions['channel']['el'],
                    navSolutions['channel']['PRN'][:, 0])
            ax2.set_title(f'Sky plot (mean PDOP: '
                          f'{np.nanmean(navSolutions["DOP"][1, :]):.1f})',
                          fontweight='bold')

        # (ax3) Position plot in the system
        ax3.scatter(np.array(navSolutions[position_label[0]]) - ref_coord[position_label[0]],
                    np.array(navSolutions[position_label[1]]) - ref_coord[position_label[1]],
                    np.array(navSolutions[position_label[2]]) - ref_coord[position_label[2]], marker='+')
        ax3.scatter([0], [0], [0], color='r', marker='+', linewidth=1.5)
        ax3.view_init(0, 90)
        ax3.set_box_aspect([1, 1, 1])
        ax3.grid(True, which='minor')
        ax3.legend(['Measurements', ref_point_lg_text])
        ax3.set_title('Positions in ' + coord + ' system (3D plot)',fontweight='bold')
        ax3.set_xlabel(axis_label[0])
        ax3.set_ylabel(axis_label[1])
        ax3.set_zlabel(axis_label[2])

        plt.tight_layout()
        plt.savefig(os.path.join(fig_path, 'measures_' + coord + '.png'))
        plt.show()
