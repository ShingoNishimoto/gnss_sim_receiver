/*!
 * \file pvt_ekf.h
 * \brief Extended Kalman Filter for Position, Velocity, and Clock bias and drift
 * \author Shingo Nishimoto, 2024.
 *
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2023  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_PVT_EKF_H
#define GNSS_SDR_PVT_EKF_H

#include "../../libs/environment/frame.h"
#include "../../libs/rtklib/rtklib.h"
#include <armadillo>

/** \addtogroup PVT
 * \{ */
/** \addtogroup PVT_libs
 * \{ */


// Frame information. Dynamics, perturbation, etc.
enum FrameType
{
    ECI,  // Earth Centered Inertia
    MCI,  // Moon Centered Inertia
    MAX
};

/*!
 * \brief Extended Kalman Filter for Position and Velocity
 *
 */
class Pvt_Ekf
{
public:
    Pvt_Ekf() = default;
    ~Pvt_Ekf();
    void init_Ekf(const arma::vec& x_ecef,
        const gtime_t gps_time_s,
        enum FrameType frame_type,
        double update_interval_s,
        double initial_pos_sd_m,
        double initial_vel_sd_ms,
        double measures_pos_sd_m,
        double measures_vel_sd_ms,
        double system_pos_sd_m,
        double system_vel_sd_ms);
    bool is_initialized() const;
    int run_Ekf(rtk_t *rtk, const obsd_t *obs, int n, const nav_t *nav);
    void get_states_Kf(arma::vec& x_ecef) const;
    void reset_Kf();

private:
    // Kalman Filter class variables
    arma::mat d_F;  // STM
    // arma::mat d_H;
    // arma::mat d_R;
    arma::mat d_Q;
    arma::mat d_P_old_old;
    arma::mat d_P_new_old;
    arma::mat d_P_new_new;
    arma::vec d_x_old_old;
    arma::vec d_x_new_old;
    arma::vec d_x_new_new;
    double d_measures_pos_sd_m;
    double d_measures_vel_sd_ms;
    bool d_initialized{false};
    double d_update_interval_s;
    double d_tt;  // terrestrial time

    enum FrameType d_state_frame_type;
    Frame* d_frame;

    void predict(const double tt);
    arma::vec state_derivative(const arma::vec& x, const double tt) const;
    arma::mat JacobiMatrix(const arma::vec& x, const double tt) const;
    void conv_states_ecef2i(const arma::vec& states_ecef, arma::vec& states_i);
    void conv_states_i2ecef(const arma::vec& states_i, arma::vec& states_ecef);
    void conv_state_matrix_i2ecef(const arma::mat& mat_i, arma::mat& mat_ecef);
    void conv_state_matrix_ecef2i(const arma::mat& mat_ecef, arma::mat& mat_i);
    uint8_t get_observation(rtk_t* rtk, const obsd_t* obs, int n, const nav_t* nav, arma::vec& res, arma::mat& H, arma::mat& R, const arma::vec& states);
};


/** \} */
/** \} */
#endif  // GNSS_SDR_PVT_EKF_H
