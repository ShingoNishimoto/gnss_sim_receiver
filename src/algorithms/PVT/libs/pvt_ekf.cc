/*!
 * \file pvt_ekf.cc
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

#include "pvt_ekf.h"
#include "../../libs/rtklib/rtklib_ephemeris.h"
#include "../../libs/rtklib/rtklib_rtkcmn.h"
#include "../../libs/rtklib/rtklib_pntpos.h"
#include "../../libs/rtklib/rtklib_rtkpos.h"
#include <glog/logging.h>
#include <iomanip>
#include <cassert>

// #define EKF_DEBUG
// #define EKF_ECEF

Pvt_Ekf::~Pvt_Ekf()
{
    delete d_frame;
}


void Pvt_Ekf::init_Ekf(const arma::vec& x_ecef,
    const gtime_t gps_time_s,
    enum FrameType frame_type,
    double update_interval_s,
    double initial_pos_sd_m,
    double initial_vel_sd_ms,
    double measures_pos_sd_m,
    double measures_vel_sd_ms,
    double system_pos_sd_m,
    double system_vel_sd_ms,
    double system_clock_offset_sd_m,
    double system_clock_drift_sd_ms)
{
    // For the frame
    TimeSystem* ts = new TimeSystem();
    TimeSystem::gtime_t ts_gps_time_s = {gps_time_s.time, gps_time_s.sec};
    d_tt = ts->ConvGPSTimeToTt(ts_gps_time_s);
    Earth* earth_ptr = new Earth(d_tt);
    // Moon d_moon(d_tt, earth_ptr->GetGravityConst());
    // d_frame = new Frame(&d_earth, &d_moon, &ts);
    d_frame = new Frame(earth_ptr, new Moon(d_tt, earth_ptr->GetGravityConst()), ts);
    // Convert states from ECEF to the desired type.
    d_state_frame_type = frame_type;
    arma::vec states_ecef = arma::join_vert(x_ecef.subvec(0, 2), x_ecef.subvec(4, 6));
    arma::vec states_i = arma::zeros(6, 1);
    conv_states_ecef2i(states_ecef, states_i);

    // states: position [m], clock offset [m], velocity [m/s], clock drift [m/s]
    d_x_old_old = x_ecef;
#ifndef EKF_ECEF
    d_x_old_old.subvec(0, 2) = states_i.subvec(0, 2);
    d_x_old_old.subvec(4, 6) = states_i.subvec(3, 5);
#endif // EKF_ECEF

    d_x_new_old = d_x_old_old;
    d_x_new_new = d_x_old_old;

    // Kalman Filter class variables
    d_update_interval_s = update_interval_s;
    const double Ti = update_interval_s;

    d_F = arma::eye(8, 8);
    d_F.at(3, 3) = 0.0;
    d_F.at(7, 7) = 0.0;

    d_measures_pos_sd_m = measures_pos_sd_m;
    d_measures_vel_sd_ms = measures_vel_sd_ms;

    // system covariance matrix
    d_Q = arma::zeros(8, 8);
    arma::mat I_33 = arma::eye(3, 3);
    d_Q.submat(0, 0, 2, 2) = pow(system_pos_sd_m, 2) * I_33;
    d_Q(3, 3) = pow(system_clock_offset_sd_m, 2);  // Clock offset.
    d_Q.submat(4, 4, 6, 6) = pow(system_vel_sd_ms, 2) * I_33;
    d_Q(7, 7) = pow(system_clock_drift_sd_ms, 2);  // Clock drift.

    // initial Kalman covariance matrix
    d_P_old_old = d_Q;
    arma::mat I_44 = arma::eye(4, 4);
    d_P_old_old.submat(0, 0, 3, 3) = pow(initial_pos_sd_m, 2) * I_44;
    d_P_old_old.submat(4, 4, 7, 7) = pow(initial_vel_sd_ms, 2) * I_44;
    d_P_new_old = d_P_old_old;
    d_P_new_new = d_P_old_old;

    d_initialized = true;

    DLOG(INFO) << "Ti: " << Ti;
    DLOG(INFO) << "F: " << d_F;
    DLOG(INFO) << "Q: " << d_Q;
    DLOG(INFO) << "P: " << d_P_old_old;
    DLOG(INFO) << "x: " << d_x_old_old;
}


bool Pvt_Ekf::is_initialized() const
{
    return d_initialized;
}


void Pvt_Ekf::reset_Kf()
{
    d_initialized = false;
}


int Pvt_Ekf::run_Ekf(rtk_t *rtk, const obsd_t *obs, int n, const nav_t *nav)
{
    if (d_initialized)
        {
            d_tt += d_update_interval_s;

            // Kalman loop
            // Prediction
            predict(d_tt);

            DLOG(INFO) << "F: " << d_F;
            DLOG(INFO) << "P: " << d_P_new_old;
            DLOG(INFO) << "x: " << d_x_new_old;
            // reset F
            static arma::mat I_88 = arma::eye(8, 8);
            d_F = I_88;
            d_F.at(3, 3) = 0.0;
            d_F.at(7, 7) = 0.0;
            // Measurement update
            try
                {
                    // Pseudo range and doppler
                    arma::vec res = arma::zeros(n * 2, 1);
                    arma::mat H = arma::zeros(n * 2, 8);
                    arma::mat R = arma::zeros(n * 2, n * 2);
                    uint8_t nv = get_observation(rtk, obs, n, nav, res, H, R, d_x_new_old);
                    if (nv == 0)
                        throw nv;

                    if (nv != n)
                        {
                            res.resize(2 * nv);
                            H.resize(2 * nv, 8);
                            R.resize(2 * nv, 2 * nv);
                        }

#ifndef EKF_DEBUG
                    // P_ecef = DCM * P * DCMt
                    arma::span row_0_2(0, 2), row_3_5(3, 5), row_4_6(4, 6);
                    arma::span col_0_2(0, 2), col_3_5(3, 5), col_4_6(4, 6);
                    static arma::mat O_66 = arma::zeros(6, 6);
                    arma::mat P_i = O_66;
                    P_i(row_0_2, col_0_2) = d_P_new_old(row_0_2, col_0_2);
                    P_i(row_3_5, col_3_5) = d_P_new_old(col_4_6, col_4_6);
                    P_i(row_0_2, col_3_5) = d_P_new_old(row_0_2, col_4_6);
                    P_i(row_3_5, col_0_2) = d_P_new_old(col_4_6, col_0_2);
                    arma::mat P_all_ecef = d_P_new_old;
#ifndef EKF_ECEF
                    arma::mat P_ecef = O_66;
                    conv_state_matrix_i2ecef(P_i, P_ecef);
                    P_all_ecef(row_0_2, col_0_2) = P_ecef(row_0_2, col_0_2);
                    P_all_ecef(row_4_6, row_4_6) = P_ecef(col_3_5, col_3_5);
                    P_all_ecef(row_0_2, row_4_6) = P_ecef(row_0_2, col_3_5);
                    P_all_ecef(row_4_6, row_0_2) = P_ecef(col_3_5, col_0_2);
#endif // EKF_ECEF
                    // Kalman gain
                    // arma::mat K = P_all_ecef * H.t() * arma::inv(H * P_all_ecef * H.t() + R);
                    // Use solve function.
                    arma::mat B = H * P_all_ecef.t();
                    arma::mat A = (R + H * P_all_ecef * H.t()).t();
                    arma::mat Kt = arma::solve(A, B);
                    arma::mat K = Kt.t();

                    arma::vec x_part_i = arma::join_vert(d_x_new_old.subvec(0, 2), d_x_new_old.subvec(4, 6));
                    static arma::vec o_6 = arma::zeros(6, 1);
                    arma::vec x_part_ecef = o_6;
                    conv_states_i2ecef(x_part_i, x_part_ecef);
                    arma::vec x_new_old_ecef = d_x_new_old;
#ifndef EKF_ECEF
                    x_new_old_ecef(row_0_2) = x_part_ecef(row_0_2);
                    x_new_old_ecef(row_4_6) = x_part_ecef(row_3_5);
#endif // EKF_ECEF
                    DLOG(INFO) << "K: " << K;
                    DLOG(INFO) << "dx: " << K * res;
                    arma::vec x_new_new_ecef = x_new_old_ecef + K * res;

                    x_part_ecef(row_0_2) = x_new_new_ecef(row_0_2);
                    x_part_ecef(row_3_5) = x_new_new_ecef(row_4_6);
                    conv_states_ecef2i(x_part_ecef, x_part_i);
                    d_x_new_new = x_new_new_ecef;
                    // d_x_new_new(7) = x_new_new_ecef(7);
#ifndef EKF_ECEF
                    d_x_new_new(row_0_2) = x_part_i(row_0_2);
                    d_x_new_new(row_4_6) = x_part_i(row_3_5);
#endif  // EKF_ECEF
                    arma::mat I_KH = I_88 - K * H;
                    P_all_ecef = I_KH * P_all_ecef * I_KH.t() + K * R * K.t();
                    // P_all_ecef = I_KH * P_all_ecef;
                    // // Regularize
                    // const double epsilon_m = 1.0;
                    // const double epsilon_ms = 0.01;
                    // static const arma::mat I_44 = arma::eye(4, 4);
                    // P_all_ecef.submat(0, 0, 3, 3) += epsilon_m * I_44;
                    // P_all_ecef.submat(4, 4, 7, 7) += epsilon_ms * I_44;

#ifndef EKF_ECEF
                    // Update clock parts
                    // d_P_new_new.at(3, 3) = P_all_ecef.at(3, 3);
                    // d_P_new_new.at(7, 7) = P_all_ecef.at(7, 7);
                    // arma::vec cov_rv_dt_ecef = arma::join_vert(d_P_new_new.submat(0, 3, 2, 3), d_P_new_new.submat(4, 3, 6, 3));
                    // arma::vec cov_rv_dtv_ecef = arma::join_vert(d_P_new_new.submat(0, 7, 2, 7), d_P_new_new.submat(4, 7, 6, 7));
                    // arma::vec cov_rv_dt_i = o_6;
                    // arma::vec cov_rv_dtv_i = o_6;
                    // conv_states_ecef2i(cov_rv_dt_ecef, cov_rv_dt_i);
                    // conv_states_ecef2i(cov_rv_dtv_ecef, cov_rv_dtv_i);
                    // d_P_new_new.submat(0, 3, 2, 3) = cov_rv_dt_i.subvec(0, 2);
                    // d_P_new_new.submat(4, 3, 6, 3) = cov_rv_dt_i.subvec(3, 5);
                    // d_P_new_new.submat(3, 0, 3, 2) = cov_rv_dt_i.subvec(0, 2);
                    // d_P_new_new.submat(3, 4, 3, 6) = cov_rv_dt_i.subvec(3, 5);
                    // d_P_new_new.submat(0, 7, 2, 7) = cov_rv_dtv_i.subvec(0, 2);
                    // d_P_new_new.submat(4, 7, 6, 7) = cov_rv_dtv_i.subvec(3, 5);
                    // d_P_new_new.submat(7, 0, 7, 2) = cov_rv_dtv_i.subvec(0, 2);
                    // d_P_new_new.submat(7, 4, 7, 6) = cov_rv_dtv_i.subvec(3, 5);

                    // Convert P from ECEF to I, and substitute.
                    P_ecef(row_0_2, col_0_2) = P_all_ecef(row_0_2, col_0_2);
                    P_ecef(row_3_5, col_3_5) = P_all_ecef(col_4_6, col_4_6);
                    P_ecef(row_0_2, col_3_5) = P_all_ecef(row_0_2, col_4_6);
                    P_ecef(row_3_5, col_0_2) = P_all_ecef(col_4_6, col_0_2);
                    conv_state_matrix_ecef2i(P_ecef, P_i);
                    // Update position and velocity parts
                    d_P_new_new(row_0_2, col_0_2) = P_i(row_0_2, col_0_2);
                    d_P_new_new(row_4_6, row_4_6) = P_i(col_3_5, col_3_5);
                    d_P_new_new(row_0_2, row_4_6) = P_i(row_0_2, col_3_5);
                    d_P_new_new(row_4_6, row_0_2) = P_i(col_3_5, col_0_2);
#else
                    d_P_new_new = P_all_ecef;
#endif  // EKF_ECEF
                    // FIXME: validate results?
                    DLOG(INFO) << "Post measurement update\n";
                    DLOG(INFO) << "x: " << d_x_new_new;
                    DLOG(INFO) << "P: " << d_P_new_new;
                    // Resize again.
                    if (nv != n)
                        {
                            // res.resize(2 * n);
                            H.resize(2 * n, 8);
                            R.resize(2 * n, 2 * n);
                        }
                    arma::vec res_post = arma::zeros(n * 2, 1);
                    get_observation(rtk, obs, n, nav, res_post, H, R, d_x_new_new);
                    // TODO: residual check and revert x P.

                    // Dynamic noise scaling
                    // static const double alpha = 0.3;
                    // arma::mat Q_p = alpha * d_Q + (1 - alpha) * K * res * (K * res).t();
                    // arma::mat P_p = d_F * d_P_old_old * d_F.t() + Q_p;
                    // arma::mat P = d_F * d_P_old_old * d_F.t() + d_Q;
                    // const double beta = arma::trace(H * P_p * H.t()) / arma::trace(H * P * H.t());
                    // assert(beta > 0);
                    // const double q_dt = d_Q.at(3, 3);
                    // const double q_dtv = d_Q.at(7, 7);
                    // d_Q = sqrt(beta) * d_Q;
                    // d_Q.at(3, 3) = q_dt;
                    // d_Q.at(7, 7) = q_dtv;
                    // DLOG(INFO) << "Q: " << d_Q;
#else
                    // Without measurement updates
                    d_x_new_new = d_x_new_old;
                    d_P_new_new = d_P_new_old;
                    arma::vec x_new_new_ecef = d_x_new_new;
                    arma::mat P_all_ecef = d_P_new_new;
#endif  // EKF_DEBUG

                    // Update solution. Solution is in ECEF
                    rtk->sol.type = 0;
                    if (!rtk->opt.clock_bias_fixed)
                        {
                            rtk->sol.time = timeadd(obs[0].time, -d_x_new_new(3) / SPEED_OF_LIGHT_M_S);
                            rtk->sol.dtr[0] = d_x_new_new(3); /* receiver clock bias (m) */
                            // rtk->sol.dtr[1] = x[4] / SPEED_OF_LIGHT_M_S; /* glo-gps time offset (s) */
                            // rtk->sol.dtr[2] = x[5] / SPEED_OF_LIGHT_M_S; /* gal-gps time offset (s) */
                            // rtk->sol.dtr[3] = x[6] / SPEED_OF_LIGHT_M_S; /* bds-gps time offset (s) */
                        }
                    else
                        {
                            // Update time based on the fixed rx clock bias
                            rtk->sol.time = timeadd(obs[0].time, -rtk->sol.dtr[0]);
                        }
                    rtk->sol.dtr[5] = d_x_new_new(7); /* receiver clock drift (m/s) */

                    for (uint8_t j = 0; j < 3; j++)
                        {
                            rtk->sol.rr[j] = x_new_new_ecef.at(j);
                            rtk->sol.rr[j + 3] = x_new_new_ecef.at(j + 4);
                        }
                    for (uint8_t j = 0; j < 3; j++)
                        {
                            rtk->sol.qr[j] = static_cast<float>(P_all_ecef.at(j, j));
                        }
                    rtk->sol.qr[3] = static_cast<float>(P_all_ecef.at(0, 1));  /* cov xy */
                    rtk->sol.qr[4] = static_cast<float>(P_all_ecef.at(1, 2));  /* cov yz */
                    rtk->sol.qr[5] = static_cast<float>(P_all_ecef.at(0, 2));  /* cov zx */
                    rtk->sol.age = rtk->sol.ratio = 0.0;

                    // FIXME: set the sol stat properly.
                    rtk->sol.stat = SOLQ_FLOAT;
                    outsolstat(rtk);

                    // prepare data for next KF epoch
                    d_x_old_old = d_x_new_new;
                    d_P_old_old = d_P_new_new;

                    return 1;
                }
            catch (...)
                {
                    d_x_new_new = d_x_new_old;
                    this->reset_Kf();
                    return 0;
                }
        }
    return 0;
}


void Pvt_Ekf::get_states_Kf(arma::vec& x) const
{
    if (d_initialized)
        {
            x = d_x_new_new;
            // p = d_x_new_new.subvec(0, 2);
            // v = d_x_new_new.subvec(3, 5);
        }
}

void Pvt_Ekf::predict(const double tt)
{
    // Use RK4.
    const double dt2 = d_update_interval_s * 0.5;
    const double dt3 = d_update_interval_s * 0.5;
    const double dt4 = d_update_interval_s;

    arma::vec x_dot1 = state_derivative(d_x_old_old, tt);
    arma::mat F_dot1 = JacobiMatrix(d_x_old_old, tt) * d_F;
    arma::mat J_1 = JacobiMatrix(d_x_old_old, tt);
    arma::mat P_dot1 = J_1 * d_P_old_old + d_P_old_old * J_1.t() + d_Q;

    arma::vec x_dot2 = state_derivative(d_x_old_old + x_dot1 * dt2, tt + dt2);
    arma::mat F_dot2 = JacobiMatrix(d_x_old_old + x_dot1 * dt2, tt + dt2) * (d_F + F_dot1 * dt2);
    arma::mat J_2 = JacobiMatrix(d_x_old_old + x_dot1 * dt2, tt + dt2);
    arma::mat P_1 = d_P_old_old + P_dot1 * dt2;
    arma::mat P_dot2 = J_2 * P_1 + P_1 * J_2.t() + d_Q;

    arma::vec x_dot3 = state_derivative(d_x_old_old + x_dot2 * dt3, tt + dt3);
    arma::mat F_dot3 = JacobiMatrix(d_x_old_old + x_dot2 * dt3, tt + dt3) * (d_F + F_dot2 * dt3);
    arma::mat J_3 = JacobiMatrix(d_x_old_old + x_dot2 * dt3, tt + dt3);
    arma::mat P_2 = d_P_old_old + P_dot2 * dt3;
    arma::mat P_dot3 = J_3 * P_2 + P_2 * J_3.t() + d_Q;

    arma::vec x_dot4 = state_derivative(d_x_old_old + x_dot3 * dt4, tt + dt4);
    arma::mat F_dot4 = JacobiMatrix(d_x_old_old + x_dot3 * dt4, tt + dt4) * (d_F + F_dot3 * dt4);
    arma::mat J_4 = JacobiMatrix(d_x_old_old + x_dot3 * dt4, tt + dt4);
    arma::mat P_3 = d_P_old_old + P_dot3 * dt4;
    arma::mat P_dot4 = J_4 * P_3 + P_3 * J_4.t() + d_Q;

    d_x_new_old = d_x_old_old + d_update_interval_s * (x_dot1 + 2.0 * x_dot2 + 2.0 * x_dot3 + x_dot4) / 6.0;
    d_F += d_update_interval_s * (F_dot1 + 2.0 * F_dot2 + 2.0 * F_dot3 + F_dot4) / 6.0;
    d_P_new_old = d_P_old_old + d_update_interval_s * (P_dot1 + 2.0 * P_dot2 + 2.0 * P_dot3 + P_dot4) / 6.0;

    // TODO: It should be updated just before measurement update.
    // d_P_new_old = d_F * d_P_old_old * d_F.t() + d_Q;
}


arma::vec Pvt_Ekf::state_derivative(const arma::vec& x, const double tt) const
{
    static arma::vec o_8 = arma::zeros(8, 1);
    arma::vec dx_dt = o_8;
    // dx_dt.subvec(0, 3) = x.subvec(4, 7);
    // Because the drift accuracy may be bad, ignore the clock part.
    dx_dt.subvec(0, 2) = x.subvec(4, 6);

    double acc_m_s2[3] = {0};
    double pos_i[3];
    for (uint8_t i = 0; i < 3; i++)
        pos_i[i] = x.at(i);

    // FIXME: in ECEF, coriolis force should be considered.
    switch (d_state_frame_type)
        {
        case ECI:
            {
                Earth* earth_ptr = d_frame->GetEarth();
                earth_ptr->GetGravityAcceleration(pos_i, tt, 0, 0, acc_m_s2);
                break;
            }
        case MCI:
            {
                Moon* moon_ptr = d_frame->GetMoon();
                moon_ptr->GetGravityAcceleration(pos_i, tt, 0, 0, acc_m_s2);
                break;
            }
        default:
            break;
        }

    for (uint8_t i = 0; i < 3; i++)
        dx_dt.at(i + 4) = acc_m_s2[i];

    return dx_dt;
}


arma::mat Pvt_Ekf::JacobiMatrix(const arma::vec& x, const double tt) const
{
    static arma::mat O_88 = arma::zeros(8, 8);
    // Because the drift accuracy may be bad, ignore the clock part.
    // static arma::mat I_44 = arma::eye(4, 4);
    static arma::mat I_33 = arma::eye(3, 3);
    arma::mat F = O_88;
    // (r dt, v dtv)
    // F.submat(0, 4, 3, 7) = I_44;
    F.submat(0, 4, 2, 6) = I_33;
    // (v, r)
    const double r = arma::norm(x.subvec(0, 2));
    const double r3 = pow(r, 3);
    const double r5 = pow(r, 5);
    double mu_m3_s2;
    switch (d_state_frame_type)
        {
        case ECI:
            {
                Earth* earth_ptr = d_frame->GetEarth();
                mu_m3_s2 = earth_ptr->GetGravityConst() * 1e9;
                break;
            }
        case MCI:
            {
                Moon* moon_ptr = d_frame->GetMoon();
                mu_m3_s2 = moon_ptr->GetGravityConst() * 1e9;
                break;
            }
        default:
            break;
        }
    // Two-body gravity terms
    F.at(4, 0) = mu_m3_s2 / r3 * (3.0 * pow(x.at(0) / r, 2) - 1);
    F.at(4, 1) = mu_m3_s2 / r5 * 3.0 * x.at(0) * x.at(1);
    F.at(4, 2) = mu_m3_s2 / r5 * 3.0 * x.at(0) * x.at(2);

    F.at(5, 0) = mu_m3_s2 / r5 * 3.0 * x.at(1) * x.at(0);
    F.at(5, 1) = mu_m3_s2 / r3 * (3.0 * pow(x.at(1) / r, 2) - 1);
    F.at(5, 2) = mu_m3_s2 / r5 * 3.0 * x.at(1) * x.at(2);

    F.at(6, 0) = mu_m3_s2 / r5 * 3.0 * x.at(2) * x.at(0);
    F.at(6, 1) = mu_m3_s2 / r5 * 3.0 * x.at(2) * x.at(1);
    F.at(6, 2) = mu_m3_s2 / r3 * (3.0 * pow(x.at(2) / r, 2) - 1);

    // FIXME: add J2 term at least.

    return F;
}


void Pvt_Ekf::conv_states_ecef2i(const arma::vec& states_ecef, arma::vec& states_i)
{
    double x_ecef[6], x_i[6];
    for (uint8_t i = 0; i < 6; i++)
        x_ecef[i] = states_ecef.at(i);

    switch (d_state_frame_type)
        {
        case ECI:
        case MCI:
            {
                Earth* earth_ptr = d_frame->GetEarth();
                // ECEF to ECI (J2000)
                earth_ptr->ConvFixed2I(x_ecef, x_i, d_tt);
                if (d_state_frame_type == ECI)
                    break;
                Moon* moon_ptr = d_frame->GetMoon();
                moon_ptr->Update(d_tt);
                const double* moon_pos_i = moon_ptr->GetPositionI();
                const double* moon_vel_i = moon_ptr->GetVelocityI();
                // ECI (J2000) to MCI (J2000)
                for (uint8_t i = 0; i < 3; i++)
                    {
                        x_i[i] -= moon_pos_i[i];
                        x_i[i + 3] -= moon_vel_i[i];
                    }
                break;
            }
        default:
            break;
        }

    for (uint8_t i = 0; i < 6; i++)
        states_i.at(i) = x_i[i];
}


void Pvt_Ekf::conv_states_i2ecef(const arma::vec& states_i, arma::vec& states_ecef)
{
    double x_ecef[6], x_i[6];
    for (uint8_t i = 0; i < 6; i++)
        x_i[i] = states_i.at(i);

    if (d_state_frame_type == ECI || d_state_frame_type == MCI)
        {
            if (d_state_frame_type == MCI)
                {
                    Moon* moon_ptr = d_frame->GetMoon();
                    moon_ptr->Update(d_tt);
                    const double* moon_pos_i = moon_ptr->GetPositionI();
                    const double* moon_vel_i = moon_ptr->GetVelocityI();
                    // MCI (J2000) to ECI (J2000)
                    for (uint8_t i = 0; i < 3; i++)
                        {
                            x_i[i] += moon_pos_i[i];
                            x_i[i + 3] += moon_vel_i[i];
                        }
                }
            Earth* earth_ptr = d_frame->GetEarth();
            // ECI (J2000) to ECEF
            earth_ptr->ConvI2Fixed(x_i, x_ecef, d_tt);
        }

    for (uint8_t i = 0; i < 6; i++)
        states_ecef.at(i) = x_ecef[i];
}


void Pvt_Ekf::conv_state_matrix_i2ecef(const arma::mat& mat_i, arma::mat& mat_ecef)
{
    double A_ecef[6 * 6], A_i[6 * 6];
    for (uint8_t i = 0; i < 6; i++)
        {
            for (uint8_t j = 0; j < 6; j++)
                A_i[i * 6 + j] = mat_i.at(i, j);
        }

    Earth* earth_ptr = d_frame->GetEarth();
    // J2000 to ECEF
    earth_ptr->ConvMatI2Fixed(A_i, A_ecef, d_tt);

    for (uint8_t i = 0; i < 6; i++)
        {
            for (uint8_t j = 0; j < 6; j++)
                mat_ecef.at(i, j) = A_ecef[i * 6 + j];
        }
}


void Pvt_Ekf::conv_state_matrix_ecef2i(const arma::mat& mat_ecef, arma::mat& mat_i)
{
    double A_ecef[6 * 6], A_i[6 * 6];
    for (uint8_t i = 0; i < 6; i++)
        {
            for (uint8_t j = 0; j < 6; j++)
                A_ecef[i * 6 + j] = mat_ecef.at(i, j);
        }

    Earth* earth_ptr = d_frame->GetEarth();
    // J2000 to ECEF
    earth_ptr->ConvMatFixed2I(A_ecef, A_i, d_tt);

    for (uint8_t i = 0; i < 6; i++)
        {
            for (uint8_t j = 0; j < 6; j++)
                mat_i.at(i, j) = A_i[i * 6 + j];
        }
}


uint8_t Pvt_Ekf::get_observation(rtk_t* rtk, const obsd_t* obs, int n, const nav_t* nav, arma::vec& res, arma::mat& H, arma::mat& R, const arma::vec& states)
{
    prcopt_t opt = rtk->opt;
    double *rs;
    double *dts;
    double *vare;
    double *azel;
    double *resp;
    int i, j;
    std::vector<int> vsat(MAXOBS, 0);
    std::vector<int> svh(MAXOBS, 0);

    rtk->sol.stat = SOLQ_NONE;

    if (n <= 0)
        return 0;

    rtk->sol.time = obs[0].time;

    rs = mat(6, n);
    dts = mat(2, n);
    vare = mat(1, n);
    azel = zeros(2, n);
    resp = mat(1, n);

    // TODO: set ion and trop option

    /* satellite positions, velocities and clocks */
    satposs(rtk->sol.time, obs, n, nav, opt.sateph, rs, dts, vare, svh.data());

    // Convert positions, velocities to ECEF to calculate observations.
    arma::vec x_i = arma::join_vert(states.subvec(0, 2), states.subvec(4, 6));
    static arma::vec o_6 = arma::zeros(6, 1);
#ifdef EKF_ECEF
    arma::vec x_ecef = x_i;
#else
    arma::vec x_ecef = o_6;
    conv_states_i2ecef(x_i, x_ecef);
#endif // EKF_ECEF

    // Pseudorange residual
    double *x_p = new double[NX]();
    int nx = NX;
    if (opt.clock_bias_fixed)
        {
            nx = 3; // only position
            x_p[3] = rtk->sol.dtr[0];
            x_p[4] = rtk->sol.dtr[1];
            x_p[5] = rtk->sol.dtr[2];
            x_p[6] = rtk->sol.dtr[3];
        }
    else
        {
            x_p[3] = states.at(3);
        }
    x_p[0] = x_ecef.at(0);
    x_p[1] = x_ecef.at(1);
    x_p[2] = x_ecef.at(2);

    // double *dx = new double[nx];
    // double *Q = new double[nx * nx];
    double *v_P;
    double *H_P;
    double *var;
    int ns;
    v_P = mat(n + 4, 1);
    H_P = mat(nx, n + 4);
    var = mat(n + 4, 1);
    // NOTE: rescode assumes the states are in ECEF
    int nv = rescode(0, obs, n, rs, dts, vare, svh.data(), nav, x_p, &opt, v_P, H_P, var, azel, vsat.data(), resp, &ns);
    rtk->sol.ns = static_cast<unsigned char>(ns);

    // NOTE: nv includes additional num to avoid rank-deficient.
    // valid sat check
    const uint8_t num_state_m = (nx == 3) ? 3 : 4;
    if (ns < num_state_m)
        return 0;

    for (i = 0; i < ns; i++)
        {
            res.at(i) = v_P[i];
            R.at(i, i) = var[i];
            for (j = 0; j < num_state_m; j++)
                H.at(i, j) = H_P[i * nx + j];
        }

    // Doppler residual
    double x_v[4];
    x_v[3] = states.at(7);
    for (i = 0; i < 3; i++)
        x_v[i] = x_ecef.at(i + 3);

    // TODO: estimation of dtv by aowr
    double *v_D;
    double *H_D;
    v_D = mat(n, 1);
    H_D = mat(4, n);

    // NOTE: resdop assumes the states are in ECEF
    nv = resdop(obs, n, rs, dts, nav, x_p, x_v, azel, vsat.data(), v_D, H_D);
    const double measures_vel_var_ms2 = pow(d_measures_vel_sd_ms, 2);
    for (i = 0; i < nv; i++)
        {
            res.at(i + nv) = v_D[i];
            R.at(i + nv, i + nv) = measures_vel_var_ms2;
            for (j = 0; j < 4; j++)
                H.at(i + nv, j + num_state_m) = H_D[j + i * 4];

            // dR_dot/dr
            arma::vec sat_rv = {rs[i * 6], rs[i * 6 + 1], rs[i * 6 + 2], rs[i * 6 + 3], rs[i * 6 + 4], rs[i * 6 + 5]};
            arma::vec los = x_ecef - sat_rv;
            const double dev_r3 = 1.0 / pow(arma::norm(los), 3);
            const double dx2 = pow(los.at(0), 2);
            const double dy2 = pow(los.at(1), 2);
            const double dz2 = pow(los.at(2), 2);
            H.at(i + nv, 0) = dev_r3 * (los.at(4) * (dy2 + dz2) - los.at(5) * los.at(0) * los.at(1) - los.at(6) * los.at(0) * los.at(2));
            H.at(i + nv, 1) = dev_r3 * (-los.at(4) * los.at(0) * los.at(1) + los.at(5) * (dx2 + dz2) - los.at(6) * los.at(1) * los.at(2));
            H.at(i + nv, 2) = dev_r3 * (-los.at(4) * los.at(0) * los.at(2) - los.at(5) * los.at(1) * los.at(2) + los.at(6) * (dx2 + dy2));
        }

    DLOG(INFO) << "res: " << res;
    DLOG(INFO) << "H: " << H;
    DLOG(INFO) << "R: " << R;

    return nv;
}