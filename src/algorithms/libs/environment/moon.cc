/*!
 * \file moon.cc
 * \brief Class to manage the Moon information
 * \author Shingo Nishimoto
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "moon.h"
#include "rtklib_rtkcmn.h"  // for matrix function

#include <iostream>

Moon::Moon(double initial_julian_day):
CelestialBody(initial_julian_day)
{
    // NOTE: from the spice data base
    rotation_rate_rad_s_ = 1.41426064e-4;  // TODO: other axis also has the angular velocity
    gravity_constant_ = 4.902800066163796e3;
    radius_km_ = 1737.4;

    // FIXME: use spice library here.
    double et_J2000 = 757339269.1839061;
    double r[3] = {-3.67980873e5, 1.42721025e5, 8.93144235e4};  // km
    double v[3] = {-0.409612122, -0.77956548, -0.402716285};  // km/s
    InitOrbitElement(et_J2000, r, v);
}

Moon::~Moon()
{
}

void Moon::Update(double julian_date)
{
    CelestialBody::Update(julian_date);
    // Position and velocity update based on two-body
    UpdateStates(julian_date);
}

void Moon::InitOrbitElement(double et_J2000, double r_ini[3], double v_ini[3])
{
    // angular momentum
    cross3(r_ini, v_ini, angular_momentum_);
    // energy
    double r_norm = norm_rtk(r_ini, 3);
    double v_norm = norm_rtk(v_ini, 3);
    energy_ = 0.5 * pow(v_norm, 2) - gravity_constant_ / r_norm;
    // periapsis vector
    cross3(v_ini, angular_momentum_, periapsis_vector_);
    double r_dir[3];
    if (!normv3(r_ini, r_dir))
    {
        std::cerr << "Something wrong!" << std::endl;
    }
    periapsis_vector_[0] -= gravity_constant_ * r_dir[0];
    periapsis_vector_[1] -= gravity_constant_ * r_dir[1];
    periapsis_vector_[2] -= gravity_constant_ * r_dir[2];

    a_ = -gravity_constant_ / (2 * energy_);
    double p = pow(norm_rtk(angular_momentum_, 3), 2) / gravity_constant_;
    e_ = pow(norm_rtk(periapsis_vector_, 3), 2) / gravity_constant_;
    // double rp_norm = p / (1 + e_);
    // double rp[3] = {rp_norm * periapsis_vector_[0],
    //                 rp_norm * periapsis_vector_[1],
    //                 rp_norm * periapsis_vector_[2]};
    double cos_theta = (p - r_norm) / (e_ * r_norm);
    // double true_anomaly = acos(cos_theta);  // rad
    double E = acos((e_ + cos_theta) / (1 + e_ * cos_theta));
    n_ = sqrt(gravity_constant_ / pow(a_, 3));
    T_ = 2 * M_PI / n_;
    double t_tp = (E - e_ * sin(E)) / n_;  // [sec]
    tp_J2000_ = et_J2000 - t_tp;
    //
}

void Moon::UpdateStates(double julian_date)
{
    const double t_J2000 = time_system_.ConvJulianDateToJ2000(julian_date);
    double t_tp_res = t_J2000 - tp_J2000_ - T_ * std::floor((t_J2000 - tp_J2000_) / T_);
    double M = n_ * t_tp_res;
    // Solve the Kepler equation
    double E = M_PI; // initial value
    while (std::fabs(KeplerEq(E, M)) > 0.01 * M_PI)
    {
        E = E - KeplerEq(E, M) / KeplerEqDot(E);
    }

    double true_anomaly = acos((cos(E) - e_) / (1 - e_ * cos(E)));
    double r = a_ * (1 - e_ * cos(E));
    double pos_inplane_m[3] = {r * cos(true_anomaly) * 1000, r * sin(true_anomaly) * 1000, 0};
    double v = sqrt(gravity_constant_ * (2 / r - 1 / a_));
    double gamma = asin(norm_rtk(angular_momentum_, 3) / (r * v));
    double vel_inplane_m_s[3] = {v * cos(true_anomaly + gamma) * 1000, v * sin(true_anomaly + gamma) * 1000, 0};

    double p[3], q[3], w[3];
    if (!normv3(periapsis_vector_, p) || !normv3(angular_momentum_, w))
    {
        std::cerr << "Something wrong!" << std::endl;
        return;
    }
    cross3(w, p, q);

    double dcm_inplane_to_eci[9] = {
        p[0], q[0], w[0],
        p[1], q[1], w[1],
        p[2], q[2], w[2]};
    matmul("N", 3, 1, 3, 1, dcm_inplane_to_eci, pos_inplane_m, 0, position_i_m_);
    matmul("N", 3, 1, 3, 1, dcm_inplane_to_eci, vel_inplane_m_s, 0, velocity_i_m_s_);
}
