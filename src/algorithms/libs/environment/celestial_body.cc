/*!
 * \file celestial_body.cc
 * \brief Class to manage the celestial information
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

#include "celestial_body.h"

CelestialBody::CelestialBody(double initial_julian_day):
initial_julian_day_(initial_julian_day),
time_system_(TimeSystem())
{
}

void CelestialBody::Update(double julian_date)
{
    double delta_t_sec = time_system_.ConvDayToSec(julian_date - initial_julian_day_);
    double rotation_angle_rad = rotation_rate_rad_s_ * delta_t_sec;
    // Just considering rotation around z axis
    rotation_matrix_around_z(rotation_angle_rad, dcm_i_to_fixed_);

    // Position and velocity update based on two-body
    if (!center_body_)
    {

    }
}

void CelestialBody::rotation_matrix_around_z(double rotation_rad, double mat[3][3])
{
    // TODO: it is too rough, probably the accuracy is not enough.
    mat[0][0] = cos(rotation_rad);
    mat[0][1] = sin(rotation_rad);
    mat[0][2] = 0;
    mat[1][0] = -sin(rotation_rad);
    mat[1][1] = cos(rotation_rad);
    mat[1][2] = 0;
    mat[2][0] = 0;
    mat[2][1] = 0;
    mat[2][2] = 1;
}
