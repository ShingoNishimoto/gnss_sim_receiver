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
#include <cstdint>
#include <cmath>

CelestialBody::CelestialBody(double initial_julian_day):
rotation_rate_rad_s_(0),
gravity_constant_(0),
radius_km_(0),
initial_julian_day_(initial_julian_day),
time_system_(TimeSystem())
{
    Update(initial_julian_day);
}

CelestialBody::~CelestialBody()
{}

void CelestialBody::Update(double julian_day)
{
    double delta_t_sec = time_system_.ConvDayToSec(julian_day - initial_julian_day_);
    double rotation_angle_rad = rotation_rate_rad_s_ * delta_t_sec;
    // Just considering rotation around z axis
    rotation_matrix_around_z(rotation_angle_rad, dcm_fixed_to_i_);

    // Transpose
    for (uint8_t i = 0; i < 3; i++)
    {
        for (uint8_t j = 0; j < 3; j++)
        {
            dcm_i_to_fixed_[3 * j + i] = dcm_fixed_to_i_[3 * i + j];
        }
    }
}

void CelestialBody::rotation_matrix_around_z(double rotation_rad, double mat[9])
{
    // TODO: it is too rough, probably the accuracy is not enough.
    mat[3 * 0 + 0] = cos(rotation_rad);
    mat[3 * 0 + 1] = sin(rotation_rad);
    mat[3 * 0 + 2] = 0;
    mat[3 * 1 + 0] = -sin(rotation_rad);
    mat[3 * 1 + 1] = cos(rotation_rad);
    mat[3 * 1 + 2] = 0;
    mat[3 * 2 + 0] = 0;
    mat[3 * 2 + 1] = 0;
    mat[3 * 2 + 2] = 1;
}
