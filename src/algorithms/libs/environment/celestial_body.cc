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
#include <iostream>
#include <SpiceUsr.h>  // for SPICE

CelestialBody::CelestialBody(double initial_julian_day):
rotation_rate_rad_s_(0),
gravity_constant_(0),
radius_km_(0),
initial_julian_day_(initial_julian_day),
time_system_(TimeSystem()),
inertial_frame_("J2000"),
fixed_frame_("ITRF93")
{
    // Load SPICE kernel
    const char *home_dir = getenv("HOME");
    if (home_dir == nullptr)
        {
            std::cerr << "HOME environment variable not set." << std::endl;
        }
    furnsh_c((std::string(home_dir) + "/work/cspice/generic_kernels/spk/planets/de430.bsp").c_str());
    furnsh_c((std::string(home_dir) + "/work/cspice/generic_kernels/lsk/naif0012.tls").c_str());
    furnsh_c((std::string(home_dir) + "/work/cspice/generic_kernels/pck/earth_070425_370426_predict.bpc").c_str());
    furnsh_c((std::string(home_dir) + "/work/cspice/generic_kernels/pck/earth_fixed.tf").c_str());
    furnsh_c((std::string(home_dir) + "/work/cspice/generic_kernels/pck/gm_de431.tpc").c_str());
    furnsh_c((std::string(home_dir) + "/work/cspice/generic_kernels/pck/pck00010.tpc").c_str());

    // Update(initial_julian_day);
}

CelestialBody::~CelestialBody()
{
    // unload SPICE kernel
    const char *home_dir = getenv("HOME");
    if (home_dir == nullptr)
        {
            std::cerr << "HOME environment variable not set." << std::endl;
        }
    unload_c((std::string(home_dir) + "/work/cspice/generic_kernels/spk/planets/de430.bsp").c_str());
    unload_c((std::string(home_dir) + "/work/cspice/generic_kernels/lsk/naif0012.tls").c_str());
    unload_c((std::string(home_dir) + "/work/cspice/generic_kernels/pck/earth_070425_370426_predict.bpc").c_str());
    unload_c((std::string(home_dir) + "/work/cspice/generic_kernels/pck/earth_fixed.tf").c_str());
    unload_c((std::string(home_dir) + "/work/cspice/generic_kernels/pck/gm_de431.tpc").c_str());
    unload_c((std::string(home_dir) + "/work/cspice/generic_kernels/pck/pck00010.tpc").c_str());
}

void CelestialBody::Update(double julian_day)
{
    // double delta_t_sec = time_system_.ConvDayToSec(julian_day - initial_julian_day_);
    // double rotation_angle_rad = rotation_rate_rad_s_ * delta_t_sec;
    // // Just considering rotation around z axis
    // rotation_matrix_around_z(rotation_angle_rad, dcm_fixed_to_i_);

    double et = time_system_.ConvJulianDateToJ2000(julian_day);
    double rotate[3][3];
    pxform_c(inertial_frame_.c_str(), fixed_frame_.c_str(), et, rotate);
    for (uint8_t i = 0; i < 3; i++)
        {
            for (uint8_t j = 0; j < 3; j++)
                dcm_i_to_fixed_[3 * i + j] = rotate[i][j];
        }

    // Transpose
    for (uint8_t i = 0; i < 3; i++)
    {
        for (uint8_t j = 0; j < 3; j++)
        {
            dcm_fixed_to_i_[3 * j + i] = dcm_i_to_fixed_[3 * i + j];
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
