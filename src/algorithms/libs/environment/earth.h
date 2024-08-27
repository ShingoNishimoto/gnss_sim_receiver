/*!
 * \file earth.h
 * \brief Class to manage the Earth information
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

#ifndef GNSS_SDR_EARTH_H
#define GNSS_SDR_EARTH_H

#include "celestial_body.h"

class Earth: public CelestialBody
{
public:
    Earth(double initial_julian_day);
    ~Earth();

    // void Update(double julian_date) override;

private:
    // bool center_body_;
    // double rotation_rate_rad_s_;  // around z axis
    // double gravity_constant_;  // km3/s2
    // double radius_m_;
    // double initial_julian_day_;

    // TimeSystem time_system_;

    // double position_i_m_[3];
    // double velocity_i_m_s_[3];
    // double dcm_i_to_fixed_[3][3];  // for attitude
    // double initial_position_i_m_[3];
    // double initial_velocity_i_m_s_[3];

    // void rotation_matrix_around_z(double rotation_rad, double mat[3][3]);
};

#endif  // GNSS_SDR_EARTH_H
