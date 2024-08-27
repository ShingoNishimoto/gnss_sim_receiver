/*!
 * \file celestial_body.h
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

#ifndef GNSS_SDR_CELESTIAL_BODY_H
#define GNSS_SDR_CELESTIAL_BODY_H

#include "time_system.h"
// FIXME: modify the cmake file.
// #include <Eigen/Dense>
#include <cstdint>
#include <vector>

class CelestialBody
{
public:
    CelestialBody();
    virtual ~CelestialBody();

    // TODO: for time, which system should be used? julian day, gregorian date, gps time?
    void Update(double julian_date);
    inline double* GetPositionI(void) { return position_i_m_; };
    inline double* GetVelocityI(void) { return velocity_i_m_s_; };

private:
    double rotation_rate_rad_s_;  // around z axis
    double revolution_rate_rad_s_;
    double gravity_constant_;
    double initial_julian_day_;

    TimeSystem time_system_;

    // TODO: matrix or vector handling...
    double position_i_m_[3];
    double velocity_i_m_s_[3];
    double dcm_i_to_fixed_[3][3];  // for attitude
    double initial_position_i_m_[3];
    double initial_velocity_i_m_s_[3];
    double initial_dcm_i_to_fixed_[3][3];

    void rotation_matrix_around_z(double rotation_rad, double mat[3][3]);
};

#endif  // GNSS_SDR_CELESTIAL_BODY_H
