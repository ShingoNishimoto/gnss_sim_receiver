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
// #include <Eigen/Dense>

#ifdef __cplusplus
#include <string>

class CelestialBody
{
public:
    CelestialBody(double initial_tt);
    virtual ~CelestialBody();

    // TODO: for time, which system should be used? julian day, gregorian date, gps time?
    virtual void Update(double tt);
    inline const double* GetPositionI(void) const { return position_i_m_; };
    inline const double* GetVelocityI(void) const { return velocity_i_m_s_; };
    inline double GetRadiusKm(void) const { return radius_km_; };
    inline double GetGravityConst(void) const { return gravity_constant_; };
    inline double* GetDcmI2Fixed(double tt) {
        Update(tt);
        return dcm_i_to_fixed_;
    };
    inline double* GetDcmFixed2I(double tt) {
        Update(tt);
        return dcm_fixed_to_i_;
    };
    void GetGravityAcceleration(const double pos_i_m[3], double tt, uint16_t n, uint16_t m, double* acc_ms2) const;
    void ConvI2Fixed(const double* x_i, double* x_f, double tt) const;
    void ConvFixed2I(const double* x_f, double* x_i, double tt) const;
    void ConvMatI2Fixed(const double* A_i, double* A_f, double tt) const;
    void ConvMatFixed2I(const double* A_f, double* A_i, double tt) const;

protected:
    double rotation_rate_rad_s_;  // around z axis
    double gravity_constant_;  // km3/s2
    double radius_km_;
    double initial_tt_;
    TimeSystem time_system_;

    // NOTE: position and velocity are represented in ECI (J2000)
    double position_i_m_[3];
    double velocity_i_m_s_[3];
    double dcm_i_to_fixed_[3 * 3];  // for attitude
    double dcm_fixed_to_i_[3 * 3];  // for attitude
    std::string inertial_frame_;
    std::string fixed_frame_;
    std::string body_name_;
    // double initial_position_i_m_[3];
    // double initial_velocity_i_m_s_[3];
private:
    void RotationMatrixAroundZ(double rotation_rad, double mat[9]);
    // void MatMultiply(const double* A, const double* B, double* C, uint16_t n, uint16_t k, uint16_t m);
};
#endif

// #ifdef __cplusplus
// extern "C" {
// #endif

// typedef struct CelestialBody CelestialBody;

// #ifdef __cplusplus
// }
// #endif

#endif  // GNSS_SDR_CELESTIAL_BODY_H
