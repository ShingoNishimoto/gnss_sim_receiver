/*!
 * \file moon.h
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

#ifndef GNSS_SDR_MOON_H
#define GNSS_SDR_MOON_H

#include "celestial_body.h"

class Moon: public CelestialBody
{
public:
    Moon(double initial_julian_day);
    ~Moon();

    // TODO: for time, which system should be used? julian day, gregorian date, gps time?
    void Update(double julian_date) override;

private:
    double angular_momentum_[3];
    double energy_;
    double periapsis_vector_[3];
    double a_;  // semi-major axis
    double e_;  // eccentricity
    double tp_J2000_;
    double n_;  // Mean motion
    double T_;  // period

    void InitOrbitElement(double et_J2000, double r_ini[3], double v_ini[3]);
    void UpdateStates(double julian_date);
    inline double KeplerEq(double E, double M) { return E - e_ * sin(E) - M; };
    inline double KeplerEqDot(double E) { return 1 - e_ * cos(E); };
};

#endif  // GNSS_SDR_MOON_H
