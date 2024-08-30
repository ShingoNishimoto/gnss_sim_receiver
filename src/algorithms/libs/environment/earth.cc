/*!
 * \file earth.cc
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

#include "earth.h"
#include <cstdint>

Earth::Earth(double J2000_julian):
CelestialBody(J2000_julian)
{
    // NOTE: from the spice data base
    rotation_rate_rad_s_ = 4.17807356e-3;  // FIXME: other axis also has the angular velocity
    gravity_constant_ = 3.986004354360959e5;
    radius_km_ = 6378.1366;
}

Earth::~Earth()
{
}

void Earth::Update(double julian_day)
{
    CelestialBody::Update(julian_day);
}

extern "C" {
    // Earth* EarthInit(int initial_gps_week, double initial_gps_sec)
    // {
    //     TimeSystem::gpstime_t gps_time;
    //     gps_time.week = initial_gps_week;
    //     gps_time.sec = initial_gps_sec;
    //     TimeSystem time_system = TimeSystem();
    //     double initial_julian_day = time_system.ConvGPSTimeToJulianDate(gps_time);
    //     return new Earth(initial_julian_day);
    // }

    Earth* EarthInit(double julian_day)
    {
        return new Earth(julian_day);
    }

    double EarthGravityConst(Earth* earth)
    {
        return earth->GetGravityConst();
    }

    void EarthDestroy(Earth* earth)
    {
        delete earth;
    }
}