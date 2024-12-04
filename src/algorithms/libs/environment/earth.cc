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
#include <SpiceUsr.h>  // for SPICE

Earth::Earth(double initial_tt):
CelestialBody(initial_tt)
{
    // NOTE: from the spice data base
    rotation_rate_rad_s_ = 4.17807356e-3;  // FIXME: other axis also has the angular velocity
    gravity_constant_ = 3.986004354360959e5;
    radius_km_ = 6378.1366;
    inertial_frame_ = "J2000";
    fixed_frame_ = "ITRF93";  // IAU_EARTH or ITRF93
}

Earth::~Earth()
{
}

void Earth::Update(double tt)
{
    CelestialBody::Update(tt);
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

    Earth* EarthInit(double tt)
    {
        return new Earth(tt);
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