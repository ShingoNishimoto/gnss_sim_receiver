/*!
 * \file time_system.h
 * \brief Class to manage the time system
 * \author Shingo Nishimoto
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

#include "time_system.h"
#include <chrono>
#include <cmath>

TimeSystem::TimeSystem()
{}

TimeSystem::~TimeSystem()
{}

double TimeSystem::ConvGPSTimeToJulianDate(gtime_t gps_time)
{
    // using namespace std::chrono;
    // date::days gps_days = duration_cast<date::days>(seconds(gps_time.time));
    double fractional_day = (gps_time.time + gps_time.sec) / sec_1_day_;
    double julian_day = gps_epoch_julian_ + fractional_day;

    return julian_day;
}

gtime_t TimeSystem::ConvJulianDateToGPSTime(double julian_day)
{
    double gps_sec = (julian_day - gps_epoch_julian_) * sec_1_day_;
    gtime_t gps_time;
    gps_time.time = std::floor(gps_sec);
    gps_time.sec = gps_sec - gps_time.time;

    return gps_time;
}
