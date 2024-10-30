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

double TimeSystem::ConvUtcToJulianDate(gtime_t utc)
{
    double fractional_day = (utc.time + utc.sec) / sec_1_day_;
    double julian_day = utc_epoch_julian_ + fractional_day;

    return julian_day;
}

TimeSystem::gtime_t TimeSystem::ConvJulianDateToUtc(double julian_day)
{
    double utc_sec = (julian_day - utc_epoch_julian_) * sec_1_day_;
    gtime_t utc;
    utc.time = std::floor(utc_sec);
    utc.sec = utc_sec - utc.time;

    return utc;
}

double TimeSystem::ConvGPSTimeToJulianDate(gpstime_t gps_time)
{
    double fractional_day = gps_time.week * 7 + (gps_time.sec) / sec_1_day_;
    double julian_day = gps_epoch_julian_ + fractional_day;

    return julian_day;
}

TimeSystem::gpstime_t TimeSystem::ConvJulianDateToGPSTime(double julian_day)
{
    double gps_week = (julian_day - gps_epoch_julian_) / 7.0;
    gpstime_t gps_time;
    gps_time.week = std::floor(gps_week);
    gps_time.sec = (gps_week - gps_time.week) * 7 * sec_1_day_;

    return gps_time;
}


extern "C" {
    TimeSystem* TimeSystemInit(void)
    {
        return new TimeSystem();
    }

    double ConvGPSTimeToJulianDate(TimeSystem* time_system, int gps_week, double gps_sec)
    {
        TimeSystem::gpstime_t gps_time;
        gps_time.week = gps_week;
        gps_time.sec = gps_sec;
        return time_system->ConvGPSTimeToJulianDate(gps_time);
    }

    double GetJ2000EpochJulianDay(TimeSystem* time_system)
    {
        return time_system->GetJ2000EpochJulianDay();
    }

    void TimeSystemDestroy(TimeSystem* time_system)
    {
        delete time_system;
    }
}