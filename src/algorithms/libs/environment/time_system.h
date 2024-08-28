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

#ifndef GNSS_SDR_TIME_SYSTEM_H
#define GNSS_SDR_TIME_SYSTEM_H

// #include "date/date.h"
// #include "date/tz.h"
#include "rtklib.h"
#include <cstdint>

class TimeSystem
{
public:
    TimeSystem();
    ~TimeSystem();

    inline double ConvJ2000ToJulianDate(double t_J2000) const { return ConvSecToDay(t_J2000) + J2000_julian_; };
    inline double ConvJulianDateToJ2000(double julian_day) const { return ConvDayToSec(julian_day - J2000_julian_); };
    double ConvGPSTimeToJulianDate(gtime_t gps_time);
    gtime_t ConvJulianDateToGPSTime(double julian_day);
    inline double ConvDayToSec(const double day) const { return day * sec_1_day_; };
    inline double ConvSecToDay(const double sec) const { return sec / sec_1_day_; };
    // double ConvGPSTimeToGregorianDate(gtime_t gps_time);
    // gtime_t ConvGregorianDateToGPSTime(double gregorian_date);

private:
    // date library provides the basic function.
    // double julian_date_;
    // date::sys_days gregorian_date_;
    // TODO: UTC?
    // gtime_t gps_time_;
    const double J2000_julian_ = 2451545.0;
    const double gps_epoch_julian_ = 2444244.5; // January 6, 1980
    // const date::sys_days gps_epoch_ = 1980_y/January/6;
    const double sec_1_day_ = 86400.0;
};

#endif  // GNSS_SDR_TIME_SYSTEM_H
