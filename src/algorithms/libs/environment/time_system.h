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
// #include "rtklib.h"
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct TimeSystem TimeSystem;

TimeSystem* TimeSystemInit(void);
double ConvGPSTimeToTt(TimeSystem* time_system, int gps_week, double gps_sec);
double GetJ2000EpochJulianDay(TimeSystem* time_system);
void TimeSystemDestroy(TimeSystem* time_system);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
class TimeSystem
{
public:
    TimeSystem();
    virtual ~TimeSystem();

    // NOTE: copy from rtklib.h file
    typedef struct
    {                /* time struct */
        time_t time; /* time (s) expressed by standard time_t */
        double sec;  /* fraction of second under 1 s */
    } gtime_t;

    // NOTE: from gpssim.h file
    typedef struct
    {
        int week;  /*!< GPS week number (since January 1980) */
        double sec;  /*!< second inside the GPS \a week */
    } gpstime_t;

    inline double ConvTtToJulianDate(double tt) const { return ConvSecToDay(tt) + J2000_julian_; };
    inline double ConvJulianDateToTt(double julian_day) const { return ConvDayToSec(julian_day - J2000_julian_); };
    // gtime_t uses the utc system
    inline double ConvUtcToTt(gtime_t utc) const { return static_cast<double>(utc.time + utc.sec) + utc_to_tai_ + tai_to_tt_; };
    // inline double ConvUtcToJulianDate(gtime_t utc) const { return ConvTtToJulianDate(ConvUtcToTt(utc)); };
    gtime_t ConvTtToUtc(double tt);
    // gtime_t ConvJulianDateToUtc(double julian_day);
    double ConvGPSTimeToTt(gpstime_t gps_time);
    // double ConvGPSTimeToJulianDate(gpstime_t gps_time);
    gpstime_t ConvTtToGPSTime(double tt);
    // gpstime_t ConvJulianDateToGPSTime(double julian_day);
    inline double ConvDayToSec(const double day) const { return day * sec_1_day_; };
    inline double ConvSecToDay(const double sec) const { return sec / sec_1_day_; };
    // double ConvGPSTimeToGregorianDate(gtime_t gps_time);
    // gtime_t ConvGregorianDateToGPSTime(double gregorian_date);

    inline double GetJ2000EpochJulianDay(void) const { return J2000_julian_; };

private:
    // date library provides the basic function.
    // double julian_date_;
    // date::sys_days gregorian_date_;
    // gtime_t gps_time_;
    const double J2000_julian_ = 2451545.0;
    const double gps_epoch_julian_ = 2444244.5;  // January 6, 1980
    const double utc_epoch_julian_ = 2440587.5;  // January 1, 1970
    const double utc_to_gpst_ = 18.0;
    const double gpst_to_tai_ = 19.0;  // TAI: Atomic time
    const double utc_to_tai_ = utc_to_gpst_ + gpst_to_tai_;
    const double tai_to_tt_ = 32.184;  // TT: Terrestrial time (sec from J2000 epoch)
    // const date::sys_days gps_epoch_ = 1980_y/January/6;
    const double sec_1_day_ = 86400.0;
};
#endif

#endif  // GNSS_SDR_TIME_SYSTEM_H
