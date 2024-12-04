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

#ifdef __cplusplus
extern "C" {
#endif

typedef struct Earth Earth;

// Earth* EarthInit(int initial_gps_week, double initial_gps_sec);
Earth* EarthInit(double tt);
double EarthGravityConst(Earth* earth);
void EarthDestroy(Earth* earth);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
class Earth: public CelestialBody
{
public:
    Earth(double J2000_julian);
    virtual ~Earth();

    void Update(double tt) override;

private:
};
#endif

#endif  // GNSS_SDR_EARTH_H
