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
class Earth: public CelestialBody
{
public:
    Earth(double initial_tt);
    virtual ~Earth();

    void Update(double tt) override;

private:
};
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct earth earth;

// Earth* EarthInit(int initial_gps_week, double initial_gps_sec);
earth* EarthInit(double tt);
double EarthGravityConst(earth* earth);
void EarthDestroy(earth* earth);

#ifdef __cplusplus
}
#endif

#endif  // GNSS_SDR_EARTH_H
