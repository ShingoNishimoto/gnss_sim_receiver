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

class Earth: public CelestialBody
{
public:
    Earth(double initial_julian_day);
    ~Earth();

    // void Update(double julian_date) override;

private:
};

#endif  // GNSS_SDR_EARTH_H
