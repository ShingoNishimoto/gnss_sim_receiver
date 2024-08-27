/*!
 * \file moon.cc
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

#include "moon.h"

Moon::Moon(double initial_julian_day):
CelestialBody(initial_julian_day)
{
    // NOTE: from the spice data base
    // FIXME:
    rotation_rate_rad_s_ = 2 * M_PI / (24 * 3600.0);
    gravity_constant_ = 4.902800066163796e3;
    radius_km_ = 1737.4;
}

Moon::~Moon()
{
}

void Moon::Update(double julian_date)
{
    // Position and velocity update based on two-body

}
