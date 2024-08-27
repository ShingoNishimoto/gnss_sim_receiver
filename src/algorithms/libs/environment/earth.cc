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

Earth::Earth(double initial_julian_day):
CelestialBody(initial_julian_day)
{
    // NOTE: from the spice data base
    rotation_rate_rad_s_ = 2 * M_PI / (24 * 3600.0); // FIXME:
    gravity_constant_ = 3.986004354360959e5;
    radius_km_ = 6378.1366;
}

Earth::~Earth()
{
}

// void Earth::Update(double julian_date)
// {
//     // Position and velocity update based on two-body
// }
