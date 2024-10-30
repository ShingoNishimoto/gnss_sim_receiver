/*!
 * \file frame.c
 * \brief Class to manage the frame (coordinate system)
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

#include "frame.h"

Frame::Frame(Earth* earth, Moon* moon, TimeSystem* time_system):
earth_(earth),
moon_(moon),
time_system_(time_system)
{
}

Frame::~Frame()
{}

void Frame::GetDcmEciToEcef(const double julian_day, double dcm[3 * 3])
{
    double* dcm_eci_to_ecef = earth_->GetDcmI2Fixed(julian_day);
    for (u_int8_t i = 0; i < 9; i++)
    {
        dcm[i] = dcm_eci_to_ecef[i];
    }
}

void Frame::GetDcmEcefToEci(const double julian_day, double dcm[3 * 3])
{
    double* dcm_ecef_to_eci = earth_->GetDcmFixed2I(julian_day);
    for (u_int8_t i = 0; i < 9; i++)
    {
        dcm[i] = dcm_ecef_to_eci[i];
    }
}

extern "C" {
    Frame* FrameInit(Earth* earth, Moon* moon, TimeSystem* time_system)
    {
        return new Frame(earth, moon, time_system);
    }

    void GetDcmEciToEcef(Frame* frame, const double julian_day, double dcm[3 * 3])
    {
        frame->GetDcmEciToEcef(julian_day, dcm);
    }

    void GetDcmEcefToEci(Frame* frame, const double julian_day, double dcm[3 * 3])
    {
        frame->GetDcmEcefToEci(julian_day, dcm);
    }

    void FrameDestroy(Frame* frame)
    {
        delete frame;
    }
}