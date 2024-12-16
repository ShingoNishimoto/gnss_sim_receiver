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

void Frame::GetDcmEciToEcef(const double tt, double dcm[3 * 3])
{
    double* dcm_eci_to_ecef = earth_->GetDcmI2Fixed(tt);
    for (u_int8_t i = 0; i < 9; i++)
    {
        dcm[i] = dcm_eci_to_ecef[i];
    }
}

void Frame::GetDcmEcefToEci(const double tt, double dcm[3 * 3])
{
    double* dcm_ecef_to_eci = earth_->GetDcmFixed2I(tt);
    for (u_int8_t i = 0; i < 9; i++)
    {
        dcm[i] = dcm_ecef_to_eci[i];
    }
}

extern "C" {
    Frame* FrameInit(earth* earth, moon* moon, TimeSystem* time_system)
    {
        return new Frame(reinterpret_cast<Earth*>(earth), reinterpret_cast<Moon*>(moon), time_system);
    }

    void GetDcmEciToEcef(Frame* frame, const double tt, double dcm[3 * 3])
    {
        frame->GetDcmEciToEcef(tt, dcm);
    }

    void GetDcmEcefToEci(Frame* frame, const double tt, double dcm[3 * 3])
    {
        frame->GetDcmEcefToEci(tt, dcm);
    }

    void FrameDestroy(Frame* frame)
    {
        delete frame;
    }
}