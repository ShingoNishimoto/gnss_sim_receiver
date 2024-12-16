/*!
 * \file frame.h
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

#ifndef GNSS_SDR_FRAME_H
#define GNSS_SDR_FRAME_H

#include "earth.h"
#include "moon.h"
// #include "time_system.h"

#ifdef __cplusplus
class Frame
{
public:
    Frame(Earth* earth, Moon* moon, TimeSystem* time_system);
    virtual ~Frame();

    void GetDcmEciToEcef(const double tt, double dcm[3 * 3]);
    void GetDcmEcefToEci(const double tt, double dcm[3 * 3]);
    inline Earth* GetEarth(void) const { return earth_; };
    inline Moon* GetMoon(void) const { return moon_; };

private:
    // NOTE: ECI=J2000
    Earth* earth_;
    Moon* moon_;
    TimeSystem* time_system_;
};
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct Frame Frame;

Frame* FrameInit(earth* earth, moon* moon, TimeSystem* time_system);
void GetDcmEciToEcef(Frame* frame, const double tt, double dcm[3 * 3]);
void GetDcmEcefToEci(Frame* frame, const double tt, double dcm[3 * 3]);
void FrameDestroy(Frame* frame);

#ifdef __cplusplus
}
#endif

#endif  // GNSS_SDR_FRAME_H
