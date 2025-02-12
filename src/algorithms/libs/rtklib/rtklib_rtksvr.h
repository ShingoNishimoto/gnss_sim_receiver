/*!
 * \file rtklib_rtksvr.h
 * \brief rtk server functions
 * \authors <ul>
 *          <li> 2007-2013, T. Takasu
 *          <li> 2017, Javier Arribas
 *          <li> 2017, Carles Fernandez
 *          </ul>
 *
 * This is a derived work from RTKLIB http://www.rtklib.com/
 * The original source code at https://github.com/tomojitakasu/RTKLIB is
 * released under the BSD 2-clause license with an additional exclusive clause
 * that does not apply here. This additional clause is reproduced below:
 *
 * " The software package includes some companion executive binaries or shared
 * libraries necessary to execute APs on Windows. These licenses succeed to the
 * original ones of these software. "
 *
 * Neither the executive binaries nor the shared libraries are required by, used
 * or included in GNSS-SDR.
 *
 * -----------------------------------------------------------------------------
 * Copyright (C) 2007-2013, T. Takasu
 * Copyright (C) 2017, Javier Arribas
 * Copyright (C) 2017, Carles Fernandez
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_RTKLIB_RKTSVR_H
#define GNSS_SDR_RTKLIB_RKTSVR_H

#include "rtklib.h"


const solopt_t SOLOPT_DEFAULT = {
    /* defaults solution output options */
    SOLF_LLH, TIMES_GPST, 1, 3, /* posf, times, timef, timeu */
    0, 1, 0, 0, 0, 0,           /* degf, outhead, outopt, datum, height, geoid */
    0, 0, 0,                    /* solstatic, sstat, trace */
    {0.0, 0.0},                 /* nmeaintv */
    " ", "", 0                  /* separator/program name */
};

const prcopt_t PRCOPT_DEFAULT = {            /* defaults processing options */
    PMODE_SINGLE, 0, 2, SYS_GPS,             /* mode, soltype, nf, navsys */
    15.0 * D2R, {{}, {{}, {}}},              /* elmin, snrmask */
    0, 1, 1, 1,                              /* sateph, modear, glomodear, bdsmodear */
    5, 0, 10, 1,                             /* maxout, minlock, minfix, armaxiter */
    0, 0, 0, 0,                              /* estion, esttrop, dynamics, tidecorr */
    1, 0, 0, 0, 0,                           /* niter, codesmooth, intpref, sbascorr, sbassatsel */
    0, 0,                                    /* rovpos, refpos */
    {100.0, 100.0, 100.0},                   /* eratio[] */
    {100.0, 0.003, 0.003, 0.0, 1.0},         /* err[] */
    {30.0, 0.03, 0.3},                       /* std[] */
    {1e-4, 1e-3, 1e-4, 1e-1, 1e-2, 0.0},     /* prn[] */
    5E-12,                                   /* sclkstab */
    {3.0, 0.9999, 0.25, 0.1, 0.05, 0, 0, 0}, /* thresar */
    0.0, 0.0, 0.05,                          /* elmaskar, almaskhold, thresslip */
    30.0, 30.0, 30.0,                        /* maxtdif, maxinno, maxgdop */
    {}, {}, {},                              /* baseline, ru, rb */
    {"", ""},                                /* anttype */
    {}, {}, {},                              /* antdel, pcv, exsats */
    0, 0, 0, {"", ""}, {}, 0, {{}, {}}, {{}, {{}, {}}, {{}, {}}, {}, {}}, 0, {}, true, false};


void writesolhead(stream_t *stream, const solopt_t *solopt);

void saveoutbuf(rtksvr_t *svr, unsigned char *buff, int n, int index);

void writesol(rtksvr_t *svr, int index);

void updatenav(nav_t *nav);

void updatefcn(rtksvr_t *svr);

void updatesvr(rtksvr_t *svr, int ret, obs_t *obs, nav_t *nav, int sat,
    sbsmsg_t *sbsmsg, int index, int iobs);

int decoderaw(rtksvr_t *svr, int index);

void decodefile(rtksvr_t *svr, int index);

void *rtksvrthread(void *arg);

int rtksvrinit(rtksvr_t *svr);

void rtksvrfree(rtksvr_t *svr);

void rtksvrlock(rtksvr_t *svr);

void rtksvrunlock(rtksvr_t *svr);

int rtksvrstart(rtksvr_t *svr, int cycle, int buffsize, int *strs,
    char **paths, const int *formats, int navsel, char **cmds,
    char **rcvopts, int nmeacycle, int nmeareq,
    const double *nmeapos, prcopt_t *prcopt,
    solopt_t *solopt, stream_t *moni);

void rtksvrstop(rtksvr_t *svr, char **cmds);

int rtksvropenstr(rtksvr_t *svr, int index, int str, const char *path,
    const solopt_t *solopt);

void rtksvrclosestr(rtksvr_t *svr, int index);

int rtksvrostat(rtksvr_t *svr, int rcv, gtime_t *time, int *sat,
    double *az, double *el, int **snr, int *vsat);

void rtksvrsstat(rtksvr_t *svr, int *sstat, char *msg);


#endif
