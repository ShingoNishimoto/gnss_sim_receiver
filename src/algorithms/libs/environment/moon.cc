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
// #include "rtklib_rtkcmn.h"  // for matrix function
#include <SpiceUsr.h>  // for SPICE
#include <iostream>

// TODO: those functions should be in a proper file.
static double dot(const double *a, const double *b, int n);
static double norm_rtk(const double *a, int n);
static void cross3(const double *a, const double *b, double *c);
static int normv3(const double *a, double *b);
// static void matmul(const char *tr, int n, int k, int m, double alpha,
//     const double *A, const double *B, double beta, double *C);

extern "C"
{
    void dgemm_(char *, char *, int *, int *, int *, double *, double *, int *, double *, int *, double *, double *, int *);
    extern void dgetrf_(int *, int *, double *, int *, int *, int *);
    extern void dgetri_(int *, double *, int *, int *, double *, int *, int *);
    extern void dgetrs_(char *, int *, int *, double *, int *, int *, double *, int *, int *);
}

Moon::Moon(double initial_tt, double mu_center_of_mass):
CelestialBody(initial_tt),
mu_center_of_mass_(mu_center_of_mass)
{
    // NOTE: from the spice data base
    rotation_rate_rad_s_ = 1.41426064e-4;  // TODO: other axis also has the angular velocity
    gravity_constant_ = 4.902800066163796e3;
    radius_km_ = 1737.4;

    inertial_frame_ = "J2000";  // MOON_J2000 is not available.
    fixed_frame_ = "MOON_PA";  // MOON_ME or MOON_PA. IAU_MOON is not accurate.
    body_name_ = "MOON";

    // // NOTE: should be earlier than bladegps ephemeris file's time
    // double et_J2000 = 596466069.1829587;  // 2018 NOV 26 01:00:00
    // double r[3] = {-6.85737996e4, 3.34248086e5, 1.34431780e5};  // km
    // double v[3] = {-1.04911126, -2.22963119e-1, 3.18153224e-3};  // km/s
    // InitOrbitElement(et_J2000, r, v);

    UpdateStates(initial_tt);
}

Moon::~Moon()
{
}

void Moon::Update(double tt)
{
    CelestialBody::Update(tt);
    // Position and velocity update based on two-body
    UpdateStates(tt);
}

void Moon::InitOrbitElement(double et_J2000, double r_ini[3], double v_ini[3])
{
    // angular momentum
    cross3(r_ini, v_ini, angular_momentum_);
    // energy
    double r_norm = norm_rtk(r_ini, 3);
    double v_norm = norm_rtk(v_ini, 3);
    energy_ = 0.5 * pow(v_norm, 2) - mu_center_of_mass_ / r_norm;
    // periapsis vector
    cross3(v_ini, angular_momentum_, periapsis_vector_);
    double r_dir[3];
    if (!normv3(r_ini, r_dir))
        {
            std::cerr << "Something wrong!" << std::endl;
            return;
        }
    periapsis_vector_[0] -= mu_center_of_mass_ * r_dir[0];
    periapsis_vector_[1] -= mu_center_of_mass_ * r_dir[1];
    periapsis_vector_[2] -= mu_center_of_mass_ * r_dir[2];

    a_ = -mu_center_of_mass_ / (2 * energy_);
    double semi_latus_rectum = pow(norm_rtk(angular_momentum_, 3), 2) / mu_center_of_mass_;
    e_ = norm_rtk(periapsis_vector_, 3) / mu_center_of_mass_;

    double cos_theta = (semi_latus_rectum - r_norm) / (e_ * r_norm);
    double cos_E = (e_ + cos_theta) / (1 + e_ * cos_theta);
    if (fabs(cos_E) > 1)
        cos_E /= fabs(cos_E);
    double E = acos(cos_E);
    // direction check
    if (!normv3(periapsis_vector_, p_) || !normv3(angular_momentum_, w_))
        {
            std::cerr << "Something wrong!" << std::endl;
            return;
        }
    cross3(w_, p_, q_);
    if (dot(r_ini, q_, 3) < 0)
        {
            E = 2 * M_PI - E;
        }

    n_ = sqrt(mu_center_of_mass_ / pow(a_, 3));
    T_ = 2 * M_PI / n_;
    double t_tp = (E - e_ * sin(E)) / n_;  // [sec]
    tp_J2000_ = et_J2000 - t_tp;
}

void Moon::UpdateStates(double tt)
{
    // const double t_J2000 = time_system_.ConvJulianDateToTt(julian_date);
    // double t_tp_res = t_J2000 - tp_J2000_ - T_ * std::floor((t_J2000 - tp_J2000_) / T_);
    // double M = n_ * t_tp_res;
    // // Adjust to within 2pi
    // if (M > 2 * M_PI)
    //         M -= 2 * M_PI * std::floor(M / (2 * M_PI));
    // // Solve the Kepler equation
    // double E = M_PI; // initial value
    // while (std::fabs(KeplerEq(E, M)) > 1e-5)
    //     {
    //         E = E - KeplerEq(E, M) / KeplerEqDot(E);
    //     }

    // double true_anomaly = acos((cos(E) - e_) / (1 - e_ * cos(E)));
    // if (M > M_PI)
    //     true_anomaly = 2 * M_PI - true_anomaly;

    // double r = a_ * (1 - e_ * cos(E));
    // double pos_inplane_m[3] = {r * cos(true_anomaly) * 1000, r * sin(true_anomaly) * 1000, 0};
    // double v = sqrt(mu_center_of_mass_ * (2 / r - 1 / a_));
    // double gamma = asin(norm_rtk(angular_momentum_, 3) / (r * v));
    // if (true_anomaly > M_PI)
    //     gamma = M_PI - gamma;

    // double vel_inplane_m_s[3] = {v * cos(true_anomaly + gamma) * 1000, v * sin(true_anomaly + gamma) * 1000, 0};

    // // FIXME: this conversion has a large error, because moon doesn't follow two-body problem.
    // double dcm_inplane_to_eci[9] = {
    //     p_[0], q_[0], w_[0],
    //     p_[1], q_[1], w_[1],
    //     p_[2], q_[2], w_[2]};
    // matmul("NN", 3, 1, 3, 1, dcm_inplane_to_eci, pos_inplane_m, 0, position_i_m_);
    // matmul("NN", 3, 1, 3, 1, dcm_inplane_to_eci, vel_inplane_m_s, 0, velocity_i_m_s_);

    // SPICE based
    double lt;
    double states[6];
    spkezr_c(body_name_.c_str(), tt, inertial_frame_.c_str(), "NONE", "EARTH", states, &lt);
    for (uint8_t i = 0; i < 3; i++)
        {
            position_i_m_[i] = states[i] * 1000;
            velocity_i_m_s_[i] = states[3 + i] * 1000;
        }
    // spkpos_c("MOON", et, "J2000", "NONE", "EARTH", position_i_m_, &lt);
    // double rotate[3][3];
    // pxform_c("J2000", "ITRF93", tt, rotate);
}

// Following functions are copy from rtklib_rtkcmn.cc

/* inner product ---------------------------------------------------------------
 * inner product of vectors
 * args   : double *a,*b     I   vector a,b (n x 1)
 *          int    n         I   size of vector a,b
 * return : a'*b
 *-----------------------------------------------------------------------------*/
double dot(const double *a, const double *b, int n)
{
    double c = 0.0;

    while (--n >= 0)
        {
            c += a[n] * b[n];
        }
    return c;
}

/* euclid norm -----------------------------------------------------------------
 * euclid norm of vector
 * args   : double *a        I   vector a (n x 1)
 *          int    n         I   size of vector a
 * return : || a ||
 *-----------------------------------------------------------------------------*/
double norm_rtk(const double *a, int n)
{
    return std::sqrt(dot(a, a, n));
}

/* outer product of 3d vectors -------------------------------------------------
 * outer product of 3d vectors
 * args   : double *a,*b     I   vector a,b (3 x 1)
 *          double *c        O   outer product (a x b) (3 x 1)
 * return : none
 *-----------------------------------------------------------------------------*/
void cross3(const double *a, const double *b, double *c)
{
    c[0] = a[1] * b[2] - a[2] * b[1];
    c[1] = a[2] * b[0] - a[0] * b[2];
    c[2] = a[0] * b[1] - a[1] * b[0];
}

/* normalize 3d vector ---------------------------------------------------------
 * normalize 3d vector
 * args   : double *a        I   vector a (3 x 1)
 *          double *b        O   normlized vector (3 x 1) || b || = 1
 * return : status (1:ok,0:error)
 *-----------------------------------------------------------------------------*/
int normv3(const double *a, double *b)
{
    double r;
    if ((r = norm_rtk(a, 3)) <= 0.0)
        {
            return 0;
        }
    b[0] = a[0] / r;
    b[1] = a[1] / r;
    b[2] = a[2] / r;
    return 1;
}

/* multiply matrix (wrapper of blas dgemm) -------------------------------------
 * multiply matrix by matrix (C=alpha*A*B+beta*C)
 * args   : char   *tr       I  transpose flags ("N":normal,"T":transpose)
 *          int    n,k,m     I  size of (transposed) matrix A,B
 *          double alpha     I  alpha
 *          double *A,*B     I  (transposed) matrix A (n x m), B (m x k)
 *          double beta      I  beta
 *          double *C        IO matrix C (n x k)
 * return : none
 *-----------------------------------------------------------------------------*/
// void matmul(const char *tr, int n, int k, int m, double alpha,
//     const double *A, const double *B, double beta, double *C)
// {
//     int lda = tr[0] == 'T' ? m : n;
//     int ldb = tr[1] == 'T' ? k : m;

//     dgemm_(const_cast<char *>(tr), const_cast<char *>(tr) + 1, &n, &k, &m, &alpha, const_cast<double *>(A), &lda, const_cast<double *>(B),
//         &ldb, &beta, C, &n);
// }

extern "C" {
    moon* MoonInit(int initial_gps_week, double initial_gps_sec, double mu_com)
    {
        TimeSystem::gpstime_t gps_time;
        gps_time.week = initial_gps_week;
        gps_time.sec = initial_gps_sec;
        TimeSystem time_system = TimeSystem();
        double initial_tt = time_system.ConvGPSTimeToTt(gps_time);
        return reinterpret_cast<moon*>(new Moon(initial_tt, mu_com));
    }

    const double* GetPositionI(moon* moon, double tt)
    {
        reinterpret_cast<Moon*>(moon)->Update(tt);
        return reinterpret_cast<Moon*>(moon)->GetPositionI();
    }

    double GetRadiusKm(moon* moon)
    {
        return reinterpret_cast<Moon*>(moon)->GetRadiusKm();
    }

    void MoonDestroy(moon* moon)
    {
        delete reinterpret_cast<Moon*>(moon);
    }
}