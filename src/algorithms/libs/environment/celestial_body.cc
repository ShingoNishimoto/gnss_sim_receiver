/*!
 * \file celestial_body.cc
 * \brief Class to manage the celestial information
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

#include "celestial_body.h"
#include <cstdint>
#include <cmath>
#include <iostream>
#include <SpiceUsr.h>  // for SPICE

CelestialBody::CelestialBody(double initial_tt):
rotation_rate_rad_s_(0),
gravity_constant_(0),
radius_km_(0),
initial_tt_(initial_tt),
time_system_(TimeSystem()),
inertial_frame_("J2000"),
fixed_frame_("ITRF93"),
body_name_("EARTH")
{
    // Load SPICE kernel
    const char *home_dir = getenv("HOME");
    if (home_dir == nullptr)
        {
            std::cerr << "HOME environment variable not set." << std::endl;
        }
    furnsh_c((std::string(home_dir) + "/work/cspice/generic_kernels/spk/planets/de430.bsp").c_str());
    furnsh_c((std::string(home_dir) + "/work/cspice/generic_kernels/lsk/naif0012.tls").c_str());
    furnsh_c((std::string(home_dir) + "/work/cspice/generic_kernels/pck/earth_070425_370426_predict.bpc").c_str());
    furnsh_c((std::string(home_dir) + "/work/cspice/generic_kernels/pck/earth_fixed.tf").c_str());
    furnsh_c((std::string(home_dir) + "/work/cspice/generic_kernels/pck/gm_de431.tpc").c_str());
    furnsh_c((std::string(home_dir) + "/work/cspice/generic_kernels/pck/moon_pa_de440_200625.bpc").c_str());
    furnsh_c((std::string(home_dir) + "/work/cspice/generic_kernels/pck/moon_pa_de421_1900-2050.bpc").c_str());
    furnsh_c((std::string(home_dir) + "/work/cspice/generic_kernels/fk/satellites/moon_080317.tf").c_str());
    furnsh_c((std::string(home_dir) + "/work/cspice/generic_kernels/pck/pck00010.tpc").c_str());

    // Update(initial_tt);
}

CelestialBody::~CelestialBody()
{
    // unload SPICE kernel
    const char *home_dir = getenv("HOME");
    if (home_dir == nullptr)
        {
            std::cerr << "HOME environment variable not set." << std::endl;
        }
    unload_c((std::string(home_dir) + "/work/cspice/generic_kernels/spk/planets/de430.bsp").c_str());
    unload_c((std::string(home_dir) + "/work/cspice/generic_kernels/lsk/naif0012.tls").c_str());
    unload_c((std::string(home_dir) + "/work/cspice/generic_kernels/pck/earth_070425_370426_predict.bpc").c_str());
    unload_c((std::string(home_dir) + "/work/cspice/generic_kernels/pck/earth_fixed.tf").c_str());
    unload_c((std::string(home_dir) + "/work/cspice/generic_kernels/pck/gm_de431.tpc").c_str());
    unload_c((std::string(home_dir) + "/work/cspice/generic_kernels/pck/moon_pa_de440_200625.bpc").c_str());
    unload_c((std::string(home_dir) + "/work/cspice/generic_kernels/pck/moon_pa_de421_1900-2050.bpc").c_str());
    unload_c((std::string(home_dir) + "/work/cspice/generic_kernels/fk/satellites/moon_080317.tf").c_str());
    unload_c((std::string(home_dir) + "/work/cspice/generic_kernels/pck/pck00010.tpc").c_str());
}

void CelestialBody::Update(double tt)
{
    // double delta_t_sec = tt - initial_tt_;
    // double rotation_angle_rad = rotation_rate_rad_s_ * delta_t_sec;
    // // Just considering rotation around z axis
    // RotationMatrixAroundZ((rotation_angle_rad, dcm_fixed_to_i_);

    double rotate[3][3];
    pxform_c(inertial_frame_.c_str(), fixed_frame_.c_str(), tt, rotate);
    for (uint8_t i = 0; i < 3; i++)
        {
            for (uint8_t j = 0; j < 3; j++)
                dcm_i_to_fixed_[3 * i + j] = rotate[i][j];
        }

    // Transpose
    for (uint8_t i = 0; i < 3; i++)
    {
        for (uint8_t j = 0; j < 3; j++)
        {
            dcm_fixed_to_i_[3 * j + i] = dcm_i_to_fixed_[3 * i + j];
        }
    }
}


void CelestialBody::GetGravityAcceleration(const double pos_i_m[3], double tt, uint16_t n, uint16_t m, double* acc_ms2) const
{
    const double r_norm_km = sqrt(pos_i_m[0] * pos_i_m[0] + pos_i_m[1] * pos_i_m[1] + pos_i_m[2] * pos_i_m[2]) * 1e-3;
    for (uint8_t i = 0; i < 3; i++)
        {
            acc_ms2[i] = -gravity_constant_ / pow(r_norm_km, 3) * pos_i_m[i];
        }

    // FIXME: Higher order
    double x_i[6] = {0}, x_f[6] = {0};
    for (uint8_t i = 0; i < 3; i++)
        x_i[i] = pos_i_m[i];
    ConvI2Fixed(x_i, x_f, tt);
}


// FIXME: can be heavy if accessing the kernel every time.
void CelestialBody::ConvI2Fixed(const double* x_i, double* x_f, double tt) const
{
    // double dcm_i2f[9] = GetDcmI2Fixed(tt);
    // MatMultiply(dcm_i2f, x_i, x_f, 3, 1, 3);

    double xform[6][6];
    sxform_c(inertial_frame_.c_str(), fixed_frame_.c_str(), tt, xform);
    mxvg_c(xform, x_i, 6, 6, x_f);
}


void CelestialBody::ConvFixed2I(const double* x_f, double* x_i, double tt) const
{
    // double dcm_f2i[9] = GetDcmFixed2I(tt);
    // MatMultiply(dcm_f2i, x_f, x_i, 3, 1, 3);

    double xform[6][6];
    sxform_c(fixed_frame_.c_str(), inertial_frame_.c_str(), tt, xform);
    mxvg_c(xform, x_f, 6, 6, x_i);
}


void CelestialBody::ConvMatI2Fixed(const double* A_i, double* A_f, double tt) const
{
    double xform[6][6], tmp[6][6];
    sxform_c(inertial_frame_.c_str(), fixed_frame_.c_str(), tt, xform);
    mxmg_c(xform, A_i, 6, 6, 6, tmp);
    mxmtg_c(tmp, xform, 6, 6, 6, A_f);
}


void CelestialBody::ConvMatFixed2I(const double* A_f, double* A_i, double tt) const
{
    double xform[6][6], tmp[6][6];
    sxform_c(fixed_frame_.c_str(), inertial_frame_.c_str(), tt, xform);
    mxmg_c(xform, A_f, 6, 6, 6, tmp);
    mxmtg_c(tmp, xform, 6, 6, 6, A_i);
}


void CelestialBody::RotationMatrixAroundZ(double rotation_rad, double mat[9])
{
    // TODO: it is too rough, probably the accuracy is not enough.
    mat[3 * 0 + 0] = cos(rotation_rad);
    mat[3 * 0 + 1] = sin(rotation_rad);
    mat[3 * 0 + 2] = 0;
    mat[3 * 1 + 0] = -sin(rotation_rad);
    mat[3 * 1 + 1] = cos(rotation_rad);
    mat[3 * 1 + 2] = 0;
    mat[3 * 2 + 0] = 0;
    mat[3 * 2 + 1] = 0;
    mat[3 * 2 + 2] = 1;
}

/* multiply matrix -------------------- -------------------------------------
 * multiply matrix by matrix (C=A*B)
 * args   : int    n,k,m     I  size of (transposed) matrix A,B
 *          double *A,*B     I  (transposed) matrix A (n x m), B (m x k)
 *          double *C        IO matrix C (n x k)
 * return : none
 *-----------------------------------------------------------------------------*/
// void CelestialBody::MatMultiply(const double *A, const double *B, double *C,
//     uint16_t n, uint16_t k, uint16_t m)
// {
//     for (uint16_t in = 0; in < n; in++)
//         {
//             for (uint16_t ik = 0; ik < k; ik++)
//                 {
//                     C[in * n + ik] = 0;
//                     for (uint16_t im = 0; im < m; im++)
//                         {
//                             C[in * n + ik] += A[in * n + im] * B[im * m + ik];
//                         }
//                 }
//         }
// }