/*!
 * \file pvt_conf.h
 * \brief Class that contains all the configuration parameters for the PVT block
 * \author Carles Fernandez, 2018. cfernandez(at)cttc.es
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

#ifndef GNSS_SDR_PVT_CONF_H
#define GNSS_SDR_PVT_CONF_H

#include <cstdint>
#include <map>
#include <string>
#include "MATH_CONSTANTS.h"

/** \addtogroup PVT
 * \{ */
/** \addtogroup PVT_libs
 * \{ */


class Pvt_Conf
{
public:
    std::map<int, int> rtcm_msg_rate_ms;

    std::string rinex_name = std::string("-");
    std::string dump_filename;
    std::string nmea_dump_filename;
    std::string nmea_dump_devname;
    std::string rtcm_dump_devname;
    std::string an_dump_devname;
    std::string output_path = std::string(".");
    std::string rinex_output_path = std::string(".");
    std::string gpx_output_path = std::string(".");
    std::string geojson_output_path = std::string(".");
    std::string nmea_output_file_path = std::string(".");
    std::string kml_output_path = std::string(".");
    std::string xml_output_path = std::string(".");
    std::string rtcm_output_file_path = std::string(".");
    std::string has_output_file_path = std::string(".");
    std::string udp_addresses;
    std::string udp_eph_addresses;
    std::string log_source_timetag_file;

    uint32_t type_of_receiver = 0;
    uint32_t observable_interval_ms = 20;
    uint32_t output_cnt_for_clk_prop_after_fix = 0;

    int32_t output_rate_ms = 0;
    int32_t display_rate_ms = 0;
    int32_t kml_rate_ms = 20;
    int32_t gpx_rate_ms = 20;
    int32_t geojson_rate_ms = 20;
    int32_t nmea_rate_ms = 20;
    int32_t rinex_version = 0;
    int32_t rinexobs_rate_ms = 0;
    int32_t an_rate_ms = 20;
    int32_t max_obs_block_rx_clock_offset_ms = 40;
    int32_t pseudo_sat_channel = -1;
    int udp_port = 0;
    int udp_eph_port = 0;
    int rtk_trace_level = 0;

    uint16_t rtcm_tcp_port = 0;
    uint16_t rtcm_station_id = 0;

    bool flag_nmea_tty_port = false;
    bool flag_rtcm_server = false;
    bool flag_rtcm_tty_port = false;
    bool output_enabled = true;
    bool rinex_output_enabled = true;
    bool gpx_output_enabled = true;
    bool geojson_output_enabled = true;
    bool nmea_output_file_enabled = true;
    bool an_output_enabled = false;
    bool kml_output_enabled = true;
    bool xml_output_enabled = true;
    bool rtcm_output_file_enabled = true;
    bool monitor_enabled = false;
    bool monitor_ephemeris_enabled = false;
    bool protobuf_enabled = true;
    bool enable_rx_clock_correction = true;
    bool enable_rx_clock_propagation = false;
    bool show_local_time_zone = false;
    bool pre_2009_file = false;
    bool dump = false;
    bool dump_mat = true;
    bool log_source_timetag = false;
    bool use_e6_for_pvt = true;
    bool use_has_corrections = true;
    bool use_unhealthy_sats = false;
    bool rec_antenna_attitude_fix = true;
    bool share_rx_clock_bias = false;
    bool hybrid_mode = false;

    // PVT KF parameters
    bool enable_pvt_kf = false;
    double measures_ecef_pos_sd_m = 1.0;
    double measures_ecef_vel_sd_ms = 0.1;
    double system_ecef_pos_sd_m = 0.01;
    double system_ecef_vel_sd_ms = 0.001;

    // PVT EKF parameters for satellite positioning
    bool enable_pvt_ekf = false;
    uint8_t center_of_gravity_type = 0;
    double initial_ecef_pos_sd_m = 100;
    double initial_ecef_vel_sd_ms = 1;
    double system_clock_offset_sd_m = 0.1;
    double system_clock_drift_sd_ms = 0.01;

    // Receiver antenna attitude
    double ini_rec_antenna_az_rad = 0 * D2R;
    double ini_rec_antenna_el_rad = 90 * D2R;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_PVT_CONF_H
