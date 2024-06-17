/*!
 * \file main.cc
 * \brief Main file of the GNSS-SDR program.
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *
 * It sets up the logging system, creates a ControlThread object,
 * makes it run, and releases memory back when the main thread has ended.
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2013 (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_VERSION
#define GNSS_SDR_VERSION "0.0.19"
#endif

#ifndef GOOGLE_STRIP_LOG
#define GOOGLE_STRIP_LOG 0
#endif

#include "concurrent_map.h"
#include "concurrent_queue.h"
#include "control_thread.h"
#include "gnss_sdr_filesystem.h"
#include "gnss_sdr_make_unique.h"
#include "gps_acq_assist.h"
#include <boost/exception/diagnostic_information.hpp>  // for diagnostic_information
#include <boost/exception/exception.hpp>               // for exception
#include <boost/thread/exceptions.hpp>                 // for thread_resource_error
#include <gflags/gflags.h>                             // for ShutDownCommandLineFlags
#include <glog/logging.h>                              // for FLAGS_log_dir
#include <chrono>                                      // for time_point
#include <exception>                                   // for exception
#include <iostream>                                    // for operator<<
#include <memory>                                      // for unique_ptr
#include <string>                                      // for string

#include "main.h"
#include "gnss_flowgraph.h"
#include "signal_source_interface.h"
#include "../algorithms/signal_source/adapters/osmosdr_signal_source.h"
#include "gnss_sdr_flags.h"
#include <libbladeRF.h>
#include <unistd.h>

#if CUDA_GPU_ACCEL
// For the CUDA runtime routines (prefixed with "cuda_")
#include <cuda_runtime.h>
#endif

#if GFLAGS_OLD_NAMESPACE
namespace gflags
{
using namespace google;
}
#endif

/*
 * Concurrent queues that communicates the Telemetry Decoder
 * to the Observables modules
 */

// For GPS NAVIGATION (L1)
Concurrent_Queue<Gps_Acq_Assist> global_gps_acq_assist_queue;
Concurrent_Map<Gps_Acq_Assist> global_gps_acq_assist_map;

int bladegps_thread(bladerf* dev, std::string args)
{
    std::string bladegps_args_ = args;
    // char *bladegps_argv_[20] = {"bladegps", NULL, };
    char *bladegps_argv_[20] = {NULL, };
    int bladegps_argc_ = 1;  // NOTE: start from 1 to use optarg in bladegps_main()
    char workstr[1024];
    {
        strcpy(workstr, bladegps_args_.c_str());
        char *context;
        char *token = strtok_r(workstr, " ", &context);
        while (token)
        {
            bladegps_argv_[bladegps_argc_ ++] = token;
            token = strtok_r(NULL, " ", &context);
        }
    }
    printf("Running bladeGPS with parameter:'%s'.\n", args.c_str());
    int ret = bladegps_main(dev, bladegps_argc_, bladegps_argv_);
    gflags::ShutDownCommandLineFlags();

    return ret;
}

int bladerffire_thread(bladerf* dev, bladerf_trigger *trig, boost::thread* bladegps_thread)
{
    sleep(1);
    int status = bladerf_trigger_fire(dev, trig);
    if (status != 0)
        {
            fprintf(stderr, "Unable to fire trigger: %s\n", bladerf_strerror(status));
            return 0;
        }
    std::cout << "Fire!!" << std::endl;
    bladegps_thread->join();
    exit(0);
}

std::shared_ptr<OsmosdrSignalSource> osmo_sig_src;

int main(int argc, char** argv)
{
    try
        {
            const std::string intro_help(
                std::string("\nGNSS-SDR is an Open Source GNSS Software Defined Receiver\n") +
                "Copyright (C) 2010-2023 (see AUTHORS file for a list of contributors)\n" +
                "This program comes with ABSOLUTELY NO WARRANTY;\n" +
                "See COPYING file to see a copy of the General Public License\n \n");

            const std::string gnss_sdr_version(GNSS_SDR_VERSION);
            gflags::SetUsageMessage(intro_help);
            gflags::SetVersionString(gnss_sdr_version);
            gflags::ParseCommandLineFlags(&argc, &argv, true);
            std::cout << "Initializing GNSS-SDR v" << gnss_sdr_version << " ... Please wait.\n";
        }
    catch (const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            std::cout << "GNSS-SDR program ended.\n";
            return 1;
        }

#if CUDA_GPU_ACCEL
    // Reset the device
    // cudaDeviceReset causes the driver to clean up all state. While
    // not mandatory in normal operation, it is good practice.  It is also
    // needed to ensure correct operation when the application is being
    // profiled. Calling cudaDeviceReset causes all profile data to be
    // flushed before the application exits
    cudaDeviceReset();
    std::cout << "Reset CUDA device done.\n";
#endif

    if (GOOGLE_STRIP_LOG == 0)
        {
            google::InitGoogleLogging(argv[0]);
            if (FLAGS_log_dir.empty())
                {
                    std::cout << "Logging will be written at "
                              << fs::temp_directory_path()
                              << '\n'
                              << "Use gnss-sdr --log_dir=/path/to/log to change that.\n";
                }
            else
                {
                    try
                        {
                            const fs::path p(FLAGS_log_dir);
                            if (!fs::exists(p))
                                {
                                    std::cout << "The path "
                                              << FLAGS_log_dir
                                              << " does not exist, attempting to create it.\n";
                                    errorlib::error_code ec;
                                    if (!fs::create_directory(p, ec))
                                        {
                                            std::cerr << "Could not create the " << FLAGS_log_dir << " folder. GNSS-SDR program ended.\n";
                                            gflags::ShutDownCommandLineFlags();
                                            return 1;
                                        }
                                }
                            std::cout << "Logging will be written at " << FLAGS_log_dir << '\n';
                        }
                    catch (const std::exception& e)
                        {
                            std::cerr << e.what() << '\n';
                            std::cerr << "Could not create the " << FLAGS_log_dir << " folder. GNSS-SDR program ended.\n";
                            gflags::ShutDownCommandLineFlags();
                            return 1;
                        }
                }
        }

    std::chrono::time_point<std::chrono::system_clock> start;
    std::chrono::time_point<std::chrono::system_clock> end;
    start = std::chrono::system_clock::now();
    int return_code = 0;
    try
        {
            std::cout << "Waiting..." << std::endl;
            sleep(3); // at least 3 sec with laptop. TODO: check with raspi

            auto control_thread = std::make_unique<ControlThread>();

            std::shared_ptr<GNSSFlowgraph> flowgraph_ = control_thread->flowgraph();
            std::vector<std::shared_ptr<SignalSourceInterface>> sig_src = flowgraph_.get()->get_sig_source();
            osmo_sig_src = std::dynamic_pointer_cast<OsmosdrSignalSource>(sig_src[0]);
            bladerf *bladerf_dev = (bladerf*)osmo_sig_src.get()->get_devptr();

            if (FLAGS_bladegps_args != "")
            {
                std::cout << "... Executing bladegps_main() with an independent thread." << std::endl;
                fLS::clstring argstring = FLAGS_bladegps_args;
                // boost::thread* th_bladegps = new boost::thread(boost::bind(bladegps_thread, bladerf_dev, argstring));
                boost::thread(boost::bind(bladegps_thread, bladerf_dev, argstring));

                std::cout << "Waiting..." << std::endl;
                sleep(3);
            }
            // record startup time
            start = std::chrono::system_clock::now();
            return_code = control_thread->run();
        }
    catch (const boost::thread_resource_error& e)
        {
            std::cerr << "Failed to create boost thread.\n";
            return_code = 1;
        }
    catch (const boost::exception& e)
        {
            if (GOOGLE_STRIP_LOG == 0)
                {
                    LOG(WARNING) << "Boost exception: " << boost::diagnostic_information(e);
                    std::cerr << boost::diagnostic_information(e) << '\n';
                }
            else
                {
                    std::cerr << "Boost exception: " << boost::diagnostic_information(e) << '\n';
                }
            return_code = 1;
        }
    catch (const std::exception& ex)
        {
            if (GOOGLE_STRIP_LOG == 0)
                {
                    LOG(WARNING) << "C++ Standard Library exception: " << ex.what();
                    std::cerr << ex.what() << '\n';
                }
            else
                {
                    std::cerr << "C++ Standard Library exception: " << ex.what() << '\n';
                }
            return_code = 1;
        }
    catch (...)
        {
            if (GOOGLE_STRIP_LOG == 0)
                {
                    LOG(WARNING) << "Unexpected catch. This should not happen.";
                    std::cerr << "Unexpected error.\n";
                }
            else
                {
                    std::cerr << "Unexpected catch. This should not happen.\n";
                }
            return_code = 1;
        }

    // report the elapsed time
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;

    std::cout << "Total GNSS-SDR run time: "
              << elapsed_seconds.count()
              << " [seconds]\n";

    gflags::ShutDownCommandLineFlags();
    std::cout << "GNSS-SDR program ended.\n";
    return return_code;
}
