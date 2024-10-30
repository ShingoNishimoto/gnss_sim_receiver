# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2010-2020 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause

# include_directories(../rtklib)

set(ENVIRONMENT_SOURCES
    celestial_body.cc
    frame.cc
    earth.cc
    moon.cc
    time_system.cc
)

set(ENVIRONMENT_HEADERS
    celestial_body.h
    frame.h
    earth.h
    moon.h
    time_system.h
)

list(SORT ENVIRONMENT_HEADERS)
list(SORT ENVIRONMENT_SOURCES)

if(USE_CMAKE_TARGET_SOURCES)
    add_library(algorithms_libs_environment STATIC)
    target_sources(algorithms_libs_environment
        PRIVATE
            ${ENVIRONMENT_SOURCES}
        PUBLIC
            ${ENVIRONMENT_HEADERS}
    )
else()
    source_group(Headers FILES ${ENVIRONMENT_HEADERS})
    add_library(algorithms_libs_environment
        ${ENVIRONMENT_SOURCES}
        ${ENVIRONMENT_HEADERS}
    )
endif()

# Setting for SPICE
set(HOME_DIR $ENV{HOME})
set(SPICE_LIB_DIR "${HOME_DIR}/work/cspice/lib/")
set(SPICE_INCLUDE_DIR "${HOME_DIR}/work/cspice/include/")

target_include_directories(algorithms_libs_environment
    PRIVATE
        ${SPICE_INCLUDE_DIR}
)

target_link_libraries(algorithms_libs_environment
    PRIVATE
        # core_system_parameters
        algorithms_libs
        Glog::glog
        # LAPACK::LAPACK
        # BLAS::BLAS
        ${SPICE_LIB_DIR}cspice.a
)

if(ENABLE_ARMA_NO_DEBUG)
    target_compile_definitions(algorithms_libs_environment
        PRIVATE -DARMA_NO_BOUND_CHECKING=1
    )
endif()

if(FILESYSTEM_FOUND)
    target_compile_definitions(algorithms_libs_environment PUBLIC -DHAS_STD_FILESYSTEM=1)
    if(find_experimental)
        target_compile_definitions(algorithms_libs_environment PUBLIC -DHAS_STD_FILESYSTEM_EXPERIMENTAL=1)
    endif()
    target_link_libraries(algorithms_libs_environment PUBLIC std::filesystem)
else()
    target_link_libraries(algorithms_libs_environment PUBLIC Boost::filesystem Boost::system)
endif()

set_property(TARGET algorithms_libs_environment
    APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

set_property(TARGET algorithms_libs_environment
    APPEND PROPERTY POSITION_INDEPENDENT_CODE ON
)
