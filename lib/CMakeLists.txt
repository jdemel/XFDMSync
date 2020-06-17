# Copyright 2011,2012,2016 Free Software Foundation, Inc.
#
# This file is part of XFDMSync
#
# GNU Radio is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
#
# GNU Radio is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with GNU Radio; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.

########################################################################
# Setup library
########################################################################
include(GrPlatform) #define LIB_SUFFIX

list(APPEND xfdm_sync_sources
    sc_delay_corr_impl.cc
    sc_tagger_impl.cc
    xcorr_tagger_impl.cc
    frame_gate_impl.cc
    sync_tag_align_cc_impl.cc )

set(xfdm_sync_sources "${xfdm_sync_sources}" PARENT_SCOPE)
if(NOT xfdm_sync_sources)
	MESSAGE(STATUS "No C++ sources... skipping lib/")
	return()
endif(NOT xfdm_sync_sources)

add_library(gnuradio-xfdm_sync SHARED ${xfdm_sync_sources})
target_link_libraries(gnuradio-xfdm_sync gnuradio::gnuradio-runtime gnuradio-fft)
target_include_directories(gnuradio-xfdm_sync
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
    PUBLIC $<INSTALL_INTERFACE:include>
  )
set_target_properties(gnuradio-xfdm_sync PROPERTIES DEFINE_SYMBOL "gnuradio_xfdm_sync_EXPORTS")

if(APPLE)
    set_target_properties(gnuradio-xfdm_sync PROPERTIES
        INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib"
    )
endif(APPLE)

########################################################################
# Install built library files
########################################################################
include(GrMiscUtils)
GR_LIBRARY_FOO(gnuradio-xfdm_sync)

########################################################################
# Print summary
########################################################################
message(STATUS "Using install prefix: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "Building for version: ${VERSION} / ${LIBVER}")

########################################################################
# Build and register unit test
########################################################################
include(GrTest)

# If your unit tests require special include paths, add them here
#include_directories()
# List all files that contain Boost.UTF unit tests here
list(APPEND test_xfdm_sync_sources
    ${CMAKE_CURRENT_SOURCE_DIR}/test_xfdm_sync.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/qa_xfdm_sync.cc
)
# Anything we need to link to for the unit tests go here
list(APPEND GR_TEST_TARGET_DEPS gnuradio-test38)

if(NOT test_test38_sources)
    MESSAGE(STATUS "No C++ unit tests... skipping")
    return()
endif(NOT test_test38_sources)

foreach(qa_file ${test_xfdm_sync_sources})
    GR_ADD_CPP_TEST("xfdm_sync_${qa_file}"
        ${CMAKE_CURRENT_SOURCE_DIR}/${qa_file}
    )
endforeach(qa_file)

