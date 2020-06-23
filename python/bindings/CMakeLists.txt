# Copyright 2020 Free Software Foundation, Inc.
#
# This file is part of GNU Radio
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

GR_PYTHON_CHECK_MODULE_RAW(
    "pygccxml"
    "import pygccxml"
    PYGCCXML_FOUND
    )

include(GrPybind)

########################################################################
# Python Bindings
########################################################################

list(APPEND xfdm_sync_python_files
    frame_gate_python.cc
    sc_delay_corr_python.cc
    sc_tagger_python.cc
    xcorr_tagger_python.cc
    sync_tag_align_cc_python.cc
    python_bindings.cc)

GR_PYBIND_MAKE_OOT(xfdm_sync
   ../..
   gr::xfdm_sync
   "${xfdm_sync_python_files}")

install(TARGETS xfdm_sync_python DESTINATION ${GR_PYTHON_DIR}/xfdm_sync COMPONENT pythonapi)
