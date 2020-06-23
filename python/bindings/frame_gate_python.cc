/*
 * Copyright 2020 Free Software Foundation, Inc.
 *
 * This file is part of GNU Radio
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

/***********************************************************************************/
/* This file is automatically generated using bindtool and can be manually edited  */
/* The following lines can be configured to regenerate this file during cmake      */
/* If manual edits are made, the following tags should be modified accordingly.    */
/* BINDTOOL_GEN_AUTOMATIC(0)                                                       */
/* BINDTOOL_USE_PYGCCXML(0)                                                        */
/* BINDTOOL_HEADER_FILE(frame_gate.h)                                              */
/* BINDTOOL_HEADER_FILE_HASH(fd278bfcc041b39c62b0095a7e0ca8df)                     */
/***********************************************************************************/

#include <pybind11/complex.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

#include <xfdm_sync/frame_gate.h>
// pydoc.h is automatically generated in the build directory
// #include <frame_gate_pydoc.h>

void bind_frame_gate(py::module& m)
{

    using frame_gate = gr::xfdm_sync::frame_gate;


    py::class_<frame_gate, gr::block, gr::basic_block, std::shared_ptr<frame_gate>>(
        m, "frame_gate")

        .def(py::init(&frame_gate::make),
             py::arg("len_prologue"),
             py::arg("len_epilogue"),
             py::arg("len_symbol"),
             py::arg("symbols_per_frame_min"),
             py::arg("symbols_per_frame_min"),
             py::arg("do_compensate"))


        ;
}
