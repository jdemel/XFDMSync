#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2017 Leonard Göhrs.
# Copyright 2019 Johannes Demel.
#
# This is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
#
# This software is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this software; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.
#

from gnuradio import gr, gr_unittest
from gnuradio import blocks
import xfdm_sync_python as xfdm_sync
import numpy as np


class qa_sc_delay_corr(gr_unittest.TestCase):

    def setUp(self):
        self.tb = gr.top_block()

    def tearDown(self):
        self.tb = None

    def test_001_t(self):
        # generate a random Schmidl&Cox preamble
        d = np.random.normal(0.0, 1, 32) + \
            1j * np.random.normal(0.0, 1, 32)
        s = np.zeros(64, dtype=np.complex)
        s[::2] = d
        sync_seq = np.fft.ifft(s)

        # prepare a data vector with the given preamble
        testdata = np.random.normal(0.0, 1e-3, 6000) + \
            1j * np.random.normal(0.0, 1e-3, 6000)
        testdata[3000:3000 + sync_seq.size] += sync_seq

        src = blocks.vector_source_c(testdata)
        dut = xfdm_sync.sc_delay_corr(sync_seq.size // 2, True)
        snk0 = blocks.vector_sink_c()
        snk1 = blocks.vector_sink_c()
        self.tb.connect(src, dut)
        self.tb.connect((dut, 0), snk0)
        self.tb.connect((dut, 1), snk1)

        # set up fg
        self.tb.run()
        # check data
        res0 = np.array(snk0.data())
        res1 = np.array(snk1.data())

        # make sure the input is passed through with a delay
        self.assertComplexTuplesAlmostEqual(testdata[0:-sync_seq.size],
                                            res0[sync_seq.size:])

        # check if the correlation max is in the desired range
        peakpos = np.argmax(np.abs(res1))
        self.assertGreaterEqual(peakpos, 3060)
        self.assertGreaterEqual(3070, peakpos)


if __name__ == '__main__':
    gr_unittest.run(qa_sc_delay_corr)
