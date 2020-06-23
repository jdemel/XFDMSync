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
import pmt
import xfdm_sync_python as xfdm_sync
import numpy as np

# import os
# print('Blocked waiting for GDB attach (pid = %d)' % (os.getpid(),))
# input('Press Enter to continue: ')


class qa_xcorr_tagger(gr_unittest.TestCase):
    def setUp(self):
        self.tb = gr.top_block()

    def tearDown(self):
        self.tb = None

    def test_001_t(self):
        # generate a random Schmidl&Cox preamble
        d = np.random.normal(0.0, 1, 32) + 1j * np.random.normal(0.0, 1, 32)
        s = np.zeros(64, dtype=np.complex)
        s[::2] = d
        sync_seq = np.fft.ifft(s).astype(np.complex64)
        sync_seq /= np.sqrt(np.mean(sync_seq ** 2))
        sync_seq = sync_seq.astype(np.complex64)

        # prepare a data vector with the given preamble
        testdata = np.random.normal(0.0, 1e-3, 6000) + 1j * np.random.normal(
            0.0, 1e-3, 6000
        )
        testdata[3000 : 3000 + sync_seq.size] = sync_seq

        sc_data = np.zeros_like(testdata)
        sc_data[
            3000 - sync_seq.size // 2 : 3000 + 1 + sync_seq.size // 2
        ] += np.concatenate(
            (np.arange(1 + sync_seq.size // 2), np.arange(sync_seq.size // 2)[::-1])
        )
        sc_data /= np.max(np.abs(sc_data))
        tag_value = pmt.dict_add(
            pmt.make_dict(), pmt.intern("sc_rot"), pmt.from_complex(1.0 + 0.0j)
        )
        tag = gr.python_to_tag(
            (3000, pmt.intern("frame_start"), tag_value, pmt.intern("qa"))
        )
        tags = [
            tag,
            tag,
        ]
        src0 = blocks.vector_source_c(testdata, tags=tags)
        src1 = blocks.vector_source_c(sc_data, tags=tags)

        dut = xfdm_sync.xcorr_tagger(30.0, sync_seq.tolist(), True, "frame_start")
        dut.set_threshold(31.)
        snk0 = blocks.vector_sink_c()
        snk1 = blocks.vector_sink_c()
        self.tb.connect((src0, 0), (dut, 0))
        self.tb.connect((src1, 0), (dut, 1))
        self.tb.connect((dut, 0), snk0)
        self.tb.connect((dut, 1), snk1)

        # # set up fg
        self.tb.run()
        # check data
        res0 = np.array(snk0.data())
        res1 = np.array(snk1.data())

        print(np.argmax(np.abs(sc_data)), np.argmax(np.abs(res1)))

        self.assertComplexTuplesAlmostEqual(testdata, res0, 6)
        self.assertComplexTuplesAlmostEqual(sc_data, res1, 6)

        tgs = snk0.tags()
        self.assertEqual(len(tgs), 1)
        t = tgs[0]
        self.assertEqual(pmt.symbol_to_string(t.key), "frame_start")
        self.assertEqual(t.offset, 3000)

        tag_value = t.value
        self.assertEqual(
            pmt.to_uint64(
                pmt.dict_ref(tag_value, pmt.intern("xcorr_offset"), pmt.from_uint64(0))
            ),
            3000,
        )
        self.assertEqual(
            pmt.to_uint64(
                pmt.dict_ref(tag_value, pmt.intern("sc_offset"), pmt.from_uint64(0))
            ),
            3000,
        )

        print(tag_value)

        self.assertComplexTuplesAlmostEqual(dut.sync_sequence(), sync_seq)


if __name__ == "__main__":
    gr_unittest.run(qa_xcorr_tagger)
