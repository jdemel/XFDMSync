#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright 2020 Johannes Demel.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

from gnuradio import gr, gr_unittest
from gnuradio import blocks
import xfdm_sync_python as xfdm_sync
import pmt
import numpy as np


class qa_sync_tag_align_cc(gr_unittest.TestCase):
    def setUp(self):
        self.tb = gr.top_block()

    def tearDown(self):
        self.tb = None

    def test_001_t(self):
        sync_key = "foo"
        n_streams = 4

        tagger = xfdm_sync.sync_tag_align_cc(n_streams, sync_key)

        tags = []
        for i in range(3):
            tag = gr.python_to_tag(
                (3000 + 30 * i, pmt.intern(sync_key), pmt.PMT_T, pmt.intern("qa"))
            )
            tags.append(tag)

        srcs = []
        snks = []
        testdata = []
        for i in range(n_streams):
            d = (np.arange(12000) + i) * (i + 1)
            d = d.astype(np.complex64) * (1.0 + 1.0j)
            testdata.append(d)
            src = blocks.vector_source_c(d.tolist())
            snk = blocks.vector_sink_c()
            self.tb.connect(src, (tagger, i), snk)
            srcs.append(src)
            snks.append(snk)
        srcs[0].set_data(testdata[0].tolist(), tags)

        self.tb.run()

        # check data
        for d, s in zip(testdata, snks):
            r = np.array(s.data())
            self.assertComplexTuplesAlmostEqual(r, d)

        for s in snks:
            for tag, t in zip(tags, s.tags()):
                self.assertEqual(t.offset, tag.offset)
                self.assertTrue(pmt.eq(t.key, tag.key))
                self.assertTrue(pmt.eq(t.value, tag.value))


if __name__ == "__main__":
    gr_unittest.run(qa_sync_tag_align_cc)
