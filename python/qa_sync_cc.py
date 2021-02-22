#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright 2021 Johannes Demel.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

import numpy as np
from gnuradio import gr, gr_unittest
from sync_cc import sync_cc


def get_pseudo_preamble(size, dtype=np.complex64):
    p = np.ones(size) + 1.0j * np.ones(size)
    return p.astype(dtype)


class qa_sync_cc(gr_unittest.TestCase):
    def setUp(self):
        self.tb = gr.top_block()

    def tearDown(self):
        self.tb = None

    def test_instance(self):
        scorr_threshold_high = 0.98
        scorr_threshold_low = 0.93
        xcorr_threshold = 100.0
        preamble = get_pseudo_preamble(128)

        instance = sync_cc(
            preamble,
            scorr_threshold_high,
            scorr_threshold_low,
            True,
            xcorr_threshold,
            True,
            "frame_foo",
        )

        instance.set_scorr_threshold_high(0.99)
        instance.set_scorr_threshold_low(0.7)
        instance.set_xcorr_threshold(42.42)
        self.assertAlmostEqual(instance.get_scorr_threshold_high(), 0.99)
        self.assertAlmostEqual(instance.get_scorr_threshold_low(), 0.7)
        self.assertAlmostEqual(instance.get_xcorr_threshold(), 42.42)


if __name__ == "__main__":
    gr_unittest.run(qa_sync_cc)
