#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright 2021 Johannes Demel.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

import numpy as np
from gnuradio import gr
import xfdm_sync


class sync_cc(gr.hier_block2):
    """Multicarrier stream sync

    Hierarchical block to synchronize to one antenna stream.
    """

    def __init__(
        self,
        preamble=np.ones(128),
        scorr_threshold_high=0.98,
        scorr_threshold_low=0.93,
        scorr_normalize=True,
        xcorr_threshold=100.0,
        xcorr_compensate_freq_offset=True,
        sync_tag_key="frame_start",
    ):
        gr.hier_block2.__init__(
            self,
            "sync_cc",
            gr.io_signature(1, 1, gr.sizeof_gr_complex),  # Input signature
            gr.io_signature(4, 4, gr.sizeof_gr_complex),
        )  # Output signature

        self.scorr_threshold_high = scorr_threshold_high
        self.scorr_threshold_low = scorr_threshold_low
        self.xcorr_threshold = xcorr_threshold

        preamble_len = len(preamble)
        scorr_delay = preamble_len // 2

        self.sc_delay_corr = xfdm_sync.sc_delay_corr(scorr_delay, scorr_normalize)

        self.sc_tagger = xfdm_sync.sc_tagger(
            scorr_threshold_low,
            scorr_threshold_high,
            scorr_delay,
            sync_tag_key,
        )

        self.xcorr_tagger = xfdm_sync.xcorr_tagger(
            xcorr_threshold,
            preamble,
            xcorr_compensate_freq_offset,
            sync_tag_key,
        )

        self.connect(
            (self, 0),
            (self.sc_delay_corr, 0),
            (self.sc_tagger, 0),
            (self.xcorr_tagger, 0),
            (self, 0),
        )

        self.connect(
            (self.sc_delay_corr, 1),
            (self.sc_tagger, 1),
            (self.xcorr_tagger, 1),
            (self, 1),
        )

        # Connect debug outputs!
        self.connect(
            (self.sc_tagger, 0),
            (self, 2),
        )
        self.connect(
            (self.sc_tagger, 1),
            (self, 3),
        )

    def get_scorr_threshold_high(self):
        return self.scorr_threshold_high

    def set_scorr_threshold_high(self, scorr_threshold_high):
        self.scorr_threshold_high = scorr_threshold_high
        self.sc_tagger.set_threshold_high(scorr_threshold_high)

    def get_scorr_threshold_low(self):
        return self.scorr_threshold_low

    def set_scorr_threshold_low(self, scorr_threshold_low):
        self.scorr_threshold_low = scorr_threshold_low
        self.sc_tagger.set_threshold_low(scorr_threshold_low)

    def get_xcorr_threshold(self):
        return self.xcorr_threshold

    def set_xcorr_threshold(self, xcorr_threshold):
        self.xcorr_threshold = xcorr_threshold
        self.xcorr_tagger.set_threshold(xcorr_threshold)
