#!/usr/bin/env python2
# -*- coding: utf-8 -*-
#
# Copyright 2017 Leonard Göhrs.
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

from gnuradio import gr, blocks

class burstsilence_corr(gr.hier_block2):
    """
    docstring for block burstsilence_corr
    """
    def __init__(self, preamble_length):
        gr.hier_block2.__init__(
            self, "burstsilence_corr",
            gr.io_signature(1, 1, gr.sizeof_gr_complex),
            gr.io_signature(2, 2, gr.sizeof_gr_complex)
        )

        half_length= int(preamble_length)/2

        # Blocks
        self.equiv_delay= blocks.delay(gr.sizeof_gr_complex, preamble_length)

        self.to_power = blocks.complex_to_mag_squared(1)
        self.pw_delay = blocks.delay(gr.sizeof_float, half_length)
        self.sub_sigref = blocks.sub_ff(1)
        self.avg = blocks.moving_average_ff(half_length, 1, 4096)

        self.null_src= blocks.null_source(gr.sizeof_float)
        self.clamp = blocks.max_ff(1,1)
        self.to_complex= blocks.float_to_complex(1)

        # Connections
        self.connect((self, 0), (self.to_power, 0))
        self.connect((self, 0), (self.equiv_delay, 0))

        self.connect((self.to_power, 0), (self.pw_delay, 0))
        self.connect((self.to_power, 0), (self.sub_sigref, 1))

        self.connect((self.pw_delay, 0), (self.sub_sigref, 0))

        self.connect((self.sub_sigref, 0), (self.avg, 0))

        self.connect((self.avg, 0), (self.clamp, 0))
        self.connect((self.null_src, 0), (self.clamp, 1))

        self.connect((self.clamp, 0), (self.to_complex, 0))
        self.connect((self.null_src, 0), (self.to_complex, 1))

        self.connect((self.equiv_delay, 0), (self, 0))
        self.connect((self.to_complex, 0), (self, 1))
