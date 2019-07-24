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
import numpy as np


class fqsweep_corr(gr.hier_block2):
    """
    docstring for block fqsweep_corr
    """
    def __init__(self, preamble_length):
        gr.hier_block2.__init__(
            self, "fqsweep_corr",
            gr.io_signature(1, 1, gr.sizeof_gr_complex),
            gr.io_signature(2, 2, gr.sizeof_gr_complex)
        )

        despread= self.gen_despread(
            preamble_length,
            np.pi * 3.0 / 4.0
        )

        # Blocks
        self.equiv_delay= blocks.delay(gr.sizeof_gr_complex, preamble_length + 2)

        self.despread_src= blocks.vector_source_c(despread, True, 1, [])
        self.despread_mul= blocks.multiply_conjugate_cc(1)

        self.deriv_0_delay= blocks.delay(gr.sizeof_gr_complex, 1)
        self.deriv_0_mul= blocks.multiply_conjugate_cc(1)

        self.deriv_1_delay= blocks.delay(gr.sizeof_gr_complex, 1)
        self.deriv_1_mul= blocks.multiply_conjugate_cc(1)

        self.avg= blocks.moving_average_cc(preamble_length, 1.0/preamble_length, 2**16)

        # Connections

        # pass output
        self.connect((self, 0), (self.equiv_delay, 0))
        self.connect((self.equiv_delay, 0), (self, 0))

        # despread
        self.connect((self, 0), (self.despread_mul, 0))
        self.connect((self.despread_src, 0), (self.despread_mul, 1))

        # first phase derivate (frequency)
        self.connect((self.despread_mul, 0), (self.deriv_0_mul, 0))
        self.connect((self.despread_mul, 0), (self.deriv_0_delay, 0))
        self.connect((self.deriv_0_delay, 0), (self.deriv_0_mul, 1))

        # second phase derivate (change of frequency)
        self.connect((self.deriv_0_mul, 0), (self.deriv_1_mul, 0))
        self.connect((self.deriv_0_mul, 0), (self.deriv_1_delay, 0))
        self.connect((self.deriv_1_delay, 0), (self.deriv_1_mul, 1))

        # average
        self.connect((self.deriv_1_mul, 0), (self.avg, 0))
        self.connect((self.avg, 0), (self, 1))

    def gen_despread(self, preamble_length, span):
        freqs= np.linspace(-span, span, preamble_length)
        phases= np.add.accumulate(freqs)
        samples= np.exp(1j * phases)

        return samples
