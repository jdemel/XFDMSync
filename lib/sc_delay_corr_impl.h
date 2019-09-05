/* -*- c++ -*- */
/*
 * Copyright 2017 Leonard Göhrs.
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifndef INCLUDED_XFDM_SYNC_SC_DELAY_CORR_IMPL_H
#define INCLUDED_XFDM_SYNC_SC_DELAY_CORR_IMPL_H

#include <xfdm_sync/sc_delay_corr.h>

namespace gr {
  namespace xfdm_sync {

    class sc_delay_corr_impl : public sc_delay_corr
    {
    private:
      const int d_seq_len;
      const bool d_normalize;

      gr_complex *d_corr_windows[3];
      float *d_norm_windows[4];

    public:
      sc_delay_corr_impl(int seq_len, bool normalize);
      ~sc_delay_corr_impl();

      int work(int noutput_items,
               gr_vector_const_void_star &input_items,
               gr_vector_void_star &output_items);
    };
  }
}

#endif
