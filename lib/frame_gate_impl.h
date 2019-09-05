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

#ifndef INCLUDED_XFDM_SYNC_FRAME_GATE_IMPL_H
#define INCLUDED_XFDM_SYNC_FRAME_GATE_IMPL_H

#include <xfdm_sync/frame_gate.h>

namespace gr {
  namespace xfdm_sync {
    class frame_gate_impl : public frame_gate
    {
    private:
      struct {
        uint64_t id;
        bool active;

        int samples_consumed;
        int samples_max;
        int samples_min;

        gr_complex fq_comp_acc;
        gr_complex fq_comp_rot;
      } d_frame;

      struct {
        int len_prologue;
        int len_epilogue;
        int len_symbol;
        int symbols_per_frame_min;
        int symbols_per_frame_max;
        bool do_compensate;
      } d_parameters;

    public:
      frame_gate_impl(int len_prologue, int len_epilogue, int len_symbol,
                      int symbols_per_frame_min, int symbols_per_frame_max,
                      bool do_compensate);
      ~frame_gate_impl();

      void on_frame_hint(pmt::pmt_t msg);

      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items);
    };
  }
}

#endif
