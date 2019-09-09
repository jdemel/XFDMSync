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

#ifndef INCLUDED_XFDM_SYNC_XCORR_TAGGER_IMPL_H
#define INCLUDED_XFDM_SYNC_XCORR_TAGGER_IMPL_H

#include <xfdm_sync/xcorr_tagger.h>

namespace gr {
  namespace xfdm_sync {

    class xcorr_tagger_impl : public xcorr_tagger
    {
    private:
      gr_complex *d_sequence_fq;

      float d_threshold;
      uint64_t d_peak_idx;
      uint64_t d_last_xcorr_tag_offset;
      bool d_use_sc_rot;
      int d_fft_len;
      int d_sync_seq_len;
      float d_reference_preamble_energy;
      float d_scale_factor;

      pmt::pmt_t d_tag_key;
      const pmt::pmt_t d_scale_factor_key;
      const pmt::pmt_t d_correlation_power_key;
      const pmt::pmt_t d_relative_correlation_power_key;
      const pmt::pmt_t d_rotation_key;
      const pmt::pmt_t d_index_key;
      const pmt::pmt_t d_sc_offset_key;

      gr::fft::fft_complex *d_fft_fwd;
      gr::fft::fft_complex *d_fft_rwd;

      float calculate_signal_energy(const gr_complex* p_in, const int ninput_size);
      float calculate_preamble_attenuation(const gr_complex* p_in);

      void update_peak_tag(pmt::pmt_t &info, const float scale_factor,
                           const float power, const float rel_power,
                           const gr_complex rotation,
                           const uint64_t idx, const uint64_t sc_offset);

    public:
      xcorr_tagger_impl(float threshold, std::vector<gr_complex> sync_sequence, bool use_sc_rot, const std::string &tag_key="frame_start");
      ~xcorr_tagger_impl();

      int work(int noutput_items,
               gr_vector_const_void_star &input_items,
               gr_vector_void_star &output_items);

      void set_threshold(float threshold){d_threshold = threshold;};
    };
  }
}

#endif
