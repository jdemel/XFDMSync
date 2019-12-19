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

#ifndef INCLUDED_XFDM_SYNC_SC_TAGGER_IMPL_H
#define INCLUDED_XFDM_SYNC_SC_TAGGER_IMPL_H

#include <xfdm_sync/sc_tagger.h>

namespace gr {
  namespace xfdm_sync {

    class sc_tagger_impl : public sc_tagger
    {
    private:
      float d_thres_low_sq;
      float d_thres_high_sq;

      int d_seq_len;
      int d_lookahead;

      float* d_norm_array;
      int d_norm_array_length;

      pmt::pmt_t d_tag_key;

      const pmt::pmt_t d_correlation_power_key;
      const pmt::pmt_t d_symbol_rotation_key;
      const pmt::pmt_t d_index_key;
      const pmt::pmt_t d_time_key;
      pmt::pmt_t make_peak_tag(const double corr_power,
                               const gr_complex rot_per_sym,
                               const uint64_t idx);

      struct peak_info{
        uint64_t id;
        bool am_inside;
        uint64_t abs_idx;
        gr_complex corr;
        float corr_pw_sq;
      };

      size_t d_frontend_offset;
      double d_frontend_freq;
      double d_frontend_samp_rate;
      uint64_t d_frontend_secs;
      double d_frontend_fracs;
      size_t d_frontend_ticks;
      void update_frontend_info(const std::vector<tag_t> &tags);

      peak_info d_peak;

      void update_norm_array_length(const int array_len);

    public:
      sc_tagger_impl(float thres_low, float thres_high, int seq_len, const std::string &tag_key);
      ~sc_tagger_impl();

      int work(int noutput_items,
               gr_vector_const_void_star &input_items,
               gr_vector_void_star &output_items);

      void set_threshold_low(float threshold)
      {
        d_thres_low_sq = threshold * threshold;
        if (d_thres_low_sq > d_thres_high_sq){
          std::cout << "WARNING: low threshold > high threshold. setting low == high!\n";
          d_thres_low_sq = d_thres_high_sq;
        }
        std::cout << "set_threshold_low(" << threshold << ") = " << d_thres_low_sq << std::endl;
      };

      void set_threshold_high(float threshold)
      {
        d_thres_high_sq = threshold * threshold;
        if (d_thres_low_sq > d_thres_high_sq){
          std::cout << "WARNING: high threshold < low threshold. setting high == low!\n";
          d_thres_high_sq = d_thres_low_sq;
        }
        std::cout << "set_threshold_high(" << threshold << ") = " << d_thres_high_sq << std::endl;
      };
    };
  }
}

#endif
