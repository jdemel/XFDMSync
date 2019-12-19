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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <volk/volk.h>
#include <gnuradio/io_signature.h>
#include "sc_tagger_impl.h"
#include <chrono>

namespace gr {
  namespace xfdm_sync {

    sc_tagger::sptr
    sc_tagger::make(float thres_low, float thres_high, int seq_len, const std::string &tag_key)
    {
      return gnuradio::get_initial_sptr(new sc_tagger_impl(thres_low, thres_high, seq_len, tag_key));
    }

    sc_tagger_impl::sc_tagger_impl(float thres_low, float thres_high, int seq_len, const std::string &tag_key)
      : gr::sync_block("sc_tagger",
                       gr::io_signature::make(2, 2, sizeof(gr_complex)),
                       gr::io_signature::make(2, 2, sizeof(gr_complex))),
      d_thres_low_sq(thres_low * thres_low),
      d_thres_high_sq(thres_high * thres_high),
      d_seq_len(seq_len),
      d_lookahead(2*seq_len),
      d_norm_array(nullptr),
      d_norm_array_length(0),
      d_correlation_power_key(pmt::mp("sc_corr_power")),
      d_symbol_rotation_key(pmt::mp("sc_rot")),
      d_index_key(pmt::mp("sc_idx")),
      d_time_key(pmt::mp("time"))
    {
      set_threshold_low(thres_low);
      set_threshold_high(thres_high);
      set_tag_propagation_policy(TPP_ONE_TO_ONE);

      set_history(d_lookahead + 1);

      d_peak.id= 0;
      d_peak.am_inside= false;

      if(tag_key.empty()){
        d_tag_key = pmt::string_to_symbol("frame_start");
      }
      else{
        d_tag_key = pmt::string_to_symbol(tag_key);
      }

      update_norm_array_length(1024); // some sensible default.
    }

    sc_tagger_impl::~sc_tagger_impl()
    {
    }

    void
    sc_tagger_impl::update_norm_array_length(const int array_len)
    {
      if(array_len > d_norm_array_length){
        volk_free(d_norm_array);
        d_norm_array_length = array_len;
        d_norm_array = (float*) volk_malloc(sizeof(float) * d_norm_array_length,
                                            volk_get_alignment());
      }
    }

    pmt::pmt_t
    sc_tagger_impl::make_peak_tag(const double corr_power,
                             const gr_complex rot_per_sym,
                             const uint64_t idx)
    {
      pmt::pmt_t info = pmt::make_dict();
      info = pmt::dict_add(info,
                           d_correlation_power_key,
                           pmt::from_double(corr_power));

      info = pmt::dict_add(info,
                           d_symbol_rotation_key,
                           pmt::from_complex(rot_per_sym));

      info = pmt::dict_add(info,
                           d_index_key,
                           pmt::from_uint64(idx));

      const auto ticks = std::chrono::high_resolution_clock::now().time_since_epoch().count();
      info = pmt::dict_add(info,
                      d_time_key,
                      pmt::from_long(ticks));
      return info;
    }

    int
    sc_tagger_impl::work(int noutput_items,
                         gr_vector_const_void_star &input_items,
                         gr_vector_void_star &output_items)
    {
      const gr_complex *in_pass_history = (const gr_complex *) input_items[0];
      const gr_complex *in_corr_history = (const gr_complex *) input_items[1];
      gr_complex *out_pass = (gr_complex *) output_items[0];
      gr_complex *out_corr = (gr_complex *) output_items[1];

      const gr_complex *in_pass = &in_pass_history[history() - 1];
      const gr_complex *in_corr = &in_corr_history[history() - 1];

      const float threshold_sq_low = d_thres_low_sq;
      const float threshold_sq_high = d_thres_high_sq;

      /* This block delays by d_lookahead samples */
      memcpy(out_pass, &in_pass[-d_lookahead], sizeof(gr_complex) * noutput_items);
      memcpy(out_corr, &in_corr[-d_lookahead], sizeof(gr_complex) * noutput_items);

      update_norm_array_length(noutput_items);
      volk_32fc_magnitude_squared_32f(d_norm_array, in_corr, noutput_items);
      for(int io_idx = 0; io_idx < noutput_items; io_idx++) {
        const gr_complex corr = in_corr[io_idx];
        const float power_sq = d_norm_array[io_idx];

        /* check if we left the peak with the current sample */
        if(d_peak.am_inside && (power_sq < threshold_sq_low)) {
          d_peak.am_inside = false;
          if(d_peak.abs_idx < nitems_written(0) ||
             d_peak.abs_idx < nitems_read(0) + io_idx){
            continue;
          }

          const float corr_power = std::sqrt(d_peak.corr_pw_sq);
          gr_complex rot_per_sym = std::pow(d_peak.corr, 1.0f / d_seq_len);
          rot_per_sym /= std::abs(rot_per_sym);
          //std::cout << d_peak.abs_idx << ", " << d_peak.id << ", " << threshold_sq_low << " < " << threshold_sq_high << " < " << d_peak.corr_pw_sq << ", " << corr_power << std::endl;

          auto info = make_peak_tag(corr_power, rot_per_sym, d_peak.id);

          add_item_tag(0, d_peak.abs_idx,
                       d_tag_key,
                       info);

          d_peak.id++;
        }

        /* check if we entered the peak with the current sample */
        if(!d_peak.am_inside && (power_sq > threshold_sq_high)) {
          d_peak.am_inside= true;
          d_peak.corr_pw_sq= 0;
        }

        if(d_peak.am_inside && (d_peak.corr_pw_sq < power_sq)) {
          d_peak.abs_idx= nitems_read(0) + io_idx + d_lookahead;
          d_peak.corr= corr;
          d_peak.corr_pw_sq= power_sq;
        }
      }

      return noutput_items;
    }

  }
}
