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

#include <gnuradio/fft/fft.h>
#include <volk/volk_alloc.hh>
#include <xfdm_sync/xcorr_tagger.h>
#include <memory>

namespace gr {
namespace xfdm_sync {

class xcorr_tagger_impl : public xcorr_tagger
{
private:
    volk::vector<gr_complex> d_sequence_fq;

    float d_threshold;
    uint64_t d_peak_idx;
    uint64_t d_last_xcorr_tag_offset;
    const bool d_use_sc_rot;
    int d_fft_len;
    const int d_sync_seq_len;
    const float d_reference_preamble_energy;
    float d_scale_factor;

    const pmt::pmt_t d_tag_key;
    const pmt::pmt_t d_scale_factor_key = pmt::mp("scale_factor");
    const pmt::pmt_t d_correlation_power_key = pmt::mp("xcorr_power");
    const pmt::pmt_t d_relative_correlation_power_key = pmt::mp("xcorr_rel_power");
    const pmt::pmt_t d_rotation_key = pmt::mp("xcorr_rot");
    const pmt::pmt_t d_index_key = pmt::mp("xcorr_idx");
    const pmt::pmt_t d_sc_offset_key = pmt::mp("sc_offset");
    const pmt::pmt_t d_xc_offset_key = pmt::mp("xcorr_offset");

    std::unique_ptr<gr::fft::fft_complex> d_fft_fwd;
    std::unique_ptr<gr::fft::fft_complex> d_fft_rwd;

    float calculate_signal_energy(const gr_complex* p_in, const int ninput_size);
    float calculate_preamble_attenuation(const gr_complex* p_in);

    void update_peak_tag_info(pmt::pmt_t& info,
                              const float scale_factor,
                              const float power,
                              const float rel_power,
                              const gr_complex rotation,
                              const uint64_t idx,
                              const uint64_t sc_offset,
                              const uint64_t tag_offset);

    gr_complex get_frequency_phase_rotation(const pmt::pmt_t& info);

public:
    xcorr_tagger_impl(float threshold,
                      std::vector<gr_complex> sync_sequence,
                      bool use_sc_rot,
                      const std::string& tag_key = "frame_start");
    ~xcorr_tagger_impl();

    int work(int noutput_items,
             gr_vector_const_void_star& input_items,
             gr_vector_void_star& output_items);

    void set_threshold(float threshold)
    {
        std::cout << "set_threshold=" << threshold << std::endl;
        d_threshold = threshold;
    };
};
} // namespace xfdm_sync
} // namespace gr

#endif
