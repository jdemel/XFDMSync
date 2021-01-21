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

/* Find a power-of-two fft length that can fit
 * the synchronization pattern, an equally sized
 * padding and a bit of slack */
int calculate_fft_length(const unsigned sequence_length)
{
    int fft_len = 4;
    while (fft_len < 3 * sequence_length) {
        fft_len *= 2;
    }
    return fft_len;
};

class awoseyila_sync_cross_correlator
{
private:
    const int d_sync_sequence_td_len;
    const volk::vector<gr_complex> d_sync_sequence_td;
    volk::vector<gr_complex> d_sync_sequence_fd;
    std::unique_ptr<gr::fft::fft_complex_fwd> d_fft_fwd;
    std::unique_ptr<gr::fft::fft_complex_rev> d_fft_rwd;
    void fill_fft_input(gr_complex* target,
                        const gr_complex* source,
                        const gr_complex phase_rotation);
    void postprocess_fft_output(gr_complex* target,
                                const gr_complex* fft_output,
                                const gr_complex* sc_corr_values);

public:
    awoseyila_sync_cross_correlator(const std::vector<gr_complex>& sync_sequence);
    // ~awoseyila_sync_cross_correlator();
    void cross_correlate(gr_complex* target,
                         const gr_complex* stream_values,
                         const gr_complex* sc_corr_values,
                         const gr_complex phase_rotation);
    int fft_size() const { return d_fft_fwd->inbuf_length(); }
    int sequence_size() const { return d_sync_sequence_td_len; }
    volk::vector<gr_complex> sync_sequence() const { return d_sync_sequence_td; }
};

class xcorr_tagger_impl : public xcorr_tagger
{
private:
    struct peak_t {
        uint32_t offset;
        gr_complex value;
        float power;
        float relative_power;
    };
    volk::vector<gr_complex> d_correlation_values;
    volk::vector<float> d_correlation_power;

    float d_threshold;
    uint64_t d_peak_idx = 0;
    uint64_t d_last_xcorr_tag_offset = 0;
    uint64_t d_last_sc_tag_offset = 0;
    const uint64_t d_minimum_tag_offset_distance = 2;
    const bool d_use_sc_rot;

    const float d_reference_preamble_energy;
    float d_scale_factor = 1.0f;

    const pmt::pmt_t d_tag_key;
    const pmt::pmt_t d_scale_factor_key = pmt::mp("scale_factor");
    const pmt::pmt_t d_correlation_power_key = pmt::mp("xcorr_power");
    const pmt::pmt_t d_relative_correlation_power_key = pmt::mp("xcorr_rel_power");
    const pmt::pmt_t d_rotation_key = pmt::mp("xcorr_rot");
    const pmt::pmt_t d_index_key = pmt::mp("xcorr_idx");
    const pmt::pmt_t d_sc_offset_key = pmt::mp("sc_offset");
    const pmt::pmt_t d_sc_index_key = pmt::mp("sc_idx");
    const pmt::pmt_t d_xc_offset_key = pmt::mp("xcorr_offset");

    std::unique_ptr<awoseyila_sync_cross_correlator> d_correlator;

    float calculate_signal_energy(const gr_complex* p_in, const int ninput_size) const;
    float calculate_preamble_attenuation(const gr_complex* p_in) const;

    float calculate_average_correlation_power(const float* correlation_values,
                                              const unsigned size) const;
    uint32_t find_index_max(const float* values, const unsigned size) const;

    peak_t find_correlation_peak(const volk::vector<gr_complex>& values,
                                 const unsigned fft_payload_len)
    {
        /* Calculate the mag^2 of the correlation output
         * this will again reuse rwd_out as output */
        volk_32fc_magnitude_squared_32f(
            d_correlation_power.data(), d_correlation_values.data(), fft_payload_len);

        // Locate the maximum
        const uint32_t peak_idx_rel =
            find_index_max(d_correlation_power.data(), fft_payload_len);

        // Determine the absolute energy
        const float avg_fft_power = calculate_average_correlation_power(
            d_correlation_power.data(), fft_payload_len);

        gr_complex peak = d_correlation_values[peak_idx_rel];
        float power = d_correlation_power[peak_idx_rel];
        float rel_power = power / avg_fft_power;
        return peak_t{ peak_idx_rel, peak, power, rel_power };
    }

    void update_peak_tag_info(pmt::pmt_t& info,
                              const float scale_factor,
                              const float power,
                              const float rel_power,
                              const gr_complex rotation,
                              const uint64_t idx,
                              const uint64_t sc_offset,
                              const uint64_t tag_offset) const;

    gr_complex get_frequency_phase_rotation(const pmt::pmt_t& info) const;

public:
    xcorr_tagger_impl(float threshold,
                      const std::vector<gr_complex>& sync_sequence,
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

    std::vector<gr_complex> sync_sequence()
    {
        auto s = d_correlator->sync_sequence();
        return std::vector<gr_complex>(s.begin(), s.end());
    }
};
} // namespace xfdm_sync
} // namespace gr

#endif
