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

#include "xcorr_tagger_impl.h"
#include <gnuradio/io_signature.h>
#include <volk/volk.h>

namespace gr {
namespace xfdm_sync {

xcorr_tagger::sptr xcorr_tagger::make(float threshold,
                                      const std::vector<gr_complex>& sync_sequence,
                                      bool use_sc_rot,
                                      const std::string& tag_key)
{
    return gnuradio::get_initial_sptr(
        new xcorr_tagger_impl(threshold, sync_sequence, use_sc_rot, tag_key));
}

awoseyila_sync_cross_correlator::awoseyila_sync_cross_correlator(
    const std::vector<gr_complex>& sync_sequence)
    : d_sync_sequence_td_len(sync_sequence.size()),
      d_sync_sequence_td(sync_sequence.begin(), sync_sequence.end()),
      d_sync_sequence_fd(calculate_fft_length(sync_sequence.size())),
      d_fft_fwd(std::make_unique<gr::fft::fft_complex>(
          calculate_fft_length(sync_sequence.size()), true, 1)),
      d_fft_rwd(std::make_unique<gr::fft::fft_complex>(
          calculate_fft_length(sync_sequence.size()), false, 1))
{
    /* Transform the reference sequence to the frequency
     * domain for fast crosscorrelation later */
    memset(d_fft_fwd->get_inbuf(), 0, sizeof(gr_complex) * d_fft_fwd->inbuf_length());
    memcpy(d_fft_fwd->get_inbuf(),
           sync_sequence.data(),
           sizeof(gr_complex) * sync_sequence.size());

    d_fft_fwd->execute();

    memcpy(d_sync_sequence_fd.data(),
           d_fft_fwd->get_outbuf(),
           sizeof(gr_complex) * d_fft_fwd->outbuf_length());

    /* Clear the input buffer.
     * It will be assumed to be zeroed later */
    memset(d_fft_fwd->get_inbuf(), 0, sizeof(gr_complex) * d_fft_fwd->inbuf_length());
}

void awoseyila_sync_cross_correlator::fill_fft_input(gr_complex* target,
                                                     const gr_complex* source,
                                                     const gr_complex phase_rotation)
{
    // Be careful! We compute correlation, not cyclic correlation.
    // Leave half of this fun empty!
    const unsigned half_size = fft_size() / 2;
    const unsigned quarter_size = fft_size() / 4;
    gr_complex fq_comp_acc = std::pow(phase_rotation, -1.0f * quarter_size);
    fq_comp_acc /= std::abs(fq_comp_acc);
    // Fill negative time
    volk_32fc_s32fc_x2_rotator_32fc(target + fft_size() - quarter_size,
                                    source,
                                    phase_rotation,
                                    &fq_comp_acc,
                                    quarter_size);

    // Fill positive time
    volk_32fc_s32fc_x2_rotator_32fc(
        target, source + quarter_size, phase_rotation, &fq_comp_acc, quarter_size);
}

void awoseyila_sync_cross_correlator::postprocess_fft_output(
    gr_complex* target, const gr_complex* fft_output, const gr_complex* sc_corr_values)
{
    const unsigned quarter_size = fft_size() / 4;

    // Fill negative time
    volk_32fc_x2_multiply_conjugate_32fc(
        target, fft_output + fft_size() - quarter_size, sc_corr_values, quarter_size);

    // Fill positive time
    volk_32fc_x2_multiply_conjugate_32fc(
        target + quarter_size, fft_output, sc_corr_values + quarter_size, quarter_size);
}

void awoseyila_sync_cross_correlator::cross_correlate(gr_complex* target,
                                                      const gr_complex* stream_values,
                                                      const gr_complex* sc_corr_values,
                                                      const gr_complex phase_rotation)
{
    fill_fft_input(d_fft_fwd->get_inbuf(), stream_values, phase_rotation);

    d_fft_fwd->execute();

    // Fill reverse fft buffer
    volk_32fc_x2_multiply_conjugate_32fc(d_fft_rwd->get_inbuf(),
                                         d_fft_fwd->get_outbuf(),
                                         d_sync_sequence_fd.data(),
                                         fft_size());

    d_fft_rwd->execute();

    /* Use the correlation input to mask the
     * wrong cross-correlation peaks.
     * This will overwrite the padding part of rwd_out.
     * Just to prevent having to allocate a new buffer */

    postprocess_fft_output(target, d_fft_rwd->get_outbuf(), sc_corr_values);
}


xcorr_tagger_impl::xcorr_tagger_impl(float threshold,
                                     const std::vector<gr_complex>& sync_sequence,
                                     bool use_sc_rot,
                                     const std::string& tag_key)
    : gr::sync_block("xcorr_tagger",
                     gr::io_signature::make(2, 2, sizeof(gr_complex)),
                     gr::io_signature::make(1, 2, sizeof(gr_complex))),
      d_correlation_values(calculate_fft_length(sync_sequence.size()) / 2),
      d_correlation_power(calculate_fft_length(sync_sequence.size()) / 2),
      d_threshold(threshold),
      d_use_sc_rot(use_sc_rot),
      d_reference_preamble_energy(
          calculate_signal_energy(sync_sequence.data(), sync_sequence.size())),
      d_tag_key(pmt::mp(tag_key.empty() ? "foo_start" : tag_key)),
      d_correlator(std::make_unique<awoseyila_sync_cross_correlator>(sync_sequence))
{
    set_tag_propagation_policy(TPP_DONT);

    GR_LOG_DEBUG(
        d_logger,
        "correlator: fft_size=" + std::to_string(d_correlator->fft_size()) +
            ", sync_sequence_size=" + std::to_string(d_correlator->sequence_size()));

    bool is_complex = false;
    for (const auto& v : d_correlator->sync_sequence()) {
        if (std::abs(v.imag()) > 1e-4) {
            is_complex = true;
        }
    }
    if (not is_complex) {
        throw std::runtime_error("sync_sequence is probably real! Curse SWIG!");
    }
    /* The block needs access to samples before and after
     * the tag so some delay is necessary */
    set_history(1 + d_correlator->fft_size() / 4);
}

xcorr_tagger_impl::~xcorr_tagger_impl() {}

gr_complex xcorr_tagger_impl::get_frequency_phase_rotation(const pmt::pmt_t& info) const
{
    gr_complex phase_rotation = gr_complex(1.0f, 0.0f);

    if (d_use_sc_rot) {
        phase_rotation = pmt::to_complex(pmt::dict_ref(
            info, pmt::mp("sc_rot"), pmt::from_complex(gr_complex(1.0f, 0.0f))));
        phase_rotation /= std::abs(phase_rotation);
    }
    return phase_rotation;
}

float xcorr_tagger_impl::calculate_signal_energy(const gr_complex* p_in,
                                                 const int ninput_size) const
{
    gr_complex energy = gr_complex(0.0, 0.0);
    volk_32fc_x2_conjugate_dot_prod_32fc(&energy, p_in, p_in, ninput_size);
    return energy.real();
}

float xcorr_tagger_impl::calculate_preamble_attenuation(const gr_complex* p_in) const
{
    return std::sqrt(d_reference_preamble_energy /
                     calculate_signal_energy(p_in, d_correlator->sequence_size()));
}

void xcorr_tagger_impl::update_peak_tag_info(pmt::pmt_t& info,
                                             const float scale_factor,
                                             const float power,
                                             const float rel_power,
                                             const gr_complex rotation,
                                             const uint64_t idx,
                                             const uint64_t sc_offset,
                                             const uint64_t tag_offset) const
{
    info = pmt::dict_add(info, d_scale_factor_key, pmt::from_double(scale_factor));

    info = pmt::dict_add(info, d_correlation_power_key, pmt::from_double(power));

    info = pmt::dict_add(
        info, d_relative_correlation_power_key, pmt::from_double(rel_power));

    info = pmt::dict_add(info, d_rotation_key, pmt::from_complex(rotation));

    info = pmt::dict_add(info, d_index_key, pmt::from_uint64(idx));

    info = pmt::dict_add(info, d_sc_offset_key, pmt::from_uint64(sc_offset));
    info = pmt::dict_add(info, d_xc_offset_key, pmt::from_uint64(tag_offset));
}

float xcorr_tagger_impl::calculate_average_correlation_power(
    const float* correlation_values, const unsigned size) const
{
    float avg_fft_power = 0;
    volk_32f_accumulator_s32f(&avg_fft_power, correlation_values, size);
    return avg_fft_power / size;
}

uint32_t xcorr_tagger_impl::find_index_max(const float* values, const unsigned size) const
{
    uint32_t peak_idx_rel = 0;
    volk_32f_index_max_32u(&peak_idx_rel, values, size);
    return peak_idx_rel;
}


int xcorr_tagger_impl::work(int noutput_items,
                            gr_vector_const_void_star& input_items,
                            gr_vector_void_star& output_items)
{
    const gr_complex* in_pass_history = (const gr_complex*)input_items[0];
    const gr_complex* in_corr_history = (const gr_complex*)input_items[1];
    gr_complex* out_pass = (gr_complex*)output_items[0];

    const unsigned block_delay = history() - 1;

    const gr_complex* in_pass = in_pass_history + block_delay;
    const gr_complex* in_corr = in_corr_history + block_delay;

    memcpy(out_pass, in_pass, sizeof(gr_complex) * noutput_items);
    if (output_items.size() > 1) {
        gr_complex* out_corr = (gr_complex*)output_items[1];
        memcpy(out_corr, in_corr, sizeof(gr_complex) * noutput_items);
    }

    const unsigned fft_len = d_correlator->fft_size();
    const unsigned half_fft_len = d_correlator->fft_size() / 2;
    const unsigned quarter_fft_len = d_correlator->fft_size() / 4;
    const unsigned three_quarter_fft_len = half_fft_len + quarter_fft_len;

    const uint64_t tag_reg_start = nitems_read(0);
    const uint64_t tag_reg_end = nitems_read(0) + noutput_items;
    std::vector<tag_t> tags;
    get_tags_in_range(tags, 0, tag_reg_start, tag_reg_end, d_tag_key);
    std::sort(tags.begin(), tags.end(), tag_t::offset_compare);

    for (const tag_t& tag : tags) {
        if (tag.offset <= d_last_sc_tag_offset) {
            continue;
        }
        const uint64_t buffer_tag_offset = tag.offset - nitems_read(0);
        if ((int64_t)buffer_tag_offset >
            (int64_t)(noutput_items - (int64_t)three_quarter_fft_len)) {
            noutput_items = buffer_tag_offset - half_fft_len;
            break;
        }
        d_last_sc_tag_offset = tag.offset;
        const int64_t buf_ptr_offset = buffer_tag_offset - quarter_fft_len;

        /* Apply frequency offset correction based on input
         * from the sc_tagger block */
        const gr_complex phase_rotation = get_frequency_phase_rotation(tag.value);
        d_correlator->cross_correlate(d_correlation_values.data(),
                                      in_pass + buf_ptr_offset,
                                      in_corr + buf_ptr_offset,
                                      phase_rotation);

        peak_t peak = find_correlation_peak(d_correlation_values, half_fft_len);

        if (peak.relative_power > d_threshold) {
            const int32_t peak_idx = peak.offset - quarter_fft_len;
            const uint64_t raw_peak_offset = tag.offset + peak_idx;
            // if (raw_peak_offset < nitems_read(0)) {
            //     int64_t past =  buffer_tag_offset + peak_idx;
            //     GR_LOG_ERROR(d_logger,
            //                  "LIVING IN THE PAST! " + std::to_string(tag.offset +
            //                  peak_idx) +
            //                      "  < " + std::to_string(nitems_read(0)) + ", samples:
            //                      " + std::to_string(past));
            //     GR_LOG_ERROR(d_logger,
            //                  "tag.offset=" + std::to_string(tag.offset) +
            //                      ", buffer_offset=" + std::to_string(buffer_tag_offset)
            //                      +
            //                      ", ptr_offset" + std::to_string(buf_ptr_offset) + ",
            //                      peak=" + std::to_string(peak_idx));
            // } else
            if (raw_peak_offset > nitems_read(0) + noutput_items) {
                GR_LOG_ERROR(
                    d_logger,
                    "INTO THE FUTURE! " + std::to_string(raw_peak_offset) + "  > " +
                        std::to_string(nitems_read(0) + noutput_items) + ", samples: " +
                        std::to_string(raw_peak_offset - nitems_read(0) + noutput_items));
                GR_LOG_ERROR(d_logger,
                             "tag.offset=" + std::to_string(tag.offset) +
                                 ", buffer_offset=" + std::to_string(buffer_tag_offset) +
                                 ", ptr_offset" + std::to_string(buf_ptr_offset) +
                                 "nout=" + std::to_string(noutput_items));
            }
            const uint64_t peak_offset = std::max(raw_peak_offset, nitems_read(0));
            if (peak_offset <= d_last_xcorr_tag_offset) {
                continue;
            }
            d_last_xcorr_tag_offset = peak_offset;

            const int64_t frame_buffer_start = peak_offset - nitems_written(0);
            d_scale_factor = calculate_preamble_attenuation(in_pass + frame_buffer_start);

            pmt::pmt_t info = tag.value;
            update_peak_tag_info(info,
                                 d_scale_factor,
                                 peak.power,
                                 peak.relative_power,
                                 peak.value / std::abs(peak.value),
                                 d_peak_idx,
                                 tag.offset,
                                 peak_offset);

            add_item_tag(0, peak_offset, d_tag_key, info);
            d_peak_idx++;
        }
    }

    get_tags_in_range(tags, 0, tag_reg_start, tag_reg_end);
    for (auto t : tags) {
        if (t.key != d_tag_key) {
            add_item_tag(0, t);
        }
    }
    return noutput_items;
}
} // namespace xfdm_sync
} // namespace gr
