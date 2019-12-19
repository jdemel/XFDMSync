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
#include <gnuradio/fft/fft.h>
#include <gnuradio/io_signature.h>
#include "xcorr_tagger_impl.h"

namespace gr {
  namespace xfdm_sync {

    xcorr_tagger::sptr
    xcorr_tagger::make(float threshold, std::vector<gr_complex> sync_sequence, bool use_sc_rot, const std::string &tag_key)
    {
      return gnuradio::get_initial_sptr(new xcorr_tagger_impl(threshold, sync_sequence, use_sc_rot, tag_key));
    }

    xcorr_tagger_impl::xcorr_tagger_impl(float threshold, std::vector<gr_complex> sync_sequence, bool use_sc_rot, const std::string &tag_key)
      : gr::sync_block("xcorr_tagger",
                       gr::io_signature::make(2, 2, sizeof(gr_complex)),
                       gr::io_signature::make(1, 2, sizeof(gr_complex))),
      d_threshold(threshold),
      d_peak_idx(0),
      d_use_sc_rot(use_sc_rot),
      d_sync_seq_len(sync_sequence.size()),
      d_scale_factor(1.0f),
      d_scale_factor_key(pmt::mp("scale_factor")),
      d_correlation_power_key(pmt::mp("xcorr_power")),
      d_relative_correlation_power_key(pmt::mp("xcorr_rel_power")),
      d_rotation_key(pmt::mp("xcorr_rot")),
      d_index_key(pmt::mp("xcorr_idx")),
      d_sc_offset_key(pmt::mp("sc_offset")),
      d_last_xcorr_tag_offset(0)
    {
      set_tag_propagation_policy(TPP_DONT);

      int seq_len = sync_sequence.size();
      d_reference_preamble_energy = calculate_signal_energy(&sync_sequence[0], seq_len);

      /* Find a power-of-two fft length that can fit
       * the synchronization pattern, an equally sized
       * padding and a bit of slack */
      for(d_fft_len= 4; d_fft_len < (3*seq_len); d_fft_len*=2);

      /* The block needs access to samples before and after
       * the tag so some delay is necessary */
      set_history(d_fft_len/2);

      /* Allocate space for the fourier-transformed sequence */
      d_sequence_fq = (gr_complex *)volk_malloc(sizeof(gr_complex) * d_fft_len,
                                                volk_get_alignment());

      /* Let the GNURadio wrapper setup some fftw contexts */
      d_fft_fwd= new gr::fft::fft_complex(d_fft_len, true, 1);
      d_fft_rwd= new gr::fft::fft_complex(d_fft_len, false, 1);

      gr_complex *fwd_in= d_fft_fwd->get_inbuf();
      gr_complex *fwd_out= d_fft_fwd->get_outbuf();

      /* Transform the referece sequence to the frequency
       * domain for fast crosscorrelation later */
      memset(fwd_in, 0, sizeof(gr_complex) * d_fft_len);
      for(int i=0; i<seq_len; i++) {
        fwd_in[i]= sync_sequence[i];
      }

      d_fft_fwd->execute();

      memcpy(d_sequence_fq, fwd_out, sizeof(gr_complex) * d_fft_len);

      /* Clear the input buffer.
       * It will be assumed to be zeroed later */
      memset(fwd_in, 0, sizeof(gr_complex) * d_fft_len);

      if(tag_key.empty()){
        d_tag_key = pmt::string_to_symbol("frame_start");
      }
      else{
        d_tag_key = pmt::string_to_symbol(tag_key);
      }
    }

    xcorr_tagger_impl::~xcorr_tagger_impl()
    {
      delete d_fft_fwd;
      delete d_fft_fwd;

      volk_free(d_sequence_fq);
    }

    gr_complex 
    xcorr_tagger_impl::get_frequency_phase_rotation(const pmt::pmt_t &info)
    {
      gr_complex fq_comp_rot = gr_complex(1.0f, 0.0f);

      if(d_use_sc_rot) {
        pmt::pmt_t sc_rot = pmt::dict_ref(info,
                                          pmt::mp("sc_rot"),
                                          pmt::PMT_NIL);

        if(pmt::is_complex(sc_rot)) {
          fq_comp_rot = std::conj(pmt::to_complex(sc_rot));
          fq_comp_rot /= std::abs(fq_comp_rot);
        }
      }
      return fq_comp_rot;
    }

    float
    xcorr_tagger_impl::calculate_signal_energy(const gr_complex* p_in, const int ninput_size)
    {
      gr_complex energy = gr_complex(0.0, 0.0);
      volk_32fc_x2_conjugate_dot_prod_32fc(&energy, p_in, p_in, ninput_size);
      return energy.real();
    }

    float
    xcorr_tagger_impl::calculate_preamble_attenuation(const gr_complex* p_in)
    {
      return std::sqrt(d_reference_preamble_energy / calculate_signal_energy(p_in, d_sync_seq_len));
    }

    void
    xcorr_tagger_impl::update_peak_tag(pmt::pmt_t &info, const float scale_factor,
                                       const float power, const float rel_power,
                                       const gr_complex rotation,
                                       const uint64_t idx, const uint64_t sc_offset)
    {
      info = pmt::dict_add(info, d_scale_factor_key,
                           pmt::from_double(scale_factor));

      info = pmt::dict_add(info, d_correlation_power_key,
                           pmt::from_double(power));

      info = pmt::dict_add(info, d_relative_correlation_power_key,
                           pmt::from_double(rel_power));

      info = pmt::dict_add(info, d_rotation_key,
                           pmt::from_complex(rotation));

      info = pmt::dict_add(info, d_index_key,
                           pmt::from_uint64(idx));

      info = pmt::dict_add(info, d_sc_offset_key,
                           pmt::from_uint64(sc_offset));
    }

    int
    xcorr_tagger_impl::work(int noutput_items,
                            gr_vector_const_void_star &input_items,
                            gr_vector_void_star &output_items)
    {
      const gr_complex *in_pass_history = (const gr_complex *) input_items[0];
      const gr_complex *in_corr_history = (const gr_complex *) input_items[1];
      gr_complex *out_pass = (gr_complex *) output_items[0];
      //gr_complex *out_corr = (gr_complex *) output_items[1];

      /* The history is d_fft_len/2 samples long.
       * The indexing done below allows us to read in_pass and in_corr
       * from -d_fft_len/4 to in_len+d_fft_len/4. */
      int block_delay= d_fft_len/4;
      const gr_complex *in_pass= &in_pass_history[block_delay];
      const gr_complex *in_corr= &in_corr_history[block_delay];

      memcpy(out_pass, in_pass, sizeof(gr_complex) * noutput_items);
      if(output_items.size() > 1){
        gr_complex *out_corr = (gr_complex *) output_items[1];
        memcpy(out_corr, in_corr, sizeof(gr_complex) * noutput_items);
      }

      const int fft_payload_len = d_fft_len / 2;
      const int fft_payload_half_len = fft_payload_len / 2;

      uint64_t tag_reg_start = ((int64_t)nitems_read(0) > block_delay) ? (nitems_read(0) - block_delay) : 0;
      // uint64_t tag_reg_start = (int64_t)nitems_read(0);
      uint64_t tag_reg_end = nitems_read(0) + noutput_items - block_delay; /* TODO: this might break for large fft_lens*/

      std::vector<tag_t> tags;
      get_tags_in_range(tags, 0,
                        tag_reg_start, tag_reg_end,
                        d_tag_key);

      for(tag_t tag: tags) {
        // remove_item_tag(0, tag);
        int tag_center= tag.offset + history() - nitems_read(0);

        pmt::pmt_t info = tag.value;
        uint64_t sc_idx = pmt::to_uint64(pmt::dict_ref(info, pmt::mp("sc_idx"), pmt::PMT_NIL));

        gr_complex *fwd_in= d_fft_fwd->get_inbuf();
        gr_complex *fwd_out= d_fft_fwd->get_outbuf();
        gr_complex *rwd_in= d_fft_rwd->get_inbuf();
        gr_complex *rwd_out= d_fft_rwd->get_outbuf();

        /* Apply frequency offset correction based on input
         * from the sc_tagger block */
        gr_complex fq_comp_rot = get_frequency_phase_rotation(info);

        gr_complex fq_comp_acc= std::pow(fq_comp_rot, -1.0f * fft_payload_half_len);
        fq_comp_acc/= std::abs(fq_comp_acc);

        // Fill negative time
        volk_32fc_s32fc_x2_rotator_32fc(&fwd_in[d_fft_len - fft_payload_half_len],
                                        &in_pass_history[tag_center - fft_payload_half_len],
                                        fq_comp_rot,
                                        &fq_comp_acc,
                                        fft_payload_half_len);

        // Fill positive time
        volk_32fc_s32fc_x2_rotator_32fc(&fwd_in[0],
                                        &in_pass_history[tag_center],
                                        fq_comp_rot,
                                        &fq_comp_acc,
                                        fft_payload_half_len);

        d_fft_fwd->execute();

        // Fill reverse fft buffer
        volk_32fc_x2_multiply_conjugate_32fc(rwd_in, fwd_out,
                                             d_sequence_fq, d_fft_len);

        d_fft_rwd->execute();

        /* Use the correlation input to mask the
         * wrong cross-correlation peaks.
         * This will overwrite the padding part of rwd_out.
         * Just to prevent having to allocate a new buffer */

        /* Create some aliases into the rwd_out buffer that
         * are used in the following sections.
         * We could allocate a buffer for each of theses sections
         * but allocating/deallocating them and handling alignments
         * looked like work to me */
        gr_complex *scratch_neg_time= &rwd_out[d_fft_len/2 - fft_payload_half_len];
        gr_complex *scratch_pos_time= &rwd_out[d_fft_len/2];
        float *scratch_mag_sq= (float*)&rwd_out[0];

        // Fill negative time
        volk_32fc_x2_multiply_conjugate_32fc(scratch_neg_time,
                                             &rwd_out[d_fft_len - fft_payload_half_len],
                                             &in_corr_history[tag_center - fft_payload_half_len],
                                             fft_payload_half_len);

        // Fill positive time
        volk_32fc_x2_multiply_conjugate_32fc(scratch_pos_time,
                                             &rwd_out[0],
                                             &in_corr_history[tag_center],
                                             fft_payload_half_len);

        /* Calculate the mag^2 of the correlation output
         * this will again reuse rwd_out as output */
        volk_32fc_magnitude_squared_32f(scratch_mag_sq,
                                        scratch_neg_time,
                                        fft_payload_len);

        // Locate the maximum
        int32_t peak_idx_rel= 0;
        volk_32f_index_max_32u((uint32_t *) &peak_idx_rel,
                               scratch_mag_sq,
                               fft_payload_len);

        // Determine the absolute energy
        float avg_fft_power= 0;
        volk_32f_accumulator_s32f(&avg_fft_power,
                                  scratch_mag_sq,
                                  fft_payload_len);
        avg_fft_power/= fft_payload_len;

        gr_complex peak= scratch_neg_time[peak_idx_rel];
        float power= scratch_mag_sq[peak_idx_rel];
        float rel_power= power/avg_fft_power;

        peak_idx_rel-= fft_payload_half_len;

        if(rel_power > d_threshold) {
          const int64_t peak_offset = (int64_t)(tag.offset + d_fft_len/4) + peak_idx_rel;
          if (peak_offset == d_last_xcorr_tag_offset){
            continue;
          }
          d_last_xcorr_tag_offset = peak_offset;
          const int64_t frame_buffer_start = peak_offset - nitems_written(0);

          // make sure sure we don't read from invalid memory!
          if(frame_buffer_start >= 0 and frame_buffer_start + d_sync_seq_len <= noutput_items){
            d_scale_factor = calculate_preamble_attenuation(out_pass + frame_buffer_start);
          }
          
          update_peak_tag(info, d_scale_factor, power, rel_power,
                          peak / std::abs(peak), d_peak_idx,
                          tag.offset);

          add_item_tag(0, peak_offset, d_tag_key, info);

          d_peak_idx++;
        }
      }

      get_tags_in_range(tags, 0, tag_reg_start, tag_reg_end);

      for(auto t : tags){
        if(t.key != d_tag_key){
          std::cout << "xcorr unknown key: ";
          pmt::print(t.value);
          add_item_tag(0, t);
        }
      }
      return noutput_items;
    }
  }
}
