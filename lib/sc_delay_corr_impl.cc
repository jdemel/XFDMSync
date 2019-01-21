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
#include "sc_delay_corr_impl.h"

/* volk_get_alignment can not be evaluated at compile-time.
 * But __builtin_assume_aligned needs an alignment at compile-time.
 * 32 bytes alignment should be correct for SSE and AVX */
#define MEM_ALIGNMENT (32)

namespace gr {
  namespace xfdm_sync {

    sc_delay_corr::sptr
    sc_delay_corr::make(int seq_len, bool normalize)
    {
      return gnuradio::get_initial_sptr(new sc_delay_corr_impl(seq_len, normalize));
    }

    sc_delay_corr_impl::sc_delay_corr_impl(int seq_len, bool normalize)
      : gr::sync_block("sc_delay_corr",
                       gr::io_signature::make(1, 1, sizeof(gr_complex)),
                       gr::io_signature::make(2, 2, sizeof(gr_complex))),
      d_seq_len(seq_len),
      d_normalize(normalize)
    {
      set_history(2 * seq_len + 1);

      for(int i=0; i<3; i++) {
        d_corr_windows[i]= (gr_complex *)volk_malloc(sizeof(gr_complex) * d_seq_len,
                                                     MEM_ALIGNMENT);
      }

      for(int i=0; i<4; i++) {
        d_norm_windows[i]= (float *)volk_malloc(sizeof(float) * d_seq_len,
                                                MEM_ALIGNMENT);
      }

    }

    sc_delay_corr_impl::~sc_delay_corr_impl()
    {
      for(int i=0; i<3; i++) volk_free(d_corr_windows[i]);
      for(int i=0; i<4; i++) volk_free(d_norm_windows[i]);
    }

    int
    sc_delay_corr_impl::work(int noutput_items,
                             gr_vector_const_void_star &input_items,
                             gr_vector_void_star &output_items)
    {
      const gr_complex *in_history = (const gr_complex *) input_items[0];
      gr_complex *out_pass = (gr_complex *) output_items[0];
      gr_complex *out_corr = (gr_complex *) output_items[1];

      /* Elements in the history will be referred to
       * using negative indices */
      const gr_complex *in = &in_history[history() - 1];

      /* Output the delayed input stream */
      memcpy(out_pass, &in[-d_seq_len * 2], sizeof(gr_complex) * noutput_items);

      gr_complex *wc_old=     (gr_complex *)__builtin_assume_aligned(d_corr_windows[0], MEM_ALIGNMENT);
      gr_complex *wc_cur=     (gr_complex *)__builtin_assume_aligned(d_corr_windows[1], MEM_ALIGNMENT);
      gr_complex *wc_scratch= (gr_complex *)__builtin_assume_aligned(d_corr_windows[2], MEM_ALIGNMENT);

      float *wn_older=   (float *)__builtin_assume_aligned(d_norm_windows[0], MEM_ALIGNMENT);
      float *wn_old=     (float *)__builtin_assume_aligned(d_norm_windows[1], MEM_ALIGNMENT);
      float *wn_cur=     (float *)__builtin_assume_aligned(d_norm_windows[2], MEM_ALIGNMENT);
      float *wn_scratch= (float *)__builtin_assume_aligned(d_norm_windows[3], MEM_ALIGNMENT);

      gr_complex corr_acc= 0;
      float norm_acc= 0;

      /* Pre-heat correlation window and corr_acc with history-values */
      volk_32fc_x2_multiply_conjugate_32fc(wc_cur,
                                           &in[-d_seq_len],
                                           &in[-d_seq_len * 2],
                                           d_seq_len);

      for(int i=0; i<d_seq_len; i++) corr_acc+= wc_cur[i];

      /* Pre-heat norm window and norm_acc with history-values */
      volk_32fc_magnitude_squared_32f(wn_old,
                                      &in[-2*d_seq_len],
                                      d_seq_len);

      for(int i=0; i<d_seq_len; i++) norm_acc+= wn_old[i];

      volk_32fc_magnitude_squared_32f(wn_cur,
                                      &in[-d_seq_len],
                                      d_seq_len);

      for(int i=0; i<d_seq_len; i++) norm_acc+= wn_cur[i];

      int io_idx= 0;

      /* Processing whole windows at once allows us to vectorize most
       * of the processing.
       * I hope it does not only make the code unreadable but will also
       * help with performance. */
      for(io_idx= 0;
          (io_idx + d_seq_len) <= noutput_items;
          io_idx+= d_seq_len) {

        /* Cycle the buffers */
        gr_complex *wc_tmp= wc_old;
        wc_old= (gr_complex *)__builtin_assume_aligned(wc_cur, MEM_ALIGNMENT);
        wc_cur= (gr_complex *)__builtin_assume_aligned(wc_tmp, MEM_ALIGNMENT);

        float *wn_tmp= wn_older;
        wn_older= (float *)__builtin_assume_aligned(wn_old, MEM_ALIGNMENT);
        wn_old=   (float *)__builtin_assume_aligned(wn_cur, MEM_ALIGNMENT);;
        wn_cur=   (float *)__builtin_assume_aligned(wn_tmp, MEM_ALIGNMENT);;


        // Calculate next correlation window
        volk_32fc_x2_multiply_conjugate_32fc(wc_cur,
                                             &in[io_idx],
                                             &in[io_idx - d_seq_len],
                                             d_seq_len);

        // Subtract samples leaving the window
        volk_32f_x2_subtract_32f((float *)wc_scratch,
                                 (float *)wc_cur,
                                 (float *)wc_old,
                                 2*d_seq_len);

        // Calculate next normalization window
        volk_32fc_magnitude_squared_32f(wn_cur,
                                        &in[io_idx],
                                        d_seq_len);

        // Subtract samples leaving the window
        volk_32f_x2_subtract_32f(wn_scratch,
                                 wn_cur,
                                 wn_older,
                                 d_seq_len);

        // Slide through the window and produce output
        for(int i=0; i<d_seq_len; i++) {
          corr_acc+= wc_scratch[i];
          norm_acc+= wn_scratch[i];

          out_corr[io_idx + i]= (norm_acc > 0) ? (2.0f * corr_acc / norm_acc) : 0;
        }
      }

      /* Process the leftover samples */
      for(int i=0; io_idx < noutput_items; io_idx++, i++) {
        gr_complex a= in[io_idx];
        gr_complex b= in[io_idx - d_seq_len];

        corr_acc-= wc_cur[i];
        corr_acc+= a * std::conj(b);

        norm_acc-= wn_cur[i];
        norm_acc+= std::norm(a);

        out_corr[io_idx]= 2.0f * corr_acc / norm_acc;
      }

      return noutput_items;
    }
  }
}
