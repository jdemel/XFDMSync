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

#include "frame_gate_impl.h"
#include <gnuradio/io_signature.h>
#include <volk/volk.h>

namespace gr {
namespace xfdm_sync {

frame_gate::sptr frame_gate::make(int len_prologue,
                                  int len_epilogue,
                                  int len_symbol,
                                  int symbols_per_frame_min,
                                  int symbols_per_frame_max,
                                  bool do_compensate)
{
    return gnuradio::get_initial_sptr(new frame_gate_impl(len_prologue,
                                                          len_epilogue,
                                                          len_symbol,
                                                          symbols_per_frame_min,
                                                          symbols_per_frame_max,
                                                          do_compensate));
}

frame_gate_impl::frame_gate_impl(int len_prologue,
                                 int len_epilogue,
                                 int len_symbol,
                                 int symbols_per_frame_min,
                                 int symbols_per_frame_max,
                                 bool do_compensate)
    : gr::block("frame_gate",
                gr::io_signature::make(1, 1, sizeof(gr_complex)),
                gr::io_signature::make(1, 1, sizeof(gr_complex)))
{
    /* We actually only want to block the
     * preamble_start tag */
    set_tag_propagation_policy(TPP_DONT);

    /* Register feedback port */
    pmt::pmt_t mpid = pmt::mp("frame_hint");
    message_port_register_in(mpid);
    set_msg_handler(mpid, [this](pmt::pmt_t msg) { this->on_frame_hint(msg); });

    /* initialize attributes */
    d_frame.id = 0;
    d_frame.active = false;

    d_parameters.len_prologue = len_prologue;
    d_parameters.len_epilogue = len_epilogue;
    d_parameters.len_symbol = len_symbol;

    d_parameters.symbols_per_frame_max = symbols_per_frame_max;
    d_parameters.symbols_per_frame_min = symbols_per_frame_min;

    d_parameters.do_compensate = do_compensate;
}

frame_gate_impl::~frame_gate_impl() {}

void frame_gate_impl::on_frame_hint(pmt::pmt_t msg)
{
    if (!pmt::is_tuple(msg) || (pmt::length(msg) != 3)) {
        GR_LOG_WARN(d_debug_logger,
                    "frame_gate: frame_hint was not formatted correctly\n");
        return;
    }

    pmt::pmt_t operation = pmt::tuple_ref(msg, 0);
    pmt::pmt_t frame_id = pmt::tuple_ref(msg, 1);
    pmt::pmt_t data = pmt::tuple_ref(msg, 2);

    /* Early out if the frame is already gone */
    if (!pmt::equal(frame_id, pmt::from_uint64(d_frame.id))) {
        return;
    }

    int len_pe = d_parameters.len_prologue + d_parameters.len_epilogue;

    bool set_max = false;
    bool set_min = false;

    /* Tell how many symbols it should output at most.
     * Keep in mind that when the message is received
     * the block might already have produced more output symbols.
     * The block may still produce fewer symbols when a new
     * start of frame tag is received. */
    if (pmt::equal(operation, pmt::mp("max_symbols")))
        set_max = true;

    /* Tell the block how many symbols to output at least.
     * This will prevent the detection of new frames until
     * this many symbols were read. */
    if (pmt::equal(operation, pmt::mp("min_symbols")))
        set_min = true;

    /* Tell the block to ready exactly this number of symbols.
     * Use this operation whenever possible. */
    if (pmt::equal(operation, pmt::mp("num_symbols"))) {
        set_max = true;
        set_min = true;
    }

    if (set_max || set_min) {
        int symbols = pmt::to_long(data);
        int samples = len_pe + d_parameters.len_symbol * symbols;

        if (set_max)
            d_frame.samples_max = samples;
        if (set_min)
            d_frame.samples_max = samples;
    }
}

void frame_gate_impl::forecast(int noutput_items, gr_vector_int& ninput_items_required)
{
    ninput_items_required[0] = noutput_items;
}

int frame_gate_impl::general_work(int noutput_items,
                                  gr_vector_int& ninput_items,
                                  gr_vector_const_void_star& input_items,
                                  gr_vector_void_star& output_items)
{
    const gr_complex* in = (const gr_complex*)input_items[0];
    gr_complex* out = (gr_complex*)output_items[0];
    const int in_len = ninput_items[0];
    const int out_len = noutput_items;

    std::vector<tag_t> tags;
    get_tags_in_window(tags, 0, 0, in_len, pmt::mp("frame_start"));

    int in_idx = 0;
    int out_idx = 0;

    while ((in_idx < in_len) && (out_idx < out_len)) {
        const int len_pe = d_parameters.len_prologue + d_parameters.len_epilogue;

        /* This stores the current estimate of the remaining
         * frame length in samples relative to in_idx.
         * It will be updated based on some rules.
         * The initial value is just the largest number that will
         * not cause overflows later on. This is a dirty hack. */
        int frame_end = INT_MAX - d_frame.samples_consumed;

        /* Enforce the maximum frame length */
        if (d_frame.samples_max >= 0) {
            frame_end =
                std::min(frame_end, d_frame.samples_max - d_frame.samples_consumed);
        }

        /* Go through all the start of frame tags after in_idx
         * and make sure the frame ends before any one of them */
        for (tag_t tag : tags) {
            if (tag.offset >= nitems_read(0) + in_idx) {
                frame_end =
                    std::min(frame_end, (int)(tag.offset - nitems_read(0)) - in_idx);
            }
        }

        /* If a minimal frame length is set the previous rules
         * may be overruled by it.
         * This may lead to received frames being skipped */
        if (d_frame.samples_min >= 0) {
            frame_end =
                std::max(frame_end, d_frame.samples_min - d_frame.samples_consumed);
        }

        /* If the block is currently outputting samples it has to enforce
         * a whole number of symbols per frame.
         * This conditional will always round down to the next integer
         * number of symbols. */
        if (d_frame.active) {
            int frame_len = d_frame.samples_consumed + frame_end;
            int num_symbols = (frame_len - len_pe) / d_parameters.len_symbol;
            frame_end = (len_pe + num_symbols * d_parameters.len_symbol) -
                        d_frame.samples_consumed;
        }

        /* The output can either be limited by the input buffer or
         * by the output buffer */
        int buffer_end = std::min(out_len - out_idx, in_len - in_idx);

        /* Copying has to end either at the end of the frame
         * or at the end of the buffeer */
        int copy_end = std::min(buffer_end, frame_end);

        if (copy_end < 0) {
            throw std::runtime_error("frame_gate: copy_end < 0. This is bad.");
        }

        if (d_frame.active) {
            /* This volk kernel will perform frequency compensation
             * and phase correction while copying from the input buffer
             * to the output buffer. */
            volk_32fc_s32fc_x2_rotator_32fc(&out[out_idx],
                                            &in[in_idx],
                                            d_frame.fq_comp_rot,
                                            &d_frame.fq_comp_acc,
                                            copy_end);

            /* While data is copied the input and output buffers advance
             * at the same pace */
            out_idx += copy_end;
            in_idx += copy_end;
            d_frame.samples_consumed += copy_end;

            /* Stop outputting data if an end of frame condition
             * was detected */
            if (frame_end < buffer_end) {
                d_frame.samples_max = -1;
                d_frame.samples_min = -1;
                d_frame.active = false;

                d_frame.id++;
            }
        } else {
            /* When no data is copied the output index and number of
             * consumed samples are not incremented */
            in_idx += copy_end;

            /* Find out if there is a start of frame tag at the current
             * input buffer position.
             * This should be the case if frame_end was set based on a tag */
            for (tag_t tag : tags) {
                if (tag.offset == nitems_read(0) + in_idx) {
                    pmt::pmt_t info = tag.value;

                    d_frame.active = true;
                    d_frame.samples_consumed = 0;

                    d_frame.samples_max = -1;
                    if (d_parameters.symbols_per_frame_max >= 0) {
                        d_frame.samples_max =
                            len_pe +
                            d_parameters.symbols_per_frame_max * d_parameters.len_symbol;
                    }

                    d_frame.samples_min = len_pe;
                    if (d_parameters.symbols_per_frame_min >= 0) {
                        d_frame.samples_min =
                            len_pe +
                            d_parameters.symbols_per_frame_min * d_parameters.len_symbol;
                    }

                    d_frame.fq_comp_acc = 1;
                    d_frame.fq_comp_rot = 1;

                    if (d_parameters.do_compensate) {
                        /* If the sc_tagger set the sc_rot field
                         * use it to compensate frequency offsets */
                        pmt::pmt_t rot =
                            pmt::dict_ref(info, pmt::mp("sc_rot"), pmt::PMT_NIL);

                        if (pmt::is_complex(rot)) {
                            d_frame.fq_comp_rot = std::conj(pmt::to_complex(rot));
                            d_frame.fq_comp_rot /= std::abs(d_frame.fq_comp_rot);
                        }

                        /* If the xcorr_tagger set the xcorr_rot field
                         * use it to set the start phase based on the preamble phase
                         * difference. This is a kind of poor-man's channel estimation */
                        pmt::pmt_t phase =
                            pmt::dict_ref(info, pmt::mp("xcorr_rot"), pmt::PMT_NIL);

                        if (pmt::is_complex(phase)) {
                            d_frame.fq_comp_acc = std::conj(pmt::to_complex(phase));
                            d_frame.fq_comp_acc /= std::abs(d_frame.fq_comp_acc);
                        }
                    }

                    /* Add own info to the tag metadata and forward
                     * it as a start of frame tag */
                    pmt::dict_add(
                        info, pmt::mp("frame_id"), pmt::from_uint64(d_frame.id));

                    add_item_tag(
                        0, nitems_written(0) + out_idx, pmt::mp("frame_start"), info);
                }
            }
        }
    }

    consume_each(in_idx);

    return out_idx;
}
} // namespace xfdm_sync
} // namespace gr
