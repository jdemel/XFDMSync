/* -*- c++ -*- */
/*
 * Copyright 2020 Johannes Demel.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "sync_tag_align_cc_impl.h"
#include <gnuradio/io_signature.h>
#include <algorithm>
#include <limits>

namespace gr {
namespace xfdm_sync {

sync_tag_align_cc::sptr sync_tag_align_cc::make(const unsigned num_inputs,
                                                const std::string& tag_key)
{
    return gnuradio::get_initial_sptr(new sync_tag_align_cc_impl(num_inputs, tag_key));
}


/*
 * The private constructor
 */
sync_tag_align_cc_impl::sync_tag_align_cc_impl(const unsigned num_inputs,
                                               const std::string& tag_key)
    : gr::sync_block("sync_tag_align_cc",
                     gr::io_signature::make(num_inputs, num_inputs, sizeof(gr_complex)),
                     gr::io_signature::make(num_inputs, num_inputs, sizeof(gr_complex))),
      d_num_ports(num_inputs),
      d_tag_key(tag_key),
      d_key(pmt::mp(tag_key))
{
    set_tag_propagation_policy(TPP_ONE_TO_ONE);
}

/*
 * Our virtual destructor.
 */
sync_tag_align_cc_impl::~sync_tag_align_cc_impl() {}

int sync_tag_align_cc_impl::work(int noutput_items,
                                 gr_vector_const_void_star& input_items,
                                 gr_vector_void_star& output_items)
{
    // GR_LOG_DEBUG(this->d_logger, "ENTER " + std::to_string(noutput_items));
    for (unsigned i = 0; i < d_num_ports; ++i) {
        const gr_complex* in = (const gr_complex*)input_items[i];
        gr_complex* out = (gr_complex*)output_items[i];
        std::memcpy(out, in, sizeof(gr_complex) * noutput_items);
    }

    std::vector<std::vector<gr::tag_t>> stream_input_tags;
    bool done = true;
    for (unsigned i = 0; i < d_num_ports; ++i) {
        std::vector<gr::tag_t> tags;
        get_tags_in_range(tags, i, nitems_read(i), nitems_read(i) + noutput_items, d_key);
        if (tags.size() > 0) {
            // GR_LOG_DEBUG(d_logger,
            //              "Found tags: @" + std::to_string(nitems_read(i)) +
            //                  " in stream: " + std::to_string(i));
            done = false;
            std::sort(tags.begin(), tags.begin(), tag_t::offset_compare);
        }
        stream_input_tags.push_back(tags);
    }

    std::vector<unsigned> positions(d_num_ports, 0);

    while (!done) {
        uint64_t offset = std::numeric_limits<uint64_t>::max();
        gr::tag_t share_tag;

        for (unsigned i = 0; i < d_num_ports; ++i) {
            const uint64_t t_offset = stream_input_tags[i].size() > positions[i]
                                          ? stream_input_tags[i][positions[i]].offset
                                          : std::numeric_limits<uint64_t>::max();
            if (offset > t_offset) {
                share_tag = stream_input_tags[i][positions[i]];
                offset = t_offset;
            }
        }
        if (offset == std::numeric_limits<uint64_t>::max()) {
            break;
        }

        for (unsigned i = 0; i < d_num_ports; ++i) {
            const uint64_t t_offset = stream_input_tags[i].size() > positions[i]
                                          ? stream_input_tags[i][positions[i]].offset
                                          : std::numeric_limits<uint64_t>::max();
            if (t_offset > offset + 1000) {
                GR_LOG_DEBUG(d_logger,
                             "Add tag with offset=" + std::to_string(share_tag.offset) +
                                 " co=" + std::to_string(offset) +
                                 " in stream: " + std::to_string(i));
                add_item_tag(i, share_tag);
            } else {
                positions.at(i) += 1;
            }
        }

        done = true;
        for (unsigned i = 0; i < d_num_ports; ++i) {
            if (positions.at(i) < stream_input_tags.at(i).size()) {
                done = false;
            }
        }
    }

    // Tell runtime system how many output items we produced.
    return noutput_items;
}

} /* namespace xfdm_sync */
} /* namespace gr */
