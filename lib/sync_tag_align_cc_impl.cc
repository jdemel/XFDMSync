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
      d_key(pmt::mp(tag_key)),
      d_port_tag_counters(num_inputs, 0),
      d_inport_tag_counters(num_inputs, 0),
      d_added_port_tag_counters(num_inputs, 0),
      d_last_offset_in_stream(num_inputs, 0)
{
    set_tag_propagation_policy(TPP_ONE_TO_ONE);
}

/*
 * Our virtual destructor.
 */
sync_tag_align_cc_impl::~sync_tag_align_cc_impl() {}

size_t sync_tag_align_cc_impl::get_tags_all_streams(
    std::vector<std::vector<gr::tag_t>>& stream_input_tags, const int noutput_items)
{
    size_t num_tags = 0;
    for (size_t i = 0; i < d_num_ports; ++i) {
        get_tags_in_range(stream_input_tags[i],
                          i,
                          nitems_read(i),
                          nitems_read(i) + noutput_items,
                          d_key);
        num_tags = std::max(stream_input_tags[i].size(), num_tags);
        if (stream_input_tags[i].size() > 0) {
            // std::string offset_str("");
            // for (const auto& t : stream_input_tags[i]) {
            //     offset_str += std::to_string(t.offset) + "\t";
            // }

            // GR_LOG_DEBUG(d_logger,
            //              "Found " + std::to_string(stream_input_tags[i].size()) +
            //                  " tags: @" + std::to_string(nitems_read(i)) +
            //                  " in stream: " + std::to_string(i) +
            //                  "\toffsets: " + offset_str);

            std::sort(stream_input_tags[i].begin(),
                      stream_input_tags[i].end(),
                      tag_t::offset_compare);
            d_inport_tag_counters[i] += stream_input_tags[i].size();
        }
    }
    return num_tags;
}

bool sync_tag_align_cc_impl::check_synchronicity(
    const std::vector<std::vector<gr::tag_t>>& stream_input_tags,
    const size_t max_stream_tags) const
{
    bool in_sync = true;
    for (const auto& tags : stream_input_tags) {
        in_sync = in_sync & (tags.size() == max_stream_tags);
    }
    if (not in_sync) {
        return false;
    }
    for (size_t i = 0; i < max_stream_tags; ++i) {
        const uint64_t offset = stream_input_tags[0][i].offset;
        for (const auto& tags : stream_input_tags) {
            const uint64_t smaller_offset = std::min(offset, tags[i].offset);
            if (smaller_offset + d_max_tag_offset_difference < offset) {
                std::string offset_str;
                size_t stream = 0;
                for (const auto& tt : stream_input_tags) {
                    offset_str += std::to_string(stream) + ": '";
                    for (const auto& t : tt) {
                        offset_str += std::to_string(t.offset) + ", ";
                    }
                    offset_str += "'";

                    stream++;
                }
                // GR_LOG_DEBUG(d_logger,
                //              "Async " + std::to_string(i) + "/" +
                //                  std::to_string(max_stream_tags) +
                //                  " offsets: " + offset_str);
                return false;
            }
        }
    }
    return true;
}

int sync_tag_align_cc_impl::work(int noutput_items,
                                 gr_vector_const_void_star& input_items,
                                 gr_vector_void_star& output_items)
{
    for (unsigned i = 0; i < d_num_ports; ++i) {
        const gr_complex* in = (const gr_complex*)input_items[i];
        gr_complex* out = (gr_complex*)output_items[i];
        std::memcpy(out, in, sizeof(gr_complex) * noutput_items);
    }

    std::vector<std::vector<gr::tag_t>> stream_input_tags(d_num_ports);
    size_t max_stream_tags = get_tags_all_streams(stream_input_tags, noutput_items);
    if (max_stream_tags == 0) {
        return noutput_items;
    }

    if (check_synchronicity(stream_input_tags, max_stream_tags)) {
        // std::string offset_str;
        // for (const auto& tags : stream_input_tags) {
        //     offset_str += std::to_string(tags[0].offset) + "\t";
        // }
        // GR_LOG_DEBUG(d_logger,
        //              "Synced! max_tags=" + std::to_string(max_stream_tags) +
        //                  "\toffsets: " + offset_str);
        return noutput_items;
    }

    std::string offset_str;
    size_t stream = 0;
    for (const auto& tt : stream_input_tags) {
        offset_str += std::to_string(stream) + ": '";
        for (const auto& t : tt) {
            offset_str += std::to_string(t.offset) + ", ";
        }
        offset_str += "'\t";

        stream++;
    }
    // GR_LOG_DEBUG(d_logger,
    //              std::to_string(nitems_read(0)) + " Async " +
    //                  std::to_string(max_stream_tags) + " offsets: " + offset_str);

    bool done = max_stream_tags == 0;

    std::vector<unsigned> positions(d_num_ports, 0);

    while (!done) {
        uint64_t offset = std::numeric_limits<uint64_t>::max();
        // gr::tag_t share_tag;
        unsigned share_tag_stream = 0;
        unsigned share_tag_pos = 0;

        for (unsigned i = 0; i < d_num_ports; ++i) {
            const uint64_t t_offset = stream_input_tags[i].size() > positions[i]
                                          ? stream_input_tags[i][positions[i]].offset
                                          : std::numeric_limits<uint64_t>::max();
            // GR_LOG_DEBUG(d_logger,
            //              "Search stream=" + std::to_string(i) +
            //                  "\t#tags=" + std::to_string(stream_input_tags[i].size()) +
            //                  "\twith offset=" + std::to_string(t_offset) +
            //                  "\tMINoffset=" + std::to_string(offset));
            if (offset > t_offset) {
                share_tag_stream = i;
                share_tag_pos = positions[i];
                // share_tag = stream_input_tags[i][positions[i]];
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
            if (t_offset > offset + d_max_tag_offset_difference) {
                auto share_tag = stream_input_tags[share_tag_stream][share_tag_pos];
                const uint64_t xcorr_idx = pmt::to_uint64(pmt::dict_ref(
                    share_tag.value, pmt::mp("xcorr_idx"), pmt::from_uint64(0)));
                std::string pst;
                for (size_t s = 0; s < d_num_ports; ++s) {
                    pst += std::to_string(stream_input_tags[s].size()) + ", ";
                }
                const uint64_t next_offset =
                    (d_last_offset_in_stream[i] < share_tag.offset)
                        ? share_tag.offset
                        : d_last_offset_in_stream[i] + 1;

                // GR_LOG_DEBUG(d_logger,
                //              std::to_string(nitems_read(0)) +
                //                  " Add tag with offset=" + std::to_string(next_offset)
                //                  + " in stream: " + std::to_string(i) + " share_stream:
                //                  " + std::to_string(share_tag_stream) + " share_pos: "
                //                  + std::to_string(share_tag_pos) + " xcorr_idx: " +
                //                  std::to_string(xcorr_idx) +
                //                  "\t#tags=" + pst);

                add_item_tag(i,
                             next_offset,
                             share_tag.key,
                             share_tag.value,
                             pmt::mp(name() + "_S" + std::to_string(i)));
                d_last_offset_in_stream[i] = next_offset;
                // add_item_tag(i, share_tag);
                d_added_port_tag_counters[i] += 1;
            } else {
                d_last_offset_in_stream[i] = stream_input_tags[i][positions[i]].offset;
                // auto& tag = stream_input_tags[i][positions[i]];
                // const uint64_t xcorr_idx = pmt::to_uint64(
                //     pmt::dict_ref(tag.value, pmt::mp("xcorr_idx"),
                //     pmt::from_uint64(0)));
                // GR_LOG_DEBUG(d_logger,
                //              "Haz tag with offset=" + std::to_string(offset) +
                //                  " co=" + std::to_string(t_offset) +
                //                  " in stream: " + std::to_string(i) +
                //                  " xcorr_idx: " + std::to_string(xcorr_idx));
                positions[i] += 1;
            }
        }

        done = true;
        for (unsigned i = 0; i < d_num_ports; ++i) {
            if (positions[i] < stream_input_tags[i].size()) {
                done = false;
            }
        }
    }

    // Tell runtime system how many output items we produced.
    return noutput_items;
}

} /* namespace xfdm_sync */
} /* namespace gr */
