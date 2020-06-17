/* -*- c++ -*- */
/*
 * Copyright 2020 Johannes Demel.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_XFDM_SYNC_SYNC_TAG_ALIGN_CC_IMPL_H
#define INCLUDED_XFDM_SYNC_SYNC_TAG_ALIGN_CC_IMPL_H

#include <xfdm_sync/sync_tag_align_cc.h>

namespace gr {
namespace xfdm_sync {

class sync_tag_align_cc_impl : public sync_tag_align_cc
{
private:
    const unsigned d_num_ports;
    const std::string d_tag_key;
    const pmt::pmt_t d_key;

public:
    sync_tag_align_cc_impl(const unsigned num_inputs, const std::string& tag_key);
    ~sync_tag_align_cc_impl();

    // Where all the action really happens
    int work(int noutput_items,
             gr_vector_const_void_star& input_items,
             gr_vector_void_star& output_items);
};

} // namespace xfdm_sync
} // namespace gr

#endif /* INCLUDED_XFDM_SYNC_SYNC_TAG_ALIGN_CC_IMPL_H */
