/* -*- c++ -*- */
/*
 * Copyright 2020 Johannes Demel.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_XFDM_SYNC_SYNC_TAG_ALIGN_CC_H
#define INCLUDED_XFDM_SYNC_SYNC_TAG_ALIGN_CC_H

#include <gnuradio/sync_block.h>
#include <xfdm_sync/api.h>

namespace gr {
namespace xfdm_sync {

/*!
 * \brief <+description of block+>
 * \ingroup xfdm_sync
 *
 */
class XFDM_SYNC_API sync_tag_align_cc : virtual public gr::sync_block
{
public:
    typedef std::shared_ptr<sync_tag_align_cc> sptr;

    /*!
     * \brief Return a shared_ptr to a new instance of xfdm_sync::sync_tag_align_cc.
     *
     * To avoid accidental use of raw pointers, xfdm_sync::sync_tag_align_cc's
     * constructor is in a private implementation
     * class. xfdm_sync::sync_tag_align_cc::make is the public interface for
     * creating new instances.
     */
    static sptr make(const unsigned num_inputs, const std::string& tag_key);
};

} // namespace xfdm_sync
} // namespace gr

#endif /* INCLUDED_XFDM_SYNC_SYNC_TAG_ALIGN_CC_H */
