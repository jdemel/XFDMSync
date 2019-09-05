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


#ifndef INCLUDED_XFDM_SYNC_FRAME_GATE_H
#define INCLUDED_XFDM_SYNC_FRAME_GATE_H

#include <xfdm_sync/api.h>
#include <gnuradio/block.h>

namespace gr {
  namespace xfdm_sync {

    /*!
     * \brief <+description of block+>
     * \ingroup xfdm_sync
     *
     */
    class XFDM_SYNC_API frame_gate : virtual public gr::block
    {
     public:
      typedef boost::shared_ptr<frame_gate> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of xfdm_sync::frame_gate.
       *
       * To avoid accidental use of raw pointers, xfdm_sync::frame_gate's
       * constructor is in a private implementation
       * class. xfdm_sync::frame_gate::make is the public interface for
       * creating new instances.
       */
      static sptr make(int len_prologue, int len_epilogue, int len_symbol, int symbols_per_frame_min, int symbols_per_frame_max, bool do_compensate);
    };

  } // namespace xfdm_sync
} // namespace gr

#endif /* INCLUDED_XFDM_SYNC_FRAME_GATE_H */

