/* -*- c++ -*- */

#define XFDM_SYNC_API

%include "gnuradio.i"			// the common stuff

//load generated python docstrings
%include "xfdm_sync_swig_doc.i"

%{
#include "xfdm_sync/sc_delay_corr.h"
#include "xfdm_sync/sc_tagger.h"
#include "xfdm_sync/xcorr_tagger.h"
#include "xfdm_sync/frame_gate.h"
%}


%include "xfdm_sync/sc_delay_corr.h"
GR_SWIG_BLOCK_MAGIC2(xfdm_sync, sc_delay_corr);

%include "xfdm_sync/sc_tagger.h"
GR_SWIG_BLOCK_MAGIC2(xfdm_sync, sc_tagger);

%include "xfdm_sync/xcorr_tagger.h"
GR_SWIG_BLOCK_MAGIC2(xfdm_sync, xcorr_tagger);

%include "xfdm_sync/frame_gate.h"
GR_SWIG_BLOCK_MAGIC2(xfdm_sync, frame_gate);
