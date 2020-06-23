#
# Copyright 2008,2009 Free Software Foundation, Inc.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

# The presence of this file turns this directory into a Python package

'''
This is the GNU Radio XFDM_SYNC module. Place your Python package
description here (python/__init__.py).
'''
import os

# import pybind11 generated symbols into the xfdm_sync namespace
try:
    from .xfdm_sync_python import *
except ImportError:
    print('find local')
    dirname, filename = os.path.split(os.path.abspath(__file__))
    __path__.append(os.path.join(dirname, "bindings"))
    from .xfdm_sync_python import *

# import any pure python here
from .fqsweep_corr import fqsweep_corr
from .burstsilence_corr import burstsilence_corr
#
