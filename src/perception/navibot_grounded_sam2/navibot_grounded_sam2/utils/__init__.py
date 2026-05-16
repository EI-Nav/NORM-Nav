#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Utility modules for GroundedSAM2 tracker.

This package provides helper utilities for:
- Mask dictionary management and tracking
- Video frame processing
- Supervision integration
- Common utilities

Author: Wang Junhui <wjh_9696@163.com>
License: MIT
"""

from navibot_grounded_sam2.utils.mask_dictionary_model import MaskDictionaryModel, ObjectInfo

__all__ = [
    'MaskDictionaryModel',
    'ObjectInfo',
]

