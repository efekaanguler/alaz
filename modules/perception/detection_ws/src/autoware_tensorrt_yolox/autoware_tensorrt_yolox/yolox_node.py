#!/usr/bin/env python3

"""Backward-compatible entrypoint.

This module keeps the historical yolox executable name working while the
actual implementation is in yolov8_node.py.
"""

from autoware_tensorrt_yolox.yolov8_node import main


if __name__ == '__main__':
    main()
