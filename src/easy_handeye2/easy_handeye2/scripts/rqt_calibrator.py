#!/usr/bin/env python

import sys

import rclpy
from rclpy.signals import SignalHandlerOptions
from rqt_gui.main import Main

# Let Qt close its plugins/spinner before destroying the ROS context. Otherwise
# Humble's rclpy signal handler can invalidate the context inside the rqt thread.
rclpy.init(args=sys.argv, signal_handler_options=SignalHandlerOptions.NO)
try:
    main = Main()
    result = main.main(sys.argv, standalone='easy_handeye2.handeye_rqt_calibrator.RqtHandeyeCalibrator')
finally:
    rclpy.try_shutdown()
sys.exit(result)
