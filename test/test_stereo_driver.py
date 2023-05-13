#!/usr/bin/python3
from stereo_usb_driver import StereoRosWrapper ,StereoUsbDriver

driver = StereoUsbDriver(channel=2,
                        resolution=(1280,480),
                        left_cfg="/home/aubin/viso_imu/config/intrinsics.yaml",
                        right_cfg="/home/aubin/viso_imu/config/intrinsics.yaml",
                        undistort=True,
                        verbose=True,
                        debug=True)
if driver.run_acquisition_loop() : 
    print("Acquisition loop exited. Returning ...")