from __future__ import annotations
import os
import argparse
import cv2
import numpy as np
import rospy
import signal
import yaml
from  sensor_msgs.msg import CameraInfo ,Image 
from cv_bridge import CvBridge, CvBridgeError
VENDOR_RESOLUTIONS : dict[tuple[int,int],float] = {}

def get_stereo_calibration_infos(left_config : str , right_config : str) -> tuple[bool,CameraInfo,CameraInfo] :
    #TODO implement how to get those infos from a well formatted ros like yaml file
    if os.path.isfile(left_config) & os.path.isfile(right_config) : 
        return  True , yaml_to_CameraInfo(left_config) ,yaml_to_CameraInfo(right_config) 
    return False , CameraInfo() , CameraInfo 

def yaml_to_CameraInfo(yaml_fname) -> CameraInfo:
    """
    Parse a yaml file containing camera calibration data (as produced by 
    rosrun camera_calibration cameracalibrator.py) into a 
    sensor_msgs/CameraInfo msg.
    
    Parameters
    ----------
    yaml_fname : str
        Path to yaml file containing camera calibration data
    Returns
    -------
    camera_info_msg : sensor_msgs.msg.CameraInfo
        A sensor_msgs.msg.CameraInfo message containing the camera calibration
        data
    """
    # Load data from file
    with open(yaml_fname, "r") as file_handle:
        calib_data = yaml.load(file_handle)
    # Parse
    camera_info_msg = CameraInfo()
    camera_info_msg.header.frame_id = calib_data["camera_name"]
    camera_info_msg.width = calib_data["image_width"]
    camera_info_msg.height = calib_data["image_height"]
    camera_info_msg.K = calib_data["camera_matrix"]["data"]
    camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
    camera_info_msg.R = calib_data["rectification_matrix"]["data"]
    camera_info_msg.P = calib_data["projection_matrix"]["data"]
    camera_info_msg.distortion_model = calib_data["distortion_model"]
    return camera_info_msg

class StereoRosUsbDriver(object) :
    __instance__ = None

    def __init__(self,channel : int , resolution: tuple[int,int],left_cfg : str = "",right_cfg : str = "",undistort : bool =False, verbose : bool = False , debug : bool = False) -> None:
        self._usb_channel_ = channel
        self.verbose = verbose 
        self.debug = debug 
        self._right_frame_ : np.array = np.zeros(np.shape(resolution[0],resolution[1]))
        self._left_frame_ : np.array = np.zeros(np.shape(resolution[0],resolution[1]))
        self.leftImage : Image = None
        self.rightImage : Image = None
        self._capture_object_  = cv2.VideoCapture(channel)
        self.__bridge__ = CvBridge()
        fps = 0 
        if verbose : 
            try :
                fps = VENDOR_RESOLUTIONS[resolution] 
            except KeyError as e : 
                print(" Could not find required resolution in the standard vendor resolution. Falling back to 60 fps")
                fps = 60
        else : 
            fps = VENDOR_RESOLUTIONS.get(resolution,60)
        self._capture_object_.set(cv2.CAP_PROP_FPS, fps)
        self._capture_object_.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
        self._capture_object_.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])
        self.left_cam_infos : CameraInfo  = None
        self.right_cam_infos : CameraInfo = None
        if undistort : 
            success,self.left_cam_infos,self.right_cam_infos = get_stereo_calibration_infos(left_config=left_cfg,right_config=right_cfg)
            if verbose and not success : 
                print("Stereo Camera Parameters loading failed during object construction.")
        __instance__ = self
        
    def acquire_frame(self) -> bool :
        success, original = self._capture_object_.read()
        if success : 
            height ,width, channel = original.shape()
            self._left_frame_ = original[0:height,0:int(width/2)]
            self._right_frame_ = original[0:height,int(width/2):(width)]
            if self.debug : 
                cv2.imshow("Left Image ",self._left_frame_)
                cv2.imshow("Right Frame",self._right_frame_)
            cv2.waitKey(1)
            return True
        else :
            if self.verbose :
                print("Frame Could not be retrieved from the camera.")
            return False

    def exportToRosMsgs(self):
        self.leftImage = self.__bridge__.cv2_to_imgmsg(self._left_frame_)
        self.rightImage = self.__bridge__.cv2_to_imgmsg(self._right_frame_)
        if (
            (self.left_cam_infos is None) or (self.right_cam_infos is None)
        ) and self.verbose:
            print("No Parameters have been given as camera parameters.")
    
    def run_acquisition_loop(self) -> bool : 
        while True : 
            self.acquire_frame()
            if cv2.waitKey(4) & 0xFF == ord('w'):
                break
        return True
    
    def __new__(cls,*args, **kwargs) -> StereoRosUsbDriver:
        if not isinstance(cls.__instance__, cls):
            cls.__instance__ = object.__new__(cls, *args, **kwargs)
        return cls.__instance__


class StereoRosWrapper(object) :
    
    def __init__(self,) -> None:
        pass    
        
