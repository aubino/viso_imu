from __future__ import annotations
import os
import argparse
import cv2
import numpy as np
import rospy
import signal
import yaml
from std_msgs.msg import Header
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

class StereoUsbDriver(object) :
    __instances__ : list[StereoUsbDriver] = []

    def __init__(self,channel : int , resolution: tuple[int,int],left_cfg : str = "",right_cfg : str = "",undistort : bool =False, verbose : bool = False , debug : bool = False) -> None:
        if len(StereoUsbDriver.__instances__) != 0:
            for instance in StereoUsbDriver.__instances__ : 
                if instance._usb_channel_ == channel :
                    self = instance
            return
        else: 
            self._usb_channel_ = channel
            self.verbose = verbose
            self.debug = debug
            self._right_frame_ : np.array = np.zeros(np.shape((resolution[0],resolution[1])))
            self._left_frame_ : np.array = np.zeros(np.shape((resolution[0],resolution[1])))
            self._left_frame_rec : np.array
            self._right_frame_rec : np.array
            self.leftImage : Image = None
            self.rightImage : Image = None
            self.leftImageRec : Image = None
            self.rightImageRec : Image = None
            self._capture_object_  = cv2.VideoCapture(channel)
            self.__bridge__ = CvBridge()
            fps = 0
            if verbose : 
                try :
                    fps = VENDOR_RESOLUTIONS[resolution] 
                except KeyError as e : 
                    print(" Could not find required resolution in the standard vendor resolutions. Falling back to 60 fps")
                    fps = 60
            else : 
                fps = VENDOR_RESOLUTIONS.get(resolution,60)
            self._capture_object_.set(cv2.CAP_PROP_FPS, fps)
            self._capture_object_.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
            self._capture_object_.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])
            self.left_cam_infos : CameraInfo  = None
            self.right_cam_infos : CameraInfo = None
            self.left_cam_infos_rec : CameraInfo = None
            self.right_cam_infos_rec : CameraInfo = None
            if undistort : 
                success,self.left_cam_infos,self.right_cam_infos = get_stereo_calibration_infos(left_config=left_cfg,right_config=right_cfg)
                if verbose and not success : 
                    print("Stereo Camera Parameters loading failed during object construction.")
                if verbose and success : 
                    print("Using the following intrinsic parameters : ")
                    print("Left Intrinsics : ",self.left_cam_infos)
                    print("")
                    print("Right intrinsics : ",self.right_cam_infos)
            StereoUsbDriver.__instances__.append(self)
        return
        
    def acquire_frame(self) -> bool :
        success, original = self._capture_object_.read()
        if success : 
            height ,width, channels = original.shape
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
        self.leftImage = self.__bridge__.cv2_to_imgmsg(self._left_frame_,encoding='bgr8' ,header=Header(frame_id= "stereo/left_frame"))
        self.rightImage = self.__bridge__.cv2_to_imgmsg(self._right_frame_,encoding='bgr8',header=Header(frame_id= "stereo/right_frame"))
        if (
            (self.left_cam_infos is None) or (self.right_cam_infos is None)
        ) and self.verbose:
            print("No Parameters have been given as camera parameters.")
    
    def undistort(self):
        left_camera_matrix = np.array([[self.left_cam_infos.K[0],self.left_cam_infos.K[1],self.left_cam_infos.K[2]],
                                            [self.left_cam_infos.K[3],self.left_cam_infos.K[4],self.left_cam_infos.K[5]],
                                            [self.left_cam_infos.K[6],self.left_cam_infos.K[7],self.left_cam_infos.K[8]]])
        left_distortion_matrix = np.array([list(self.left_cam_infos.D)])
        right_camera_matrix = np.array([[self.right_cam_infos.K[0],self.right_cam_infos.K[1],self.right_cam_infos.K[2]],
                                            [self.right_cam_infos.K[3],self.right_cam_infos.K[4],self.right_cam_infos.K[5]],
                                            [self.right_cam_infos.K[6],self.right_cam_infos.K[7],self.right_cam_infos.K[8]]])
        right_distortion_matrix = np.array([list(self.right_cam_infos.D)])
        # R1, R2, P1, P2, Q, validPixROI1, validPixROI2 = cv2.stereoRectify(left_camera_matrix,
                                                                        #   left_distortion_matrix,
                                                                        #   right_camera_matrix,
                                                                        #   right_distortion_matrix,
                                                                        #   (left_camera_matrix.Shape[0],left_camera_matrix.Shape[1]),
                                                                        #   )
        # left_opt_camera_matrix, validLeftPixROI = cv2.getOptimalNewCameraMatrix(left_camera_matrix, 
                                                                                # left_distortion_matrix, 
                                                                                # (self._left_frame_.Shape[0],self._left_frame_.Shape[1]),
                                                                                # 1,
                                                                                # (self._left_frame_.Shape[0],self._left_frame_.Shape[1]))
        # right_opt_camera_matrix, validRightPixROI = cv2.getOptimalNewCameraMatrix(right_camera_matrix, right_distortion_matrix, self.,
        # 1)
        # self._left_frame_rec =  cv2.undistort(self._left_frame_, left_camera_matrix, left_distortion_matrix, None, left_camera_matrix)
        # self._right_frame_rec = cv2.undistort(self._right_frame_, right_camera_matrix, right_distortion_matrix, None, left_camera_matrix)
        
        
    
    def run_acquisition_loop(self) -> bool : 
        while True : 
            self.acquire_frame()
            if cv2.waitKey(4) & 0xFF == ord('w'):
                break
        return True

class StereoRosWrapper(StereoUsbDriver) :
    
    def __init__(self, channel: int, 
                resolution: tuple[int, int],
                left_cfg: str = "", 
                right_cfg: str = "", 
                undistort: bool = False, 
                verbose: bool = False, 
                debug: bool = False) -> None:
        """_summary_

        Args:
            channel (int): _description_
            resolution (tuple[int, int]): _description_
            ros_config (str): _description_
            left_cfg (str, optional): _description_. Defaults to "".
            right_cfg (str, optional): _description_. Defaults to "".
            undistort (bool, optional): _description_. Defaults to False.
            verbose (bool, optional): _description_. Defaults to False.
            debug (bool, optional): _description_. Defaults to False.
        """
        _debug = debug
        _verbose = verbose
        _verbose = rospy.get_param('~/verbose',_verbose)
        _debug   = rospy.get_param('~/verbose',_debug)
        super().__init__(
            channel, resolution, left_cfg, right_cfg, undistort, _verbose, _debug
        )
        left_topic_name = rospy.get_param('~/left_topic_name',"left_camera")
        right_topic_name = rospy.get_param('~/right_topic_name',"right_camera")
        left_cam_info_topic_name = rospy.get_param('~/left_cam_info_topic_name',"left_info")
        right_cam_info_topic_name = rospy.get_param('~/right_cam_info_topic_name',"right_info")
        self.left_image_topic = rospy.Publisher(left_topic_name,Image,queue_size=1)
        self.right_image_topic = rospy.Publisher(right_topic_name,Image,queue_size=1)
        if self.right_cam_infos is not None : 
            self.left_info_topic = rospy.Publisher(left_cam_info_topic_name,CameraInfo,queue_size=1)
        if self.left_cam_infos is not None :
            self.right_info_topic = rospy.Publisher(right_cam_info_topic_name,CameraInfo,queue_size=1)
        
    def loop(self) :
        rospy.init_node('StereoRosWrapper', anonymous=False)
        while not rospy.is_shutdown() : 
            if self.acquire_frame() :
                self.exportToRosMsgs()
                if cv2.waitKey(4) & 0xFF == ord('w'):
                    break
                self.left_image_topic.publish(self.leftImage)
                self.right_image_topic.publish(self.rightImage)
                if self.left_cam_infos is not None : 
                    self.left_info_topic.publish(self.left_cam_infos)
                if self.right_cam_infos is not None :
                    self.right_info_topic.publish(self.right_cam_infos)
        rospy.logwarn_once("The acquisition loop has excited after request ")


