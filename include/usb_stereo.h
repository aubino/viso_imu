#pragma once 
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/videoio.hpp>
#include <opencv4/opencv2/video.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/calib3d.hpp>
#include <image.h>
#include <map>
#include <signal.h>
#include <thread>
#include <mutex>
#include <chrono>
#include <memory>
#include <iostream>
#include "camera_params.h"
#include "resolution.h"


class StereoImage
{
    public : 
        std::mutex mutex;
        RESOLUTION resolution;
        CameraParams left_params;
        CameraParams right_params;
        StereoImage(RESOLUTION res);
        StereoImage(CameraParams lp, CameraParams rp);
        void setLeftParams(CameraParams params) ;
        void setRightParams(CameraParams params) ;
        std::pair<ImageStamped,ImageStamped> getImages();
        ImageStamped getLeft();
        ImageStamped getRight();
        bool setLeft(const ImageStamped&);
        bool setRight(const ImageStamped&);
    private :
        ImageStamped left;
        ImageStamped right;
     
};

using StereoImageRessource = std::shared_ptr<StereoImage> ;

const std::map<RESOLUTION,float> RES_FPS_MAP({{RESOLUTION(10,20),10},{RESOLUTION(20,30),60}}); //TODO Take the good values from vendor and put theme here

int stereoUsbCaptureThread(int usb_channel,StereoImageRessource& ressource, bool verbose = false,bool debug = false);