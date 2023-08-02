#ifndef USB_STEREO_H
#define USB_STEREO_H
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/videoio.hpp>
#include <opencv4/opencv2/video.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/calib3d.hpp>
#include <filesystem>
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
        StereoImage(std::string left_file, std::string right_file) ; 
        void setLeftParams(CameraParams params) ;
        void setRightParams(CameraParams params) ;
        std::pair<ImageStamped,ImageStamped> getImages();
        ImageStamped getLeft();
        ImageStamped getRight();
        bool setLeft(const ImageStamped&);
        bool setRight(const ImageStamped&);
        bool rectifyImages(std::pair<ImageStamped,ImageStamped> &) ; 
    private :
        ImageStamped left;
        ImageStamped right;
};

using StereoImageRessource = std::shared_ptr<StereoImage> ;


const std::map<RESOLUTION,float> RES_FPS_MAP({{RESOLUTION(640,240),60},
                                            {RESOLUTION(1280,480),60},
                                            {RESOLUTION(2560,720),60},
                                            {RESOLUTION(2560,960),60}});

void stereoUsbCaptureThread(int usb_channel,StereoImageRessource ressource, bool undistord = false, bool verbose = false,bool debug = false);

#endif