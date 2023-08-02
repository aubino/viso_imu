#ifndef CLOUD_COMPUTER_H
#define CLOUD_COMPUTER_H
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/calib3d.hpp>
#include <vector>
#include <algorithm>
#include <functional>
#include <time.h>
// #include <pcl-1.10/pcl/point_cloud.h>
// #include <pcl-1.10/pcl/point_types.h>
// #include <pcl-1.10/pcl/point_types_conversion.h>
#include <thread>
#include <mutex>
#include <chrono>
#include <memory>
#include <iostream>
#include "usb_stereo.h"

typedef struct RGBD_ImageStamped
{
    cv::Mat image ; 
    time_t t ; 
    cv::Mat disparity_map ;
    cv::Mat cloud ; 
    
} RGBD_ImageStamped ; 

using RGBDRessource = std::shared_ptr<RGBD_ImageStamped> ;

/// @brief A function to comput ethe disparity-to-depth matrix. See https://docs.opencv.org/3.4/d9/d0c/group__calib3d.html#ga617b1685d4059c6040827800e72ad2b6 for more details
/// @param P1 3x4  Projection matrix of the lest camera
/// @param P2  3x4 Projection matrix of the right camera
/// @return 4×4  disparity-to-depth mapping matrix 
cv::Mat disparity_to_map_matrix(cv::Mat P1, cv::Mat P2) ; 

void stereoCloudComputingThread(StereoImageRessource stereo_image , RGBDRessource rgbd, bool debug = false, bool verbose = false) ;

#endif