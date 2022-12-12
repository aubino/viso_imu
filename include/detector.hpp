#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/features2d.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#pragma once 
#define DEBUG 1

std::pair<std::vector<cv::KeyPoint>,cv::Mat> find_key_points_and_descriptors(const cv::Mat& ,const int); 
#ifdef DEBUG 
    #include <opencv4/opencv2/highgui.hpp>
    std::pair<std::vector<cv::KeyPoint>,cv::Mat> find_key_points_and_descriptors(const cv::Mat& img ,const int MAX_FEATURES,std::string window_name);
#endif 




