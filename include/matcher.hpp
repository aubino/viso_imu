#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/features2d.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include "detector.hpp"

std::pair<std::vector<cv::Point2f>,std::vector<cv::Point2f>> match_images(const cv::Mat& img1, const cv::Mat& img2,const int MAX_FEATURES,const float GOOD_MATCH_PERCENT, const std::string matching_method="BruteForce-Hamming");

#ifdef DEBUG 
    std::pair<std::vector<cv::Point2f>,std::vector<cv::Point2f>> match_images(const cv::Mat& img1, const cv::Mat& img2,const int MAX_FEATURES,const float GOOD_MATCH_PERCENT, const std::string matching_method="BruteForce-Hamming",cv::String debug_window="matching_debug");
#endif