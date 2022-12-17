//#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/features2d.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include "detector.hpp"
#pragma once

std::pair<std::vector<cv::Point2f>,std::vector<cv::Point2f>> match_images(const cv::Mat& img1, const cv::Mat& img2,const int MAX_FEATURES,const float GOOD_MATCH_PERCENT, const std::string matching_method="BruteForce-Hamming");

std::pair<std::vector<cv::Point2f>,std::vector<cv::Point2f>> match_images(const cv::Mat& img1, const cv::Mat& img2,const int MAX_FEATURES,const float GOOD_MATCH_PERCENT,cv::String debug_window, const std::string matching_method="BruteForce-Hamming");

std::pair<std::vector<cv::Point2f>,std::vector<cv::Point2f>> filter_matches_with_disp(std::pair<std::vector<cv::Point2f>,std::vector<cv::Point2f>> matches,cv::Mat intrinsics,double minimum_dist = 0.05); 

double pixel_distance(cv::Point2f P1, cv::Point2f P2); 