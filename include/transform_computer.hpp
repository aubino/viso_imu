#pragma once
#include "matcher.hpp"
#include <opencv4/opencv2/calib3d.hpp>
#define InfThresh 100
cv::Mat compute_transform_essential(const cv::Mat& img1, const cv::Mat& img2,cv::Mat cameraMatrix,cv::Mat& R, cv::Mat& t, cv::Mat& Essential);

bool compute_transform(const cv::Mat& img1, const cv::Mat& img2,cv::Mat cameraMatrix,cv::Mat& R, cv::Mat& t, cv::Mat& Essential);