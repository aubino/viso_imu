#include "transform_computer.hpp"

cv::Mat compute_transform_essential(const cv::Mat& img1, const cv::Mat& img2,cv::Mat cameraMatrix,cv::Mat& R, cv::Mat& t, cv::Mat& Essential)
{
    auto points_pairs = match_images(img1,img2,300,0.1f);
    if (points_pairs.first.size()<=7)
    {
        #ifdef DEBUG 
            std::cout<<"Not Enougth matches to compute transform between the two images"<<std::endl;
        #endif
    }
    else if(points_pairs.first.size()==7)
    {
        //to process 
    }
    else if(points_pairs.first.size()>7)
    {
        Essential =  cv::findEssentialMat(points_pairs.first, points_pairs.second,cameraMatrix,cv::RANSAC,0.999, 3,cv::Mat());
        cv::Mat points;
        cv::recoverPose(Essential,points_pairs.first,points_pairs.second,cameraMatrix,R,t,InfThresh, cv::noArray(),points);
        return points;
    }
}
