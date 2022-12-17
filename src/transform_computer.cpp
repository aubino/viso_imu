#include "transform_computer.hpp"

cv::Mat compute_transform_essential(const cv::Mat& img1, const cv::Mat& img2,cv::Mat cameraMatrix,cv::Mat& R, cv::Mat& t, cv::Mat& Essential)
{
    //auto points_pairs = filter_matches_with_disp(match_images(img1,img2,300,0.5f),cameraMatrix,1.0);
    auto points_pairs = match_images(img1,img2,300,0.5f);
    cv::Mat points;
    cv::Mat inliers;
    if (points_pairs.first.size()<=7)
    {
        #ifdef DEBUG 
            std::cout<<"Not Enougth matches to compute transform between the two images"<<std::endl;
        #endif
        return points;
    }
    else if(points_pairs.first.size()==7)
    {
        //to process 
        return points;
    }
    else if(points_pairs.first.size()>7)
    {
        cv::Mat mask;
        Essential =  cv::findEssentialMat(points_pairs.first, points_pairs.second,cameraMatrix,cv::RANSAC,0.999, 1,mask);
        #ifdef DEBUG 
            std::cout<<"Essential materix is " << Essential << std::endl;
        #endif
        cv::recoverPose(Essential,points_pairs.first,points_pairs.second,cameraMatrix,R,t,InfThresh, inliers,points);
        return points;
    }
}

bool compute_transform(const cv::Mat& img1, const cv::Mat& img2,cv::Mat cameraMatrix,cv::Mat& R, cv::Mat& t, cv::Mat& Essential)
{
    auto points_pairs = filter_matches_with_disp(match_images(img1,img2,300,0.5f),cameraMatrix,5.0);
    //auto points_pairs = match_images(img1,img2,300,0.5f);
    cv::Mat points;
    cv::Mat inliers;
    if (points_pairs.first.size()<=7)
    {
        #ifdef DEBUG 
            std::cout<<"Not Enougth matches to compute transform between the two images"<<std::endl;
        #endif
        return false;
    }
    else if(points_pairs.first.size()==7)
    {
        //to process 
        return false;
    }
    else if(points_pairs.first.size()>7)
    {
        cv::Mat mask;
        Essential =  cv::findEssentialMat(points_pairs.first, points_pairs.second,cameraMatrix,cv::RANSAC,0.999, 1,mask);
        #ifdef DEBUG 
            std::cout<<"Essential materix is " << Essential << std::endl;
        #endif
        cv::recoverPose(Essential,points_pairs.first,points_pairs.second,cameraMatrix,R,t,InfThresh, inliers,points);
        return true;
    }
}