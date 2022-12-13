#include "detector.hpp"

std::pair<std::vector<cv::KeyPoint>,cv::Mat> find_key_points_and_descriptors(const cv::Mat& img ,const int MAX_FEATURES)
{
    cv::Mat img_gray;
    cv::cvtColor(img,img_gray,cv::COLOR_BGR2GRAY);
    std::pair<std::vector<cv::KeyPoint>,cv::Mat> result;
    cv::Ptr<cv::Feature2D> orb = cv::ORB::create(MAX_FEATURES);
    orb->detectAndCompute(img_gray,cv::Mat(),result.first,result.second);
    #ifdef DEBUG 
        std::cout<<"Found "<<result.first.size()<<" Key Points in the image"<<std::endl;
    #endif
    return result;
}

#ifdef DEBUG 
    std::pair<std::vector<cv::KeyPoint>,cv::Mat> find_key_points_and_descriptors(const cv::Mat& img ,const int MAX_FEATURES,cv::String window_name)
    {
    cv::Mat img_gray;
    cv::Mat img_gray_drawn;
    cv::cvtColor(img,img_gray,cv::COLOR_BGR2GRAY);
    std::pair<std::vector<cv::KeyPoint>,cv::Mat> result;
    cv::Ptr<cv::Feature2D> orb = cv::ORB::create(MAX_FEATURES);
    orb->detectAndCompute(img_gray,cv::Mat(),result.first,result.second);
    std::cout<<"Found "<<result.first.size()<<" Key Points in the image"<<std::endl;
    cv::drawKeypoints(img_gray,result.first,img_gray_drawn);
    cv::imshow(window_name,img_gray_drawn);
    return result;
     
    }
#endif 