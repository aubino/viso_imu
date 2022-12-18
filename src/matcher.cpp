#include "matcher.hpp"
std::pair<std::vector<cv::Point2f>,std::vector<cv::Point2f>> match_images(const cv::Mat& img1, const cv::Mat& img2,const int MAX_FEATURES,const float GOOD_MATCH_PERCENT, const std::string matching_method)
{
    std::pair<std::vector<cv::Point2f>,std::vector<cv::Point2f>> result;
    std::vector<cv::DMatch> matches;
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(matching_method);
    auto kp_des1 = find_key_points_and_descriptors(img1,MAX_FEATURES);
    auto kp_des2 = find_key_points_and_descriptors(img2,MAX_FEATURES);
    if(kp_des1.first.size()>0  && kp_des2.first.size()>0)
    {
        matcher->match(kp_des1.second, kp_des2.second, matches, cv::Mat());
        std::sort(matches.begin(), matches.end());
        const int numGoodMatches = matches.size() * GOOD_MATCH_PERCENT;
        matches.erase(matches.begin()+numGoodMatches, matches.end());
        std::vector<cv::Point2f> points1, points2;
    for( size_t i = 0; i < matches.size(); i++ )
        {
            points1.push_back( kp_des1.first[ matches[i].queryIdx ].pt );
            points2.push_back( kp_des2.first[ matches[i].trainIdx ].pt );
        }
    result.first= points1; 
    result.second= points2; 
    }
    std::cout<<"Could not find any key point"<<std::endl; 
  return result;
}

#ifdef DEBUG 
    std::pair<std::vector<cv::Point2f>,std::vector<cv::Point2f>> match_images(const cv::Mat& img1, const cv::Mat& img2,const int MAX_FEATURES,const float GOOD_MATCH_PERCENT,cv::String debug_window, const std::string matching_method)
    {
        std::pair<std::vector<cv::Point2f>,std::vector<cv::Point2f>> result;
        std::vector<cv::DMatch> matches;
        cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(matching_method);
        auto kp_des1 = find_key_points_and_descriptors(img1,MAX_FEATURES);
        auto kp_des2 = find_key_points_and_descriptors(img2,MAX_FEATURES);
        if(kp_des1.first.size()>0  && kp_des2.first.size()>0)
        {
            matcher->match(kp_des1.second, kp_des2.second, matches, cv::Mat());
            std::sort(matches.begin(), matches.end());
            const int numGoodMatches = matches.size() * GOOD_MATCH_PERCENT;
            matches.erase(matches.begin()+numGoodMatches, matches.end());
            std::vector<cv::Point2f> points1, points2;
            for( size_t i = 0; i < matches.size(); i++ )
            {
                points1.push_back( kp_des1.first[ matches[i].queryIdx ].pt );
                points2.push_back( kp_des2.first[ matches[i].trainIdx ].pt );
            }
            result.first= points1; 
            result.second= points2;
            cv::Mat img_match_drawn;
            cv::drawMatches(img1,kp_des1.first,img2,kp_des2.first,matches,img_match_drawn,100);
            cv::imshow(debug_window,img_match_drawn); 
        }
        std::cout<<"Could not find any key point"<<std::endl; 
        return result;
    }
#endif

std::pair<std::vector<cv::Point2f>,std::vector<cv::Point2f>> filter_matches_with_disp(std::pair<std::vector<cv::Point2f>,std::vector<cv::Point2f>> matches,cv::Mat intrinsics,double minimum_dist)
{
    std::pair<std::vector<cv::Point2f>,std::vector<cv::Point2f>> result;
    for(int i = 0; i< std::min(matches.first.size(),matches.second.size());i++)
    {
        if(pixel_distance(matches.first[i],matches.second[i])>=minimum_dist)
            {
                result.first.push_back(matches.first[i]) ;
                result.second.push_back(matches.second[i]);
            }
    }
    #ifdef DEBUG 
        std::cout<<"Filter found "<<result.first.size()<<" matches with a minimum pixelic disparity of "<<minimum_dist<<std::endl;
    #endif
    return result;
} 

double pixel_distance(cv::Point2f P1, cv::Point2f P2)
{
    return  cv::norm(P1-P2);//std::sqrt((P1.x-P2.x)*(P1.x-P2.x)+(P1.y-P2.y)*(P1.y-P2.y)) ; 
}