#include "matcher.hpp"
std::pair<std::vector<cv::Point2f>,std::vector<cv::Point2f>> match_images(const cv::Mat& img1, const cv::Mat& img2,const int MAX_FEATURES,const float GOOD_MATCH_PERCENT, const std::string matching_method)
{
    std::vector<cv::DMatch> matches;
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(matching_method);
    auto kp_des1 = find_key_points_and_descriptors(img1,MAX_FEATURES);
    auto kp_des2 = find_key_points_and_descriptors(img2,MAX_FEATURES);
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
  std::pair<std::vector<cv::Point2f>,std::vector<cv::Point2f>> result; 
  result.first= points1; 
  result.second= points2; 
  return result;
}

#ifdef DEBUG 
    std::pair<std::vector<cv::Point2f>,std::vector<cv::Point2f>> match_images(const cv::Mat& img1, const cv::Mat& img2,const int MAX_FEATURES,const float GOOD_MATCH_PERCENT,cv::String debug_window, const std::string matching_method)
    {
        std::vector<cv::DMatch> matches;
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(matching_method);
    auto kp_des1 = find_key_points_and_descriptors(img1,MAX_FEATURES);
    auto kp_des2 = find_key_points_and_descriptors(img2,MAX_FEATURES);
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
        cv::Mat img_match_drawn;
        cv::drawMatches(img1,kp_des1.first,img2,kp_des2.first,matches,img_match_drawn,100);
        std::pair<std::vector<cv::Point2f>,std::vector<cv::Point2f>> result;
        result.first= points1; 
        result.second= points2;
        cv::imshow(debug_window,img_match_drawn); 
        return result;
    }
#endif
