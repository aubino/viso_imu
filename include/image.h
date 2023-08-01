#ifndef IMAGE_H
#define IMAGE_H
#include <opencv4/opencv2/opencv.hpp>
#include <time.h>
class ImageStamped
{
    public:
    cv::Mat image;
    time_t t;
    ImageStamped()
    {
        return ;
    };
    
    ImageStamped(time_t T,cv::Mat im)
    {
        image = im;
        t = T; 
    };
};
#endif