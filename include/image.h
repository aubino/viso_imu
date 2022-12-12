#include <opencv4/opencv2/opencv.hpp>
#include <time.h>
class ImageStamped
{
    cv::Mat image;
    time_t t;
};