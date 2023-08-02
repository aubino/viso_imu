#include "cloud_computer.hpp"
#include <condition_variable>
#include <atomic>

void stereoCloudComputingThread(StereoImageRessource stereo_image , RGBDRessource rgbd, bool debug , bool verbose ) 
{
    cv::Ptr<cv::StereoSGBM> stereo_bm_object  = cv::StereoSGBM::create( 0,
                                                                        8,
                                                                        5,
                                                                        0,
                                                                        0,
                                                                        0,
                                                                        0,
                                                                        0,
                                                                        0,
                                                                        0,
                                                                        cv::StereoSGBM::MODE_HH) ;
    std::pair<ImageStamped,ImageStamped> _2im ; 
    std::condition_variable cond ;
    std::unique_lock<std::mutex> ulk{stereo_image->mutex} ; 
    while ((true))
    {
        // cond.wait(stereo_image->mutex.try_lock()) ; 
        cond.wait(ulk) ; 
        stereo_image->rectifyImages(_2im) ; 
        cv::Mat calculated_disparity, float_disparity ; 
        stereo_bm_object->compute(_2im.first.image,_2im.second.image,calculated_disparity) ; 
        calculated_disparity.convertTo(float_disparity,CV_32F) ; 
        cv::Mat Q  = disparity_to_map_matrix(stereo_image->left_params.getProjectionMatrix() , stereo_image->right_params.getProjectionMatrix()) ;
        cv::Mat cloudImage ; 
        cv::reprojectImageTo3D(float_disparity,cloudImage,Q,true) ; 
        rgbd->image = stereo_image->getLeft().image ; 
        rgbd->t  = stereo_image->getLeft().t ;  
        rgbd->disparity_map = calculated_disparity ; 
        rgbd->cloud = cloudImage ; 
    }
    return ;
}

cv::Mat disparity_to_map_matrix(cv::Mat P1, cv::Mat P2) 
{
    cv::Mat Q ; 
    Q.at<double>(0,0) = 1    ; Q.at<double>(0,1) = 0   ; Q.at<double>(0,2) =  0                                       ; Q.at<double>(0,3) = - P1.at<double>(0,2)                                                                    ;
    Q.at<double>(1,0) = 0    ; Q.at<double>(1,1) = 1   ; Q.at<double>(1,2) =  0                                       ; Q.at<double>(1,3) = - P1.at<double>(1,2)                                                                    ;
    Q.at<double>(2,0) = 0    ; Q.at<double>(2,1) = 0   ; Q.at<double>(2,2) =  0                                       ; Q.at<double>(2,3) =   P1.at<double>(0,0)                                                                    ;
    Q.at<double>(3,0) = 0    ; Q.at<double>(3,1) = 0   ; Q.at<double>(3,2) = -(P2.at<double>(0,0)/P2.at<double>(0,3)) ; Q.at<double>(3,3) =   ((P1.at<double>(0,2) - P2.at<double>(0,2)) * P2.at<double>(0,0)) / P2.at<double>(0,3) ;
    return Q ; 
}